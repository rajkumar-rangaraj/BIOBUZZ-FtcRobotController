package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** IntakeGuard: the G407 push-back while the queue is FULL and the pair-jam unjam on the intake current. */
public class IntakeGuardTest {

    static final double DT = 0.02;
    static final double FREE_AMPS = 1.5, JAM_AMPS = 6.0;

    /** A loop driver: reads the current only when the guard says it's due, like the op modes. */
    static class Loop {
        final IntakeGuard g;
        double t = 0;
        double requested = 1.0;
        boolean full = false, freed = false, staged = false, fullBeam = false;
        double amps = FREE_AMPS;
        int currentReads = 0;
        double out;

        Loop(IntakeGuard.Settings s) {
            g = new IntakeGuard(s);
        }

        Loop() {
            this(new IntakeGuard.Settings());
        }

        double step() {
            double a = Double.NaN;
            if (g.currentDue(t, requested)) {
                a = amps;
                currentReads++;
            }
            out = g.update(t, requested, full, freed, a, staged, fullBeam);
            t += DT;
            return out;
        }

        void run(double sec) {
            double end = t + sec;
            while (t < end - 1e-9) {
                step();
            }
        }
    }

    // ---------------- G407 push-back ----------------

    @Test
    public void pushBackPulseWhenTheQueueBecomesFull() {
        Loop l = new Loop();
        assertEquals(1.0, l.step(), 0);
        l.full = true;
        l.staged = l.fullBeam = true;
        assertEquals(BioBuzzConstants.QUEUE_FULL_PUSHBACK_POWER, l.step(), 0);
        assertNotNull(l.g.event());
        assertEquals(IntakeGuard.State.PUSHBACK, l.g.state());
        double start = l.t - DT;
        while (l.t < start + BioBuzzConstants.QUEUE_FULL_PUSHBACK_SEC - 1e-9) {
            assertEquals("reverse for the whole pulse", BioBuzzConstants.QUEUE_FULL_PUSHBACK_POWER, l.step(), 0);
        }
        assertEquals("then the gentle reverse", BioBuzzConstants.QUEUE_FULL_INTAKE_POWER, l.step(), 0);
        assertEquals(IntakeGuard.State.FULL, l.g.state());
        l.run(5.0);
        assertEquals("gentle reverse while FULL and IN is asked", BioBuzzConstants.QUEUE_FULL_INTAKE_POWER, l.out, 0);
        assertTrue("the defaults reverse", BioBuzzConstants.QUEUE_FULL_INTAKE_POWER < 0
                && BioBuzzConstants.QUEUE_FULL_PUSHBACK_POWER < BioBuzzConstants.QUEUE_FULL_INTAKE_POWER);
    }

    @Test
    public void pushBackRunsEvenWithoutAnInRequest() {
        Loop l = new Loop();
        l.requested = 0;
        l.step();
        l.full = true;
        l.staged = l.fullBeam = true;
        assertEquals(BioBuzzConstants.QUEUE_FULL_PUSHBACK_POWER, l.step(), 0);
        l.run(BioBuzzConstants.QUEUE_FULL_PUSHBACK_SEC + 0.05);
        assertEquals("FULL, nothing asked: off", 0, l.out, 0);
    }

    @Test
    public void driverReversePassesThroughWhileFull() {
        Loop l = new Loop();
        l.full = true;
        l.step();
        l.requested = -1.0;
        assertEquals("a stronger reverse during the pulse", -1.0, l.step(), 0);
        l.run(1.0);
        assertEquals(-1.0, l.out, 0);
        l.requested = -0.2;
        assertEquals("unchanged after the pulse", -0.2, l.step(), 0);
    }

    @Test
    public void noCapOrPushBackOnceABallHasLeft() {
        Loop l = new Loop();
        l.freed = true;
        l.full = true;
        l.staged = l.fullBeam = true;
        l.requested = 0.5;
        assertEquals(0.5, l.step(), 0);
        l.run(1.0);
        assertEquals(0.5, l.out, 0);
    }

    @Test
    public void fullProtectionStaysUntilABallHasLeft() {
        Loop l = new Loop();                                     // FULL, a volley started (back-off / opening / hold)
        l.full = true;
        l.staged = l.fullBeam = true;
        l.requested = 1.0 + BioBuzzConstants.FEED_INTAKE_BOOST;
        l.run(1.0);
        assertEquals("no ball out yet: still the gentle reverse", BioBuzzConstants.QUEUE_FULL_INTAKE_POWER, l.out, 0);
        l.full = false;                                          // both beams broken, not FULL: still paused
        assertEquals(0, l.step(), 0);
        l.freed = true;                                          // a ball counted out: room for one more
        assertEquals(1.0 + BioBuzzConstants.FEED_INTAKE_BOOST, l.step(), 0);
    }

    @Test
    public void newPulseEachTimeTheQueueBecomesFull() {
        Loop l = new Loop();
        l.full = true;
        l.staged = l.fullBeam = true;
        l.run(1.0);
        assertEquals(BioBuzzConstants.QUEUE_FULL_INTAKE_POWER, l.out, 0);
        l.full = false;
        l.staged = l.fullBeam = false;
        l.run(0.5);
        assertEquals(1.0, l.out, 0);
        l.full = true;
        l.staged = l.fullBeam = true;
        assertEquals(BioBuzzConstants.QUEUE_FULL_PUSHBACK_POWER, l.step(), 0);
    }

    @Test
    public void fullWithBeamFullClearStopsTheIntakeInsteadOfReversing() {
        Loop l = new Loop();
        l.full = true;
        l.staged = l.fullBeam = true;
        assertEquals(BioBuzzConstants.QUEUE_FULL_PUSHBACK_POWER, l.step(), 0);
        l.fullBeam = false;                                      // the pulse moved ball 4 off beam_full
        assertEquals("stopped, not reversing: the queue re-packs ball 4", 0, l.step(), 0);
        assertEquals(IntakeGuard.State.REPACK, l.g.state());
        l.requested = 0;
        assertEquals(0, l.step(), 0);
        l.requested = -1.0;
        assertEquals("a driver's reverse still passes", -1.0, l.step(), 0);
        l.requested = 1.0;
        l.fullBeam = true;                                       // re-packed: the push-back pulse goes on
        assertEquals(BioBuzzConstants.QUEUE_FULL_PUSHBACK_POWER, l.step(), 0);
        l.run(1.0);
        assertEquals("then the gentle reverse", BioBuzzConstants.QUEUE_FULL_INTAKE_POWER, l.out, 0);
        l.fullBeam = false;
        assertEquals(0, l.step(), 0);
    }

    @Test
    public void ballCrossingTheFullBeamPausesTheIntake() {
        Loop l = new Loop();
        l.staged = l.fullBeam = true;                            // both beams, not yet FULL
        assertEquals(0, l.step(), 0);
        assertEquals(IntakeGuard.State.CROSSING, l.g.state());
        l.requested = -0.4;
        assertEquals(-0.4, l.step(), 0);
        l.fullBeam = false;
        l.requested = 1.0;
        assertEquals(1.0, l.step(), 0);
    }

    // ---------------- pair jam ----------------

    @Test
    public void currentReadAtMostEvery50ms() {
        Loop l = new Loop();
        l.run(1.0);
        assertTrue("about one read per 50-60 ms in 1 s, not 50 (" + l.currentReads + ")",
                l.currentReads >= 14 && l.currentReads <= 21);
        Loop idle = new Loop();
        idle.requested = 0;
        idle.run(1.0);
        assertEquals("no reads while the intake isn't pulling in", 0, idle.currentReads);
        Loop full = new Loop();
        full.full = true;
        full.run(1.0);
        assertEquals("no reads while FULL", 0, full.currentReads);
    }

    @Test
    public void shortCurrentSpikeIsNotAJam() {
        Loop l = new Loop();
        l.run(0.5);
        l.amps = JAM_AMPS;                                       // a start-up / pickup spike shorter than INTAKE_JAM_SEC
        l.run(BioBuzzConstants.INTAKE_JAM_SEC - 0.1);
        l.amps = FREE_AMPS;
        l.run(1.0);
        assertEquals(0, l.g.tries());
        assertEquals(1.0, l.out, 0);
    }

    @Test
    public void currentBelowThresholdIsNotAJam() {
        Loop l = new Loop();
        l.amps = BioBuzzConstants.INTAKE_JAM_AMPS - 0.1;
        l.run(3.0);
        assertEquals(0, l.g.tries());
        assertEquals(1.0, l.out, 0);
    }

    @Test
    public void jamGivesAShortReversePulseThenResumes() {
        Loop l = new Loop();
        l.run(0.2);
        l.amps = JAM_AMPS;
        double jamStart = l.t;
        boolean unjammed = false;
        while (l.t < jamStart + 1.0) {
            if (l.step() == BioBuzzConstants.INTAKE_UNJAM_POWER) {
                unjammed = true;
                break;
            }
        }
        assertTrue(unjammed);
        double detectedAfter = l.t - DT - jamStart;
        assertTrue("detected after INTAKE_JAM_SEC (" + detectedAfter + ")",
                detectedAfter >= BioBuzzConstants.INTAKE_JAM_SEC - 1e-9
                        && detectedAfter <= BioBuzzConstants.INTAKE_JAM_SEC + 0.15);
        assertEquals(1, l.g.tries());
        assertEquals(IntakeGuard.State.UNJAM, l.g.state());
        l.amps = FREE_AMPS;                                      // the pair came apart
        double pulseStart = l.t - DT;
        while (l.t < pulseStart + BioBuzzConstants.INTAKE_UNJAM_SEC - 1e-9) {
            assertEquals(BioBuzzConstants.INTAKE_UNJAM_POWER, l.step(), 0);
        }
        assertEquals("pulls in again", 1.0, l.step(), 0);
        l.run(1.0);
        assertEquals(1.0, l.out, 0);
    }

    @Test
    public void givesUpAfterMaxTriesUntilReleased() {
        Loop l = new Loop();
        l.amps = JAM_AMPS;                                       // wedged for good, no beam changes
        int pulses = 0;
        double prev = 1.0;
        for (int i = 0; i < 400; i++) {
            double p = l.step();
            if (p == BioBuzzConstants.INTAKE_UNJAM_POWER && prev != p) {
                pulses++;
            }
            prev = p;
        }
        assertEquals(BioBuzzConstants.INTAKE_UNJAM_MAX_TRIES, pulses);
        assertTrue(l.g.gaveUp());
        assertEquals(IntakeGuard.State.GAVE_UP, l.g.state());
        assertTrue(l.g.status().contains("gave up"));
        assertEquals("keeps the driver's power, no more unjams", 1.0, l.out, 0);
        int reads = l.currentReads;
        l.run(1.0);
        assertEquals("no current reads after giving up", reads, l.currentReads);

        l.requested = 0;                                         // the driver lets the intake go
        l.step();
        assertFalse(l.g.gaveUp());
        assertEquals(0, l.g.tries());
        l.requested = 1.0;
        boolean pulsed = false;
        for (int i = 0; i < 50 && !pulsed; i++) {
            pulsed = l.step() == BioBuzzConstants.INTAKE_UNJAM_POWER;
        }
        assertTrue("tries again after the release", pulsed);
    }

    @Test
    public void beamChangeResetsTheTries() {
        Loop l = new Loop();
        l.amps = JAM_AMPS;
        int pulses = 0;
        double prev = 1.0;
        for (int i = 0; i < 600; i++) {
            if (i % 40 == 39) {
                l.fullBeam = !l.fullBeam;                        // a ball moves between unjams: progress
            }
            double p = l.step();
            if (p == BioBuzzConstants.INTAKE_UNJAM_POWER && prev != p) {
                pulses++;
            }
            prev = p;
        }
        assertFalse("never gives up while the balls move", l.g.gaveUp());
        assertTrue(pulses > BioBuzzConstants.INTAKE_UNJAM_MAX_TRIES);
    }

    @Test
    public void noJamWatchWhileFullOrDisabled() {
        Loop l = new Loop();
        l.full = true;
        l.staged = l.fullBeam = true;
        l.amps = JAM_AMPS;
        l.run(2.0);
        assertEquals(0, l.g.tries());
        assertEquals(BioBuzzConstants.QUEUE_FULL_INTAKE_POWER, l.out, 0);

        IntakeGuard.Settings s = new IntakeGuard.Settings();
        s.jamDetect = false;
        Loop off = new Loop(s);
        off.amps = JAM_AMPS;
        off.run(2.0);
        assertEquals(0, off.currentReads);
        assertEquals(1.0, off.out, 0);
    }

    @Test
    public void releaseDuringAnUnjamStopsIt() {
        Loop l = new Loop();
        l.amps = JAM_AMPS;
        boolean pulsed = false;
        for (int i = 0; i < 50 && !pulsed; i++) {
            pulsed = l.step() == BioBuzzConstants.INTAKE_UNJAM_POWER;
        }
        assertTrue(pulsed);
        l.requested = 0;
        assertEquals(0, l.step(), 0);
        assertEquals(0, l.g.tries());
    }
}

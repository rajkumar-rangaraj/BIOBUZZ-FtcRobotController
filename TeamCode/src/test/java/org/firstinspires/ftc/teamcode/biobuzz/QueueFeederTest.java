package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Indexer v2 queue (QueueFeeder) against a small simulation of the lane: balls (73 mm) in single file, the gate face at
 * x = 0 (balls move +x toward the flywheel), beam_staged at ball 1's centre and beam_full at ball 4's centre, each beam
 * mounted off the ball centre line so it is blocked over a 42 mm chord (touching balls leave a short gap). The belts
 * carry every ball at power x 600 mm/s; a closed gate (or one still swinging) stops ball 1, balls never overlap, and a
 * ball past the gate is taken by the flywheel. Every step checks that the gate never closes on a ball or while the
 * queue was moving.
 */
public class QueueFeederTest {

    static final double BALL = 73, R = BALL / 2;
    static final double VMAX = 600;                              // mm/s at full power (0.58-0.63 m/s measured)
    static final double TICKS_PER_MM = BioBuzzConstants.QUEUE_TICKS_PER_BALL / BALL;
    static final double STAGED_X = -R, FULL_X = -R - 3 * BALL, ENTRY_X = FULL_X - 60;

    static class Sim {
        final QueueFeeder f;
        final List<Double> balls = new ArrayList<>();
        final List<String> events = new ArrayList<>();
        final List<Double> launchTimes = new ArrayList<>();
        double t = 0;
        double ticks = 0;
        double beamHalf = 21;                                    // beam 30 mm off the ball centre line
        boolean fire = false, intake = false, ready = true, dip = false, aimOk = true;
        boolean stuck = false, stuckStallsMotor = false;         // ball 1 stuck at the gate
        boolean gateCmdOpen = false;
        double gateCmdT = -1e9;
        double gateSwingSec = 0.10;                              // the real paddle: a bit faster than GATE_OPEN_SEC
        int closings = 0;
        int pendingBalls = 0;                                    // balls the intake will feed in
        double pushedAtGateSec = 0, longestPushAtGate = 0;       // ball 1 pressed into the closed gate

        Sim(QueueFeeder.Settings s) {
            f = new QueueFeeder(s);
        }

        Sim() {
            this(new QueueFeeder.Settings());
        }

        Sim preload(int n) {
            for (int i = 0; i < n; i++) {
                balls.add(-R - i * BALL);
            }
            return this;
        }

        boolean beam(double x) {
            for (double c : balls) {
                if (Math.abs(c - x) < beamHalf) {
                    return true;
                }
            }
            return false;
        }

        boolean gateBlocks() {
            return !gateCmdOpen || t - gateCmdT < gateSwingSec;
        }

        void step(double dt) {
            boolean wasOpen = f.gateOpen();
            double prevPower = f.power();
            double p = f.update(t, fire, aimOk, intake, beam(STAGED_X), beam(FULL_X), ready, dip,
                    (int) Math.round(ticks));
            if (f.event() != null) {
                events.add(String.format("%.3f %s", t, f.event()));
            }
            if (wasOpen && !f.gateOpen()) {
                closings++;
                assertEquals("gate closed while the queue moved", 0, prevPower, 0);
                assertFalse("gate closed with the staged beam broken", beam(STAGED_X));
                for (double c : balls) {
                    if (c > -R + 0.5 && c - R < 5) {
                        fail("gate closed on a ball at " + c);
                    }
                }
            }
            if (f.gateOpen() != gateCmdOpen) {
                gateCmdOpen = f.gateOpen();
                gateCmdT = t;
            }
            // physics over this loop
            double v = p * VMAX;
            boolean motorStalled = stuck && stuckStallsMotor && v > 0;
            if (!motorStalled) {
                ticks += v * dt * TICKS_PER_MM;
            }
            Collections.sort(balls, Collections.reverseOrder());  // front (nearest the flywheel) first
            boolean pressing = false;
            for (int i = 0; i < balls.size(); i++) {
                double c = balls.get(i);
                double nx = motorStalled ? c : c + v * dt;
                if (v > 0) {
                    if (i == 0) {
                        if (stuck) {
                            nx = c;
                        } else if (gateBlocks() && c <= -R + 1e-9) {
                            nx = Math.min(nx, -R);
                            pressing = nx >= -R - 1e-9;
                        }
                    } else {
                        nx = Math.min(nx, balls.get(i - 1) - BALL);
                    }
                    nx = Math.max(nx, c);                        // the belts never push a ball backward
                } else {
                    nx = Math.max(nx, -600);
                }
                balls.set(i, nx);
            }
            if (pressing && !gateCmdOpen) {
                pushedAtGateSec += dt;
                longestPushAtGate = Math.max(longestPushAtGate, pushedAtGateSec);
            } else {
                pushedAtGateSec = 0;
            }
            for (int i = balls.size() - 1; i >= 0; i--) {
                if (balls.get(i) > R + 40) {                     // grabbed by the flywheel
                    balls.remove(i);
                    launchTimes.add(t + dt);
                }
            }
            if (intake && pendingBalls > 0) {
                boolean entryClear = true;
                for (double c : balls) {
                    entryClear &= Math.abs(c - ENTRY_X) >= BALL;
                }
                if (entryClear) {
                    balls.add(ENTRY_X);
                    pendingBalls--;
                }
            }
            t += dt;
        }

        void run(double sec, double dt) {
            double end = t + sec;
            while (t < end - 1e-9) {
                step(dt);
            }
        }

        /** Steps until the condition holds; false if it never did within sec. */
        boolean runUntil(double sec, double dt, java.util.function.BooleanSupplier done) {
            double end = t + sec;
            while (t < end - 1e-9) {
                step(dt);
                if (done.getAsBoolean()) {
                    return true;
                }
            }
            return false;
        }

        String log() {
            return String.join("\n", events);
        }
    }

    private static double volley(double dt, Sim s) {
        s.fire = true;
        double start = s.t;
        assertTrue("EMPTY within 2 s:\n" + s.log(), s.runUntil(2.0, dt, s.f::emptied));
        return s.t - start;
    }

    // ---------------- firing ----------------

    @Test
    public void fullVolleyAt60msLoops() {
        Sim s = new Sim().preload(4);
        double sec = volley(0.060, s);
        assertEquals("all 4 launched", 4, s.launchTimes.size());
        assertTrue("4 balls + the EMPTY check in ~1.5 s (" + sec + " s)", sec < 1.8);
        assertTrue("count can only miss balls, never add them: " + s.f.volleyBalls(), s.f.volleyBalls() <= 4);
        assertTrue("most balls counted at 60 ms: " + s.f.volleyBalls() + "\n" + s.log(), s.f.volleyBalls() >= 3);
        double gap = (s.launchTimes.get(3) - s.launchTimes.get(0)) / 3;
        assertEquals("spacing comes from the queue speed: ~0.2 s per ball", BALL / (BioBuzzConstants.QUEUE_FIRE_POWER * VMAX),
                gap, 0.07);
        s.run(1.0, 0.060);
        assertFalse("gate closed after EMPTY (after the back-off)", s.f.gateOpen());
        assertEquals(1, s.closings);
        assertEquals(0, s.f.jams());
    }

    @Test
    public void fullVolleyAt33msLoops() {
        Sim s = new Sim().preload(4);
        double sec = volley(0.033, s);
        assertEquals(4, s.launchTimes.size());
        assertEquals("every ball counted at 33 ms\n" + s.log(), 4, s.f.volleyBalls());
        assertTrue("" + sec, sec < 1.8);
        s.run(1.0, 0.033);
        assertFalse(s.f.gateOpen());
        assertEquals(0, s.f.jams());
    }

    @Test
    public void flywheelReadyOnceThenNoPausePerBall() {
        Sim s = new Sim().preload(4);
        s.ready = false;
        s.fire = true;
        s.run(1.0, 0.05);
        assertFalse("gate stays closed until the flywheel is ready", s.f.gateOpen());
        assertEquals(QueueFeeder.State.WAIT_READY, s.f.state());
        s.ready = true;
        s.step(0.05);
        assertTrue(s.f.gateOpen());
        s.ready = false;                                         // normal small dips: READY flickers, no hold
        assertTrue(s.runUntil(2.0, 0.05, s.f::emptied));
        assertEquals(4, s.launchTimes.size());
        assertFalse(s.log().contains("HOLD"));
    }

    @Test
    public void touchingBallsWithNoBeamGapStillEndOnEmpty() {
        Sim s = new Sim().preload(4);
        s.beamHalf = R;                                          // beam through the ball centres: no gap at all
        volley(0.060, s);
        assertEquals(4, s.launchTimes.size());
        assertTrue("touching balls count as fewer: " + s.f.volleyBalls(), s.f.volleyBalls() >= 1 && s.f.volleyBalls() < 4);
        assertEquals("no false jam from a long broken beam", 0, s.f.jams());
    }

    @Test
    public void flywheelDipHoldsTheQueueAndResumes() {
        Sim s = new Sim().preload(4);
        s.fire = true;
        assertTrue(s.runUntil(1.0, 0.05, () -> s.launchTimes.size() == 1));
        s.dip = true;
        s.ready = false;
        s.step(0.05);
        assertEquals("HOLD at once", 0, s.f.power(), 0);
        assertTrue(s.f.holding());
        int launched = s.launchTimes.size();
        s.run(0.5, 0.05);
        assertEquals("nothing fed while held", 0, s.f.power(), 0);
        assertTrue("gate stays open during a hold", s.f.gateOpen());
        assertEquals(QueueFeeder.State.FIRING, s.f.state());
        assertTrue(s.launchTimes.size() <= launched + 1);        // only a ball already in the flywheel
        s.dip = false;                                           // back up, but not READY yet: still held
        s.step(0.05);
        assertEquals(0, s.f.power(), 0);
        s.ready = true;
        s.step(0.05);
        assertEquals(BioBuzzConstants.QUEUE_FIRE_POWER, s.f.power(), 1e-9);
        assertTrue(s.log().contains("RESUME"));
        assertTrue(s.runUntil(2.0, 0.05, s.f::emptied));
        assertEquals(4, s.launchTimes.size());
        assertEquals("a hold is not a jam", 0, s.f.jams());
    }

    @Test
    public void stalledQueueJamsBacksOffAndRecovers() {
        Sim s = new Sim().preload(4);
        s.stuck = true;
        s.stuckStallsMotor = true;                               // the encoder stops too
        s.fire = true;
        assertTrue(s.runUntil(1.0, 0.05, () -> s.f.state() == QueueFeeder.State.UNJAM));
        assertTrue(s.log(), s.log().contains("JAM 1 (queue stalled"));
        assertEquals(-BioBuzzConstants.QUEUE_UNJAM_POWER, s.f.power(), 1e-9);
        s.stuck = false;                                         // the back-off freed it
        assertTrue(s.runUntil(3.5, 0.05, s.f::emptied));
        assertEquals(4, s.launchTimes.size());
        assertEquals(1, s.f.jams());
        s.run(1.0, 0.05);
        assertFalse(s.f.gateOpen());
    }

    @Test
    public void ballStuckAtTheGateWithSlippingBeltsIsAJamToo() {
        Sim s = new Sim().preload(4);
        s.stuck = true;                                          // belts slip: the encoder keeps counting
        s.fire = true;
        assertTrue(s.runUntil(2.0, 0.05, () -> s.f.jams() == 1));
        assertTrue(s.log(), s.log().contains("ball stuck at the gate"));
        s.stuck = false;
        assertTrue(s.runUntil(3.0, 0.05, s.f::emptied));
        assertEquals(4, s.launchTimes.size());
    }

    @Test
    public void jamsInARowEndJammedAndStop() {
        Sim s = new Sim().preload(4);
        s.stuck = true;
        s.stuckStallsMotor = true;
        s.fire = true;
        assertTrue(s.runUntil(5.0, 0.05, () -> s.f.state() == QueueFeeder.State.JAMMED));
        assertEquals(BioBuzzConstants.QUEUE_MAX_JAMS, s.f.jams());
        assertTrue(s.log().contains("JAMMED"));
        s.run(1.0, 0.05);
        assertEquals("stopped while JAMMED", 0, s.f.power(), 0);
        assertEquals(QueueFeeder.State.JAMMED, s.f.state());
        assertTrue("the stuck ball keeps the gate open (never closed on it)", s.f.gateOpen());
        s.fire = false;                                          // let go: back off, then close
        s.stuck = false;
        assertTrue(s.runUntil(2.0, 0.05, () -> !s.f.gateOpen()));
        assertTrue(s.runUntil(1.0, 0.05, () -> s.f.state() == QueueFeeder.State.IDLE));
    }

    @Test
    public void emptyNeedsBothBeamsClearWhileTheQueueRuns() {
        Sim s = new Sim();                                        // no balls at all
        s.fire = true;
        double start = s.t;
        assertTrue(s.runUntil(2.0, 0.05, s.f::emptied));
        double sec = s.t - start;
        assertTrue("EMPTY after the gate (" + BioBuzzConstants.GATE_OPEN_SEC + " s) + "
                + BioBuzzConstants.QUEUE_EMPTY_CLEAR_SEC + " s of running: " + sec, sec >= BioBuzzConstants.GATE_OPEN_SEC
                + BioBuzzConstants.QUEUE_EMPTY_CLEAR_SEC && sec < 1.1);
        assertEquals(0, s.f.volleyBalls());

        Sim held = new Sim();                                    // clear beams while HELD never count as empty
        held.fire = true;
        assertTrue(held.runUntil(1.0, 0.05, () -> held.f.state() == QueueFeeder.State.FIRING));
        held.dip = true;
        held.ready = false;
        held.run(2.0, 0.05);
        assertTrue(held.f.holding());
        assertFalse(held.f.emptied());
    }

    @Test
    public void lastBallIsCarriedIntoTheFlywheelEvenAtLowFirePower() {
        QueueFeeder.Settings st = new QueueFeeder.Settings();
        st.firePower = 0.25;                                     // Tune's slowest: 0.3 s clear would stop it short
        Sim s = new Sim(st).preload(4);
        s.fire = true;
        assertTrue(s.runUntil(6.0, 0.05, s.f::emptied));
        assertEquals("no ball left in the gate", 4, s.launchTimes.size());
        assertTrue(s.runUntil(2.0, 0.05, () -> !s.f.gateOpen()));
    }

    @Test
    public void ballOnTheFullBeamKeepsTheVolleyGoing() {
        Sim s = new Sim();
        s.balls.add(FULL_X);                                     // one ball far back (beam_full broken)
        s.fire = true;
        assertTrue(s.runUntil(3.0, 0.05, s.f::emptied));
        assertEquals("the far ball was fired, not left behind", 1, s.launchTimes.size());
    }

    @Test
    public void fireReleasedMidVolleyStopsAndClosesSafely() {
        Sim s = new Sim().preload(4);
        s.fire = true;
        assertTrue(s.runUntil(1.0, 0.05, () -> s.launchTimes.size() == 2));
        s.fire = false;
        s.step(0.05);
        assertEquals("queue stops at once", 0, s.f.power(), 0);
        assertTrue(s.log().contains("RELEASED mid-volley"));
        assertTrue(s.runUntil(2.0, 0.05, () -> !s.f.gateOpen()));   // step() checks it never closes on a ball
        assertTrue(s.launchTimes.size() <= 3);
        int left = s.balls.size();
        assertTrue("balls kept in the queue", left >= 1);
        s.intake = true;                                         // loading stages them at the gate again
        assertTrue(s.runUntil(2.0, 0.05, () -> s.beam(STAGED_X) && s.f.power() == 0));
        assertEquals(left, s.balls.size());
        s.intake = false;
        s.fire = true;                                           // the next FIRE finishes them
        assertTrue(s.runUntil(2.0, 0.05, s.f::emptied));
        assertEquals(4, s.launchTimes.size());
    }

    @Test
    public void releaseBeforeTheGateOpensNeverOpensIt() {
        Sim s = new Sim().preload(4);
        s.ready = false;
        s.fire = true;
        s.run(0.3, 0.05);
        s.fire = false;
        s.run(0.3, 0.05);
        assertFalse(s.f.gateOpen());
        assertEquals(0, s.launchTimes.size());
        assertEquals(QueueFeeder.State.IDLE, s.f.state());
    }

    @Test
    public void uncalibratedGateRefusesToFire() {
        QueueFeeder.Settings st = new QueueFeeder.Settings();
        st.gateCalibrated = false;
        Sim s = new Sim(st).preload(4);
        s.fire = true;
        s.run(1.0, 0.05);
        assertFalse(s.f.gateOpen());
        assertEquals(0, s.launchTimes.size());
        assertTrue(s.log().startsWith("0.000 REFUSED"));
        assertEquals("refused once per press", 1, s.events.size());
        assertFalse(QueueFeeder.gateCalibrated(-1, -1));
        assertFalse(QueueFeeder.gateCalibrated(0.4, -1));
        assertFalse(QueueFeeder.gateCalibrated(0.4, 0.4));
        assertTrue(QueueFeeder.gateCalibrated(0.62, 0.35));
    }

    @Test
    public void manualUnjamResetClosesTheGateSafely() {
        Sim s = new Sim().preload(4);
        s.fire = true;
        assertTrue(s.runUntil(1.0, 0.05, () -> s.f.state() == QueueFeeder.State.FIRING && s.f.power() > 0));
        s.f.reset();                                             // TeleOp B
        s.fire = false;
        assertEquals(QueueFeeder.State.CLOSING, s.f.state());
        assertTrue(s.runUntil(2.0, 0.05, () -> !s.f.gateOpen()));
    }

    // ---------------- loading ----------------

    @Test
    public void loadingStopsAtTheStagedBeam() {
        Sim s = new Sim();
        s.pendingBalls = 1;
        s.intake = true;
        assertTrue(s.runUntil(3.0, 0.05, () -> s.beam(STAGED_X)));
        s.step(0.05);
        assertEquals("stops as soon as ball 1 reaches the beam", 0, s.f.power(), 0);
        s.run(3.0, 0.05);                                        // intake still on: only short packing pulses
        assertTrue("ball 1 pressed into the gate only for a moment (" + s.longestPushAtGate + " s)",
                s.longestPushAtGate <= BioBuzzConstants.QUEUE_PACK_ON_SEC + 0.05 + 1e-9);
        assertFalse(s.f.gateOpen());
        assertFalse(s.f.isFull());
        s.intake = false;
        s.run(BioBuzzConstants.QUEUE_LOAD_LINGER_SEC + 0.1, 0.05);
        assertEquals(QueueFeeder.State.IDLE, s.f.state());
        assertEquals(0, s.f.power(), 0);
        assertEquals(1, s.balls.size());
    }

    @Test
    public void loadingWithoutTheIntakeNeverRunsTheQueue() {
        Sim s = new Sim();
        s.balls.add(-200.0);
        s.run(2.0, 0.05);
        assertEquals(0, s.f.power(), 0);
        assertEquals(QueueFeeder.State.IDLE, s.f.state());
    }

    @Test
    public void fullOnlyWithFourPackedBalls() {
        for (double dt : new double[]{0.033, 0.060}) {
            Sim s = new Sim();
            s.pendingBalls = 4;
            s.intake = true;
            boolean sawFull = s.runUntil(20.0, dt, () -> {
                if (s.f.isFull()) {
                    return true;
                }
                return false;
            });
            assertTrue("FULL with 4 balls (dt " + dt + ")\n" + s.log(), sawFull);
            assertEquals("never FULL before ball 4 is in", 4, s.balls.size());
            assertTrue(s.beam(STAGED_X) && s.beam(FULL_X));
            s.step(dt);
            assertEquals(QueueFeeder.State.FULL, s.f.state());
            assertEquals("FULL, intake asked IN: the queue holds ball 4", BioBuzzConstants.QUEUE_HOLD_POWER,
                    s.f.power(), 0);
            s.run(1.0, dt);
            assertEquals(BioBuzzConstants.QUEUE_HOLD_POWER, s.f.power(), 0);
            assertTrue(s.f.isFull());
            assertEquals("the hold doesn't move the packed balls", 4, s.balls.size());
            s.fire = true;                                       // a FULL queue fires as usual
            assertTrue(s.runUntil(3.0, dt, s.f::emptied));
            assertFalse("not FULL once fired", s.f.isFull());
            assertEquals(4, s.launchTimes.size());
        }
    }

    @Test
    public void ballPassingTheFullBeamIsNotFull() {
        Sim s = new Sim().preload(1);                            // ball 1 staged: beam_staged broken all along
        s.pendingBalls = 2;                                      // two more cross beam_full on their way up
        s.intake = true;
        s.run(12.0, 0.05);
        assertEquals(3, s.balls.size());
        assertFalse("3 balls: never FULL\n" + s.log(), s.log().contains("FULL"));
        assertFalse(s.f.isFull());
        assertTrue("packed behind ball 1", s.beam(STAGED_X) && s.balls.contains(-R - 2 * BALL));
    }

    @Test
    public void fireHeldWhileIntakingFiresNewBallsToo() {
        Sim s = new Sim().preload(1);
        s.fire = true;
        assertTrue(s.runUntil(2.0, 0.05, s.f::emptied));
        assertTrue(s.runUntil(2.0, 0.05, () -> s.f.state() == QueueFeeder.State.EMPTY));
        assertFalse(s.f.gateOpen());
        s.pendingBalls = 1;                                      // the driver keeps FIRE and pulls a ball in
        s.intake = true;
        assertTrue(s.runUntil(6.0, 0.05, () -> s.launchTimes.size() == 2));
        assertEquals(2, s.f.volleys());
    }

    // ---------------- FULL hold (G407 push-back) ----------------

    private static Sim fillToFull(double dt) {
        Sim s = new Sim();
        s.pendingBalls = 4;
        s.intake = true;
        assertTrue("FULL\n" + s.log(), s.runUntil(20.0, dt, s.f::isFull));
        return s;
    }

    @Test
    public void holdPowerOnlyWhileFullIdleAndGateClosed() {
        double dt = 0.05;
        Sim s = fillToFull(dt);
        s.step(dt);
        assertEquals(QueueFeeder.State.FULL, s.f.state());
        assertFalse(s.f.gateOpen());
        assertEquals("FULL + gate closed: hold ball 4", BioBuzzConstants.QUEUE_HOLD_POWER, s.f.power(), 0);

        s.intake = false;                                        // intake let go: the hold ends after the linger
        s.run(BioBuzzConstants.QUEUE_LOAD_LINGER_SEC + 0.2, dt);
        assertTrue(s.f.isFull());
        assertEquals("FULL, intake idle, push-back over: no hold", 0, s.f.power(), 0);

        s.intake = true;                                         // the driver pulls IN again: gentle reverse, hold
        s.step(dt);
        assertEquals(BioBuzzConstants.QUEUE_HOLD_POWER, s.f.power(), 0);

        s.fire = true;                                           // firing: never the hold power
        boolean sawOpening = false;
        while (!s.f.emptied() && s.t < 30) {
            s.step(dt);
            QueueFeeder.State st = s.f.state();
            if (s.f.gateOpen() || s.f.firing() || st == QueueFeeder.State.WAIT_READY || st == QueueFeeder.State.CLOSING
                    || st == QueueFeeder.State.UNJAM || st == QueueFeeder.State.JAMMED) {
                sawOpening |= st == QueueFeeder.State.OPENING;
                assertTrue("no hold power while firing (" + st + ")",
                        s.f.power() != BioBuzzConstants.QUEUE_HOLD_POWER);
            }
        }
        assertTrue(sawOpening);
        assertTrue(s.f.emptied());
        assertFalse(s.f.isFull());
    }

    @Test
    public void holdCoversThePushBackPulseEvenWithoutTheIntake() {
        QueueFeeder.Settings st = new QueueFeeder.Settings();
        st.loadLingerSec = 0.05;
        st.pushbackSec = 0.4;
        Sim s = new Sim(st);
        s.pendingBalls = 4;
        s.intake = true;
        assertTrue(s.runUntil(20.0, 0.05, s.f::isFull));
        s.intake = false;
        s.step(0.05);                                            // FULL entered this loop or the last one
        assertEquals("the push-back pulse: hold", BioBuzzConstants.QUEUE_HOLD_POWER, s.f.power(), 0);
        s.run(0.5, 0.05);
        assertEquals("pulse over, intake idle: no hold", 0, s.f.power(), 0);
    }

    @Test
    public void zeroHoldPowerKeepsTheOldStop() {
        QueueFeeder.Settings st = new QueueFeeder.Settings();
        st.holdPower = 0;
        Sim s = new Sim(st);
        s.pendingBalls = 4;
        s.intake = true;
        assertTrue(s.runUntil(20.0, 0.05, s.f::isFull));
        s.run(1.0, 0.05);
        assertEquals(0, s.f.power(), 0);
    }
}

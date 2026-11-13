package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.auto.VolleyGate.Verdict;
import org.firstinspires.ftc.teamcode.biobuzz.BallCounter;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants;
import org.junit.Test;

public class VolleyGateTest {

    private static final double WAIT = BioBuzzHiveAutoBase.EVIDENCE_FRESH_SEC;
    private static final double STOP = BioBuzzHiveAutoBase.CELL_LOST_STOP_SEC;

    @Test
    public void goesWhileTheCameraKeepsSeeingIt() {
        VolleyGate g = new VolleyGate(10.0, WAIT, STOP);
        for (double t = 10.0; t < 13.0; t += 0.02) {
            assertEquals(Verdict.GO, g.update(t, t - 0.05, false));   // a fresh frame every loop
        }
        assertEquals(0, g.waits());
        assertEquals(13.0, g.feederTime(13.0), 1e-9);
    }

    @Test
    public void aBallCrossingThePictureDoesNotStopTheWindmill() {
        VolleyGate g = new VolleyGate(10.0, WAIT, STOP);
        assertEquals(Verdict.GO, g.update(10.1, -1, false));            // tags hidden ~0.1-0.25 s
        assertEquals(Verdict.GO, g.update(10.25, -1, false));
        assertEquals(Verdict.GO, g.update(10.3, 10.3, false));
        assertEquals(0, g.waits());
    }

    @Test
    public void waitsAtOnceThenGoesOnAndTheFeederClockSkipsTheWait() {
        VolleyGate g = new VolleyGate(10.0, WAIT, STOP);
        assertEquals(Verdict.GO, g.update(10.2, -1, false));
        assertEquals(Verdict.WAIT, g.update(10.35, -1, false));         // 0.35 s without it: the windmill stops
        assertEquals(Verdict.WAIT, g.update(10.5, -1, false));
        assertEquals(10.35, g.feederTime(10.5), 1e-9);                  // the feeder's clock stands still
        assertEquals(0.15, g.waitedSec(10.5), 1e-9);                    // the wait going on counts
        assertEquals(Verdict.GO, g.update(10.55, 10.53, false));        // seen again: go on
        assertEquals(1, g.waits());
        assertEquals(0.2, g.waitedSec(10.55), 1e-9);
        assertEquals(10.8, g.feederTime(11.0), 1e-9);
    }

    @Test
    public void stopsWhenMissingTooLongEvenAfterAnEarlierWait() {
        VolleyGate g = new VolleyGate(10.0, WAIT, STOP);
        assertEquals(Verdict.WAIT, g.update(10.4, -1, false));
        assertEquals(Verdict.WAIT, g.update(10.6, -1, false));
        assertEquals(Verdict.STOP, g.update(10.61, -1, false));         // 0.61 s: the CELL tipped (stickers turn away)
    }

    @Test
    public void framesThatDisagreeDoNotCountAsSeen() {
        VolleyGate g = new VolleyGate(10.0, WAIT, STOP);
        // the CELL is still in view but reads "not sure" or DOWN: the caller passes -1, the evidence ages
        assertEquals(Verdict.GO, g.update(10.2, -1, false));
        assertEquals(Verdict.WAIT, g.update(10.4, -1, false));
        assertEquals(Verdict.STOP, g.update(10.7, -1, false));
    }

    @Test
    public void aCellGoingDownStopsAtOnce() {
        VolleyGate g = new VolleyGate(10.0, WAIT, STOP);
        assertEquals(Verdict.STOP, g.update(10.05, 10.05, true));
    }

    @Test
    public void staleEvidenceAtTheStartStopsBeforeTheFirstPocket() {
        VolleyGate g = new VolleyGate(9.3, WAIT, STOP);                // decided on a frame 0.5 s old, nothing since
        assertEquals(Verdict.WAIT, g.update(9.8, -1, false));
        assertEquals(Verdict.STOP, g.update(9.95, -1, false));
    }

    /**
     * The op mode runs the ball counter on the feeder's clock: a flywheel dip while the windmill waits for the camera
     * (its pocket still open, FEEDING) is not counted, so it can't make the robot look empty (G407); the pocket's own
     * ball after the wait still counts, once.
     */
    @Test
    public void aDipWhileTheWindmillWaitsIsNotCountedAndThePocketIsKept() {
        double target = 1400, low = target - 4 * BioBuzzConstants.AUTO_SHOT_DIP_TPS;
        VolleyGate g = new VolleyGate(10.0, WAIT, STOP);
        BallCounter feederClock = BallCounter.forAuto(), wallClock = BallCounter.forAuto();
        // frames taken 5 ms before the loop: seen up to 10.10 (i 5), WAIT from 10.40 (i 20), a dip 10.46-10.60
        // (i 23-30), seen again from 10.66 (i 33)
        for (int i = 0; i <= 50; i++) {
            double t = 10.0 + i * 0.02;
            Verdict v = g.update(t, i <= 5 || i >= 33 ? t - 0.005 : -1, false);
            assertEquals("i " + i, i >= 20 && i <= 32 ? Verdict.WAIT : Verdict.GO, v);
            double velocity = i >= 23 && i <= 30 ? low : target;
            feederClock.update(g.feederTime(t), true, target, velocity);
            wallClock.update(t, true, target, velocity);
        }
        assertEquals("on the wall clock it would count", 1, wallClock.balls());
        assertEquals("on the feeder's clock it doesn't", 0, feederClock.balls());
        for (int i = 51; i <= 65; i++) {                                // the pocket's ball, after the wait
            double t = 10.0 + i * 0.02;
            assertEquals(Verdict.GO, g.update(t, t, false));
            feederClock.update(g.feederTime(t), true, target, i <= 56 ? low : target);
        }
        assertEquals(1, feederClock.balls());
    }
}

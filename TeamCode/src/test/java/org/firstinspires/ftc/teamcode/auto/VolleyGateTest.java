package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.auto.VolleyGate.Verdict;
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
        assertEquals(Verdict.GO, g.update(10.55, 10.53, false));        // seen again: go on
        assertEquals(1, g.waits());
        assertEquals(0.2, g.waitedSec(), 1e-9);
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
}

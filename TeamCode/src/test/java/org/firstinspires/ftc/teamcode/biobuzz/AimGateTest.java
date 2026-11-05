package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** TeleOp aim gate: hold a ready shot while aimed badly or just lost, never forever. */
public class AimGateTest {

    private static AimGate gate() {
        return new AimGate(2.0, 0.75, 1.5, 0.05);
    }

    @Test
    public void aimedAndStillFires() {
        AimGate g = gate();
        assertTrue(g.update(0, true, true, true, 1.2, 0));
        assertEquals(AimGate.State.AIMED, g.state());
    }

    @Test
    public void badAimOrTurningHolds() {
        AimGate g = gate();
        assertFalse(g.update(0, true, true, true, 6.0, 0));
        assertEquals(AimGate.State.HOLD_AIM, g.state());
        assertTrue(g.holding());
        assertFalse(g.update(0.1, true, true, true, 1.0, 0.12));    // aimed but still turning
        assertEquals(AimGate.State.HOLD_TURNING, g.state());
        assertTrue(g.update(0.2, true, true, true, 1.0, 0.0));
    }

    @Test
    public void oneEmptyFrameDoesNotReleaseAHeldShot() {
        AimGate g = gate();
        assertFalse(g.update(0.00, true, true, true, 8.0, 0));
        assertFalse(g.update(0.05, true, true, false, 0, 0));       // CELL lost: still held
        assertEquals(AimGate.State.HOLD_LOST, g.state());
        assertFalse(g.update(0.10, true, true, true, 8.0, 0));      // back, still off
        assertFalse(g.update(0.70, true, true, false, 0, 0));       // lost 0.6 s: still held
        assertTrue(g.update(0.90, true, true, false, 0, 0));        // lost 0.8 s: the driver's call now
        assertEquals(AimGate.State.NO_CELL, g.state());
    }

    @Test
    public void neverSeenMeansNoGate() {
        AimGate g = gate();
        assertTrue(g.update(0, true, true, false, 0, 0.4));
        assertEquals(AimGate.State.NO_CELL, g.state());
    }

    @Test
    public void heldShotGivesWayAfterTheTimeoutAndEachBallEarnsItsOwn() {
        AimGate g = gate();                                         // e.g. knocked camera: reads 5 deg when on target
        assertFalse(g.update(0.0, true, true, true, 5.0, 0));
        assertFalse(g.update(1.4, true, true, true, 5.0, 0));
        assertTrue(g.update(1.5, true, true, true, 5.0, 0));
        assertTrue(g.gaveWay());
        g.ballFed();
        assertFalse(g.update(1.6, true, true, true, 5.0, 0));       // next ball: held again
        assertTrue(g.update(3.1, true, true, true, 5.0, 0));
    }

    @Test
    public void readyFlickerDoesNotRestartTheGiveWayClock() {
        AimGate g = gate();                                         // reviews: READY flickers while the speed hunts
        assertFalse(g.update(0.0, true, false, true, 5.0, 0));      // not ready yet: the clock hasn't started
        assertFalse(g.update(0.5, true, true, true, 5.0, 0));       // first ready ball held back: clock starts
        for (int i = 1; i < 70; i++) {                              // READY on/off every loop until 1.88 s
            assertFalse(g.update(0.5 + i * 0.02, true, i % 2 == 0, true, 5.0, 0));
        }
        assertTrue(g.update(2.0, true, true, true, 5.0, 0));        // 1.5 s after the hold started
        assertTrue(g.gaveWay());
        assertFalse(g.update(2.1, false, true, true, 5.0, 0));      // FIRE released
        assertFalse(g.update(2.2, true, true, true, 5.0, 0));       // pressed again: a fresh 1.5 s
        assertFalse(g.update(3.6, true, true, true, 5.0, 0));
        assertTrue(g.update(3.7, true, true, true, 5.0, 0));
    }

    @Test
    public void spotHoldWaitsForTheDistanceButGivesWayToo() {
        AimGate g = gate();
        assertFalse(g.update(0.0, true, true, true, 0.5, 0, true));  // aimed, but X still driving to the spot
        assertEquals(AimGate.State.HOLD_SPOT, g.state());
        assertTrue(g.holding());
        assertFalse(g.update(1.0, true, true, true, 0.5, 0, true));
        assertTrue(g.update(1.5, true, true, true, 0.5, 0, true));   // blocked spot (robot in the way): goes anyway
        assertTrue(g.gaveWay());
        g.ballFed();
        assertTrue(g.update(1.6, true, true, true, 0.5, 0, false));  // on the spot and aimed
        assertEquals(AimGate.State.AIMED, g.state());
    }

    @Test
    public void onTheSpotTheHeadingIsStillChecked() {
        AimGate g = gate();
        assertFalse(g.update(0.0, true, true, true, 1.7, 0.10, false));  // distance right, X still turning
        assertEquals(AimGate.State.HOLD_TURNING, g.state());
        assertTrue(g.update(0.1, true, true, true, 1.2, 0.0, false));
    }

    @Test
    public void offMeansAlwaysAllowed() {
        AimGate g = gate();
        g.setEnabled(false);
        assertTrue(g.update(0, true, true, true, 30.0, 0.5));
        assertEquals(AimGate.State.OFF, g.state());
        assertTrue(g.update(0.1, true, true, true, 0.0, 0.0, true));    // no SHOT SPOT hold either
        assertEquals(AimGate.State.OFF, g.state());
    }
}

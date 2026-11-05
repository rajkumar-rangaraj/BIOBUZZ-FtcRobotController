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
    public void giveWayCountsOnlyReadyHeldTime() {
        AimGate g = gate();
        assertFalse(g.update(0.0, true, true, true, 5.0, 0));
        assertFalse(g.update(1.0, true, false, true, 5.0, 0));      // flywheel not ready: nothing is held back
        assertFalse(g.update(1.1, true, true, true, 5.0, 0));       // ready again: the timer restarts
        assertFalse(g.update(2.5, true, true, true, 5.0, 0));
        assertTrue(g.update(2.6, true, true, true, 5.0, 0));
        assertFalse(g.update(2.7, false, true, true, 5.0, 0));      // FIRE released
        assertFalse(g.update(2.8, true, true, true, 5.0, 0));       // pressed again: a fresh 1.5 s
    }

    @Test
    public void offMeansAlwaysAllowed() {
        AimGate g = gate();
        g.setEnabled(false);
        assertTrue(g.update(0, true, true, true, 30.0, 0.5));
        assertEquals(AimGate.State.OFF, g.state());
    }
}

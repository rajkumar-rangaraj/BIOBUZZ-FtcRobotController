package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Auto shot speed from the camera distance, with presets, the driver's manual choice and the FIRE latch. */
public class ShotSpeedTest {

    private static final double NEAR = 1450, FAR = 1570, NONE = Double.NaN;
    private static final String A = "BLUE_REAR", B = "BLUE_AUDIENCE";

    private static ShotSpeed speed() {
        return new ShotSpeed(new double[]{30, 40, 56}, new double[]{1440, 1465, 1585}, 3.0, 10, 0.5);
    }

    /** One CELL (A) in view at distance d, or none when d is NaN. */
    private static double see(ShotSpeed s, double t, double d, boolean fire, double preset) {
        return s.update(t, Double.isNaN(d) ? null : A, d, fire, preset);
    }

    @Test
    public void tableInterpolatesAndClampsAtTheEnds() {
        ShotSpeed s = speed();
        assertEquals(1440, s.tableSpeed(20), 1e-9);
        assertEquals(1440, s.tableSpeed(30), 1e-9);
        assertEquals(1452.5, s.tableSpeed(35), 1e-9);
        assertEquals(1525, s.tableSpeed(48), 1e-9);
        assertEquals(1585, s.tableSpeed(70), 1e-9);
    }

    @Test
    public void standardTableMatchesTheDriversOneWheelShots() {
        ShotSpeed s = ShotSpeed.standard();
        assertEquals(1445, s.tableSpeed(33), 5);                // 7 Nov logs: 1435-1450 at 32-33 in
        assertEquals(1440, s.tableSpeed(40), 5);                // 10 Nov labels: IN balls 10-26 under the old 1465
        assertEquals(1590, s.tableSpeed(57), 10);               // 1585 at 57 in
        double prev = 0;
        for (int d = 20; d <= 80; d++) {                         // never goes down with distance
            assertTrue(s.tableSpeed(d) >= prev);
            prev = s.tableSpeed(d);
        }
    }

    @Test
    public void autoSpeedFollowsTheCameraDistance() {
        ShotSpeed s = speed();
        assertEquals(1465, see(s, 0.0, 40, false, NEAR), 1e-9);
        assertEquals(ShotSpeed.Source.AUTO, s.source());
        assertEquals(1525, see(s, 0.1, 48, false, NEAR), 1e-9);
        assertEquals(48, s.lastDistanceIn(), 1e-9);
        assertEquals(A, s.cell());
    }

    @Test
    public void cameraJitterDoesNotMoveTheTarget() {
        ShotSpeed s = speed();
        double first = see(s, 0.0, 48, false, NEAR);            // 1525
        for (int i = 1; i <= 20; i++) {
            double d = 48 + (i % 2 == 0 ? 1.0 : -1.0);          // +/-1 in = +/-7.5 t/s: inside the 10 t/s hysteresis
            assertEquals(first, see(s, i * 0.05, d, false, NEAR), 1e-9);
        }
        assertEquals(1550, see(s, 2.0, 51.3, false, NEAR), 1e-9);    // a real move (+25 t/s) does
    }

    @Test
    public void twoCellsInViewDoNotMakeTheSpeedHunt() {
        // 6 Nov logs: REAR at 30.6 in (4 tags) and AUDIENCE at 65.8 in (3 tags), the best CELL flipping every few frames
        ShotSpeed s = speed();
        assertEquals(1440, s.update(0.0, A, 30.6, false, NEAR), 1e-9);
        for (int i = 1; i <= 60; i++) {
            boolean other = i % 3 != 0;                          // B wins two frames of every three
            double base = other ? s.update(i * 0.05, B, 65.8, false, NEAR) : s.update(i * 0.05, A, 30.6, false, NEAR);
            assertEquals(1440, base, 1e-9);
            assertEquals(ShotSpeed.Source.AUTO, s.source());
        }
        assertEquals(A, s.cell());
    }

    @Test
    public void followsAnotherCellOnceTheFirstIsGone() {
        ShotSpeed s = speed();
        assertEquals(1440, s.update(0.0, A, 30, false, NEAR), 1e-9);
        assertEquals(1440, s.update(0.3, B, 56, false, NEAR), 1e-9);    // A seen 0.3 s ago: still A
        assertEquals(1440, s.update(0.5, B, 56, false, NEAR), 1e-9);
        assertEquals(1585, s.update(0.55, B, 56, false, NEAR), 1e-9);   // A gone for more than 0.5 s: B
        assertEquals(B, s.cell());
        assertEquals(56, s.lastDistanceIn(), 1e-9);
        assertEquals(1585, s.update(0.6, A, 30, false, NEAR), 1e-9);    // and now A is the other one
    }

    @Test
    public void speedIsLatchedWhileFireIsHeld() {
        ShotSpeed s = speed();
        assertEquals(1465, see(s, 0.0, 40, true, NEAR), 1e-9);
        assertEquals(1465, see(s, 0.1, 56, true, NEAR), 1e-9);  // the robot moved, but the burst keeps its speed
        assertEquals(1465, see(s, 0.2, NONE, true, NEAR), 1e-9);
        assertEquals(ShotSpeed.Source.AUTO, s.source());
        assertEquals(1585, see(s, 0.3, 56, false, NEAR), 1e-9); // FIRE released: follows the camera again
    }

    @Test
    public void aCellSeenOnlyDuringABurstDoesNotChangeItsSpeed() {
        ShotSpeed s = speed();
        assertEquals(NEAR, see(s, 0.0, NONE, true, NEAR), 1e-9);
        assertEquals(NEAR, see(s, 0.1, 56, true, NEAR), 1e-9);
        assertEquals(ShotSpeed.Source.PRESET, s.source());
        assertEquals(1585, see(s, 0.2, 56, false, NEAR), 1e-9);
    }

    @Test
    public void lostCellKeepsTheSpeedForAWhileThenUsesThePreset() {
        ShotSpeed s = speed();
        assertEquals(1525, see(s, 0.0, 48, false, NEAR), 1e-9);
        assertEquals(1525, see(s, 0.4, NONE, false, NEAR), 1e-9);
        assertEquals(ShotSpeed.Source.AUTO, s.source());        // a short drop-out is still AUTO on screen
        assertEquals(1525, see(s, 2.9, NONE, false, NEAR), 1e-9);
        assertEquals(ShotSpeed.Source.AUTO_KEPT, s.source());
        assertEquals(NEAR, see(s, 3.1, NONE, false, NEAR), 1e-9);
        assertEquals(ShotSpeed.Source.PRESET, s.source());
        assertNull(s.cell());
        assertEquals(1525, see(s, 3.2, 48, false, NEAR), 1e-9); // seen again: no hysteresis against the preset
    }

    @Test
    public void noCameraAtAllUsesThePresets() {
        ShotSpeed s = speed();
        assertEquals(NEAR, see(s, 0.0, NONE, false, NEAR), 1e-9);
        assertEquals(FAR, see(s, 1.0, NONE, false, FAR), 1e-9);
        assertEquals(ShotSpeed.Source.PRESET, s.source());
    }

    @Test
    public void manualPresetWinsUntilTheDriverAsksForAutoAgain() {
        ShotSpeed s = speed();
        see(s, 0.0, 40, false, NEAR);
        s.setManual(true);
        assertEquals(FAR, see(s, 0.1, 40, false, FAR), 1e-9);
        assertEquals(ShotSpeed.Source.MANUAL, s.source());
        assertTrue(s.manual());
        s.setManual(false);
        assertEquals(1465, see(s, 0.2, 40, false, FAR), 1e-9);
        assertFalse(s.manual());
    }

    @Test
    public void aManualChoiceDuringABurstTakesEffectAtOnce() {
        ShotSpeed s = speed();
        assertEquals(1465, see(s, 0.0, 40, true, NEAR), 1e-9);
        s.setManual(true);                                       // driver presses FAR while holding FIRE
        assertEquals(FAR, see(s, 0.1, 40, true, FAR), 1e-9);
        assertEquals(FAR, see(s, 0.2, 40, true, FAR), 1e-9);
    }

    @Test
    public void autoOffInInitMeansPresetsOnly() {
        ShotSpeed s = speed();
        s.setAutoEnabled(false);
        assertEquals(NEAR, see(s, 0.0, 48, false, NEAR), 1e-9);
        assertEquals(ShotSpeed.Source.PRESET, s.source());
    }

    @Test(expected = IllegalArgumentException.class)
    public void tableMustIncrease() {
        new ShotSpeed(new double[]{30, 30}, new double[]{1440, 1450}, 3, 10, 0.5);
    }
}

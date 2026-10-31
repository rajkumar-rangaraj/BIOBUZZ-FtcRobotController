package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Saved flower-arm calibration: file format and the guided-setup math. */
public class ArmCalibrationTest {

    @Test
    public void formatParseRoundTrip() {
        double[] v = ArmCalibration.parse(ArmCalibration.format(0.712, -1));
        assertNotNull(v);
        assertEquals(0.712, v[0], 1e-4);
        assertEquals(-1, (int) v[1]);
    }

    @Test
    public void parseRejectsBadFiles() {
        assertNull(ArmCalibration.parse(""));
        assertNull(ArmCalibration.parse("deploy=abc\ndirection=1"));
        assertNull(ArmCalibration.parse("deploy=0.9\ndirection=1"));       // up position past 1.0
        assertNull(ArmCalibration.parse("deploy=0.3\ndirection=2"));
    }

    @Test
    public void levelFitGivesValidDownAndUpEitherWay() {
        // arm fitted level at 0.50, then ~90 degrees (0.30) down in either servo direction
        for (double down : new double[]{0.20, 0.80, 0.26, 0.74}) {
            double[] r = ArmCalibration.fromDownPosition(down);
            assertNotNull("down " + down, r);
            int sign = down > 0.5 ? 1 : -1;
            assertEquals(down + sign * 0.002, r[0], 1e-9);               // 0.002 further into the pads
            assertEquals(-sign, (int) r[1]);                              // up is the other way
            double up = ArmCalibration.stowPosition(r[0], (int) r[1]);
            assertTrue(up >= 0 && up <= 1);
        }
    }

    @Test
    public void tooLittleMovementIsRefused() {
        assertNull(ArmCalibration.fromDownPosition(0.45));
        assertNull(ArmCalibration.fromDownPosition(0.60));
    }
}

package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Pinpoint Setup math: pod checks, offsets from an in-place spin, saved file format. */
public class PinpointCalibrationTest {
    private static final double TPM = PinpointCalibration.TICKS_PER_MM_4_BAR;

    /** Raw ticks a pod reads when the robot spins theta (rad, CCW +) about its centre. */
    private static double[] spinTicks(double xLeftMm, double yFwdMm, double theta) {
        return new double[]{-xLeftMm * theta * TPM, yFwdMm * theta * TPM};
    }

    @Test
    public void offsetsRecoveredForPodsAnywhere() {
        double[][] pods = {{104, -164}, {-120, -60}, {0, 90}, {150, 150}};
        for (double[] p : pods) {
            for (double theta : new double[]{4 * Math.PI, -4 * Math.PI}) {      // 2 turns either way
                double[] t = spinTicks(p[0], p[1], theta);
                double[] off = PinpointCalibration.offsetsFromSpin(t[0], t[1], theta, TPM);
                assertEquals(p[0], off[0], 0.01);
                assertEquals(p[1], off[1], 0.01);
            }
        }
    }

    @Test
    public void swappedPodCablesAreCaught() {
        assertNull(PinpointCalibration.forwardCheck(12000, 300));
        assertNotNull(PinpointCalibration.forwardCheck(300, 12000));            // forward push moved the Y pod
        assertNull(PinpointCalibration.leftCheck(-200, -12000));                // reversed Y is fine here
        assertNotNull(PinpointCalibration.leftCheck(12000, 200));
        assertTrue(PinpointCalibration.forwardCheck(100, 50).contains("barely moved"));
    }

    @Test
    public void fileRoundTripAndRejectsNonsense() {
        PinpointCalibration c = new PinpointCalibration(true, false, -118.4, 62.0, true);
        PinpointCalibration r = PinpointCalibration.parse(c.format());
        assertNotNull(r);
        assertTrue(r.xReversed);
        assertEquals(-118.4, r.xPodLeftMm, 0.05);
        assertEquals(62.0, r.yPodForwardMm, 0.05);
        assertNull(PinpointCalibration.parse("x_reversed=true\ny_reversed=false\nx_pod_left_mm=900\ny_pod_forward_mm=0"));
        assertNull(PinpointCalibration.parse("garbage"));
    }
}

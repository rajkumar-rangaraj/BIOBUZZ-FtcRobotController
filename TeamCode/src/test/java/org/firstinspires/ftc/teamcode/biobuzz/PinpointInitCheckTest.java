package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class PinpointInitCheckTest {

    @Test
    public void aStillPinpointIsFine() {
        PinpointInitCheck c = new PinpointInitCheck(1, 3);
        for (int k = 0; k < 400; k++) {                                // 20 s of INIT, both gyros a little noisy
            assertFalse(c.update(true, 0.02 * Math.sin(k), 10 + 0.03 * Math.cos(k)));
        }
    }

    @Test
    public void aDriftingPinpointIsRecalibrated() {
        PinpointInitCheck c = new PinpointInitCheck(1, 3);
        int k = 0;
        while (!c.update(true, 0.5 * k * 0.05, 10)) {                 // 0.5 deg/s, the IMU still
            k++;
            assertTrue("found within 3 s", k < 60);
        }
        assertFalse("starts over", c.update(true, 0.5 * (k + 1) * 0.05, 10));
    }

    @Test
    public void movingTheRobotStartsTheCheckOver() {
        PinpointInitCheck c = new PinpointInitCheck(1, 3);
        c.update(true, 0, 0);
        for (int k = 1; k <= 40; k++) {                                // turned 40 deg by hand, both gyros agree
            assertFalse(c.update(true, k, k));
        }
        for (int k = 41; k <= 180; k++) {                              // on to 180: the IMU flips at 90 meanwhile
            double imu = k > 90 ? k - 180 : k;
            assertFalse("k=" + k, c.update(true, MecanumMath.normalizeDegrees(k), imu));
        }
    }

    @Test
    public void anImuFlippingEveryLoopAtPlusMinus90DoesNotHideADrift() {
        // final review 10 Nov: each flip restarted the comparison, so the drift was never seen
        PinpointInitCheck c = new PinpointInitCheck(1, 3);
        c.update(true, 0, 89.9);
        boolean found = false;
        for (int k = 1; k <= 20 && !found; k++) {                      // the Pinpoint drifts 0.1 deg a loop
            found = c.update(true, 0.1 * k, k % 2 == 0 ? 89.9 : -89.9);
        }
        assertTrue(found);
    }

    @Test
    public void anImuFlippingEveryLoopWithAStillPinpointIsFine() {
        PinpointInitCheck c = new PinpointInitCheck(1, 3);
        for (int k = 0; k < 200; k++) {
            assertFalse(c.update(true, 0.01 * Math.sin(k), k % 2 == 0 ? 89.95 : -89.95));
        }
    }

    @Test
    public void noReadingWhileThePinpointCalibrates() {
        PinpointInitCheck c = new PinpointInitCheck(1, 3);
        c.update(true, 0, 0);
        assertFalse(c.update(false, Double.NaN, 0));
        assertFalse("new reference after calibrating", c.update(true, 25, 0));
        assertFalse(c.update(true, 25.5, 0));
        assertTrue(c.update(true, 26.2, 0));
    }
}

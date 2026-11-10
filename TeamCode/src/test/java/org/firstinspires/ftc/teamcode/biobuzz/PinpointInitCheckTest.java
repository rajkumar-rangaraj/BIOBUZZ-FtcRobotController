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
        assertFalse("an IMU yaw flip is not a Pinpoint drift", c.update(true, 40, 40 - 180));
        assertFalse(c.update(true, 40, -140));
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

package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/** Pinpoint field readings -> robot-relative distances, and the switch to wheel-encoder steps. */
public class OdometryAccumulatorTest {
    private static final double EPS = 1e-6;

    @Test
    public void straightAtStartHeading() {
        OdometryAccumulator o = new OdometryAccumulator();
        for (int i = 0; i <= 12; i++) {
            o.fieldUpdate(-i, 0, 0);                 // backing off the wall 12 in (LEAVE)
        }
        assertEquals(-12, o.forwardInches(), EPS);
        assertEquals(0, o.strafeRightInches(), EPS);
    }

    @Test
    public void strafeRightIsFieldMinusY() {
        OdometryAccumulator o = new OdometryAccumulator();
        o.fieldUpdate(0, 0, 0);
        o.fieldUpdate(0, -14, 0);                    // PARK: 14 in to the right
        assertEquals(0, o.forwardInches(), EPS);
        assertEquals(14, o.strafeRightInches(), EPS);
    }

    @Test
    public void movesAreRobotRelativeAfterATurn() {
        OdometryAccumulator o = new OdometryAccumulator();
        o.fieldUpdate(0, 0, 90);                     // robot turned 90 deg CCW: its forward is field +Y
        o.fieldUpdate(0, 10, 90);
        assertEquals(10, o.forwardInches(), EPS);
        o.fieldUpdate(10, 10, 90);                   // field +X is now the robot's right
        assertEquals(10, o.strafeRightInches(), EPS);
    }

    @Test
    public void turningInPlaceAddsNothing() {
        OdometryAccumulator o = new OdometryAccumulator();
        for (int h = 0; h >= -34; h--) {
            o.fieldUpdate(0, 0, h);                  // AIM turn
        }
        assertEquals(0, o.forwardInches(), EPS);
        assertEquals(0, o.strafeRightInches(), EPS);
    }

    @Test
    public void wheelStepsContinueWhenThePinpointDrops() {
        OdometryAccumulator o = new OdometryAccumulator();
        o.fieldUpdate(0, 0, 0);
        o.fieldUpdate(5, 0, 0);
        o.robotStep(3, 1);                           // Pinpoint fault: wheel encoders for one loop
        o.fieldUpdate(50, 50, 0);                    // back again: restarts from here, no jump
        o.fieldUpdate(52, 50, 0);
        assertEquals(10, o.forwardInches(), EPS);
        assertEquals(1, o.strafeRightInches(), EPS);
    }
}

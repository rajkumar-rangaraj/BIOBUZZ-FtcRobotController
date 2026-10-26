package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class MecanumMathTest {
    private static final double EPS = 1e-9;

    @Test
    public void forwardDrivesAllWheelsForward() {
        assertArrayEquals(new double[]{0.5, 0.5, 0.5, 0.5}, MecanumMath.wheelPowers(0.5, 0, 0, 1), EPS);
    }

    @Test
    public void slideRightUsesTheMecanumXPattern() {
        // LF and RB forward, RF and LB backward
        assertArrayEquals(new double[]{0.5, -0.5, -0.5, 0.5}, MecanumMath.wheelPowers(0, 0.5, 0, 1), EPS);
    }

    @Test
    public void slideLeftIsTheMirrorOfSlideRight() {
        assertArrayEquals(new double[]{-0.5, 0.5, 0.5, -0.5}, MecanumMath.wheelPowers(0, -0.5, 0, 1), EPS);
    }

    @Test
    public void clockwiseTurnDrivesLeftSideForwardRightSideBack() {
        assertArrayEquals(new double[]{0.5, -0.5, 0.5, -0.5}, MecanumMath.wheelPowers(0, 0, 0.5, 1), EPS);
    }

    @Test
    public void fullDiagonalPlusTurnIsScaledKeepingTheRatio() {
        double[] p = MecanumMath.wheelPowers(1, 1, 1, 1);
        assertArrayEquals(new double[]{1, -1.0 / 3, 1.0 / 3, 1.0 / 3}, p, EPS);
        for (double v : p) {
            assertTrue(Math.abs(v) <= 1.0);
        }
    }

    @Test
    public void precisionScaleAppliesToEveryWheel() {
        assertArrayEquals(new double[]{0.35, -0.35, -0.35, 0.35}, MecanumMath.wheelPowers(0, 1, 0, 0.35), EPS);
    }

    @Test
    public void encoderMathRecoversTheCommand() {
        double[] w = MecanumMath.wheelPowers(0.2, -0.3, 0.1, 1);
        assertEquals(0.2, MecanumMath.forward(w), EPS);
        assertEquals(-0.3, MecanumMath.strafeRight(w), EPS);
        assertEquals(0.1, MecanumMath.turnCw(w), EPS);
    }

    @Test
    public void anglesWrapToPlusMinus180() {
        assertEquals(-170, MecanumMath.normalizeDegrees(190), EPS);
        assertEquals(170, MecanumMath.normalizeDegrees(-190), EPS);
        assertEquals(180, MecanumMath.normalizeDegrees(-180), EPS);
        assertEquals(0, MecanumMath.normalizeDegrees(720), EPS);
    }

    @Test
    public void cameraAimTurnsTowardTheTarget() {
        assertTrue("target on the left -> counter-clockwise", MotionController.aimTurnCw(10) < 0);
        assertTrue("target on the right -> clockwise", MotionController.aimTurnCw(-10) > 0);
        assertEquals(0, MotionController.aimTurnCw(BioBuzzConstants.AIM_TOLERANCE_DEG / 2), EPS);
    }
}

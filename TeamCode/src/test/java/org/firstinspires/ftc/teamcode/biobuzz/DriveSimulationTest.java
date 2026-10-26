package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Drives the simulated robot with every stick direction and with the closed-loop controller. */
public class DriveSimulationTest {

    @Test
    public void forwardAndBackMoveStraight() {
        SimMecanumDrive fwd = new SimMecanumDrive();
        fwd.hold(0.5, 0, 0, 0.5);
        assertTrue(fwd.x > 10);
        assertEquals(0, fwd.y, 0.1);
        assertEquals(0, fwd.headingDeg(), 0.1);

        SimMecanumDrive back = new SimMecanumDrive();
        back.hold(-0.5, 0, 0, 0.5);
        assertTrue(back.x < -10);
        assertEquals(0, back.y, 0.1);
    }

    @Test
    public void leftStickRightSlidesRightWithoutTurning() {
        SimMecanumDrive sim = new SimMecanumDrive();
        sim.hold(0, 0.5, 0, 0.5);
        assertTrue("slid right " + sim.rightOfStart(), sim.rightOfStart() > 10);
        assertEquals(0, sim.x, 0.1);
        assertEquals(0, sim.headingDeg(), 0.1);
    }

    @Test
    public void leftStickLeftSlidesLeftWithoutTurning() {
        SimMecanumDrive sim = new SimMecanumDrive();
        sim.hold(0, -0.5, 0, 0.5);
        assertTrue("slid left " + -sim.rightOfStart(), sim.rightOfStart() < -10);
        assertEquals(0, sim.x, 0.1);
        assertEquals(0, sim.headingDeg(), 0.1);
    }

    @Test
    public void rightStickTurnsInPlace() {
        SimMecanumDrive cw = new SimMecanumDrive();
        cw.hold(0, 0, 0.3, 0.5);
        assertTrue("clockwise lowers the heading", cw.headingDeg() < -20);
        assertEquals(0, Math.hypot(cw.x, cw.y), 0.1);

        SimMecanumDrive ccw = new SimMecanumDrive();
        ccw.hold(0, 0, -0.3, 0.5);
        assertTrue("counter-clockwise raises the heading", ccw.headingDeg() > 20);
    }

    /**
     * Why the System Check asks a person to watch the robot: if the front and back motors of one side are
     * swapped in the configuration (or both wheels on one side are the same hand), forward and turning
     * still look normal but the wheels fight during a slide. The encoders still report a slide.
     */
    @Test
    public void frontBackSwapOnOneSideStopsOnlyTheSlide() {
        int[] leftFrontAndBackSwapped = {MecanumMath.LB, MecanumMath.RF, MecanumMath.LF, MecanumMath.RB};

        SimMecanumDrive fwd = new SimMecanumDrive(leftFrontAndBackSwapped, 30);
        fwd.hold(0.5, 0, 0, 0.5);
        assertTrue("forward still works", fwd.x > 10);

        SimMecanumDrive turn = new SimMecanumDrive(leftFrontAndBackSwapped, 30);
        turn.hold(0, 0, 0.3, 0.5);
        assertTrue("turning still works", turn.headingDeg() < -20);

        SimMecanumDrive slide = new SimMecanumDrive(leftFrontAndBackSwapped, 30);
        slide.hold(0, 0.5, 0, 0.5);
        assertTrue("robot does not slide", Math.hypot(slide.x, slide.y) < 0.5);
        assertEquals("heading unchanged, so the IMU cannot see it", 0, slide.headingDeg(), 0.1);
        assertTrue("encoders still claim a slide", slide.strafeRightInches() > 10);
    }

    @Test
    public void controllerDrivesExactDistancesInEveryDirection() {
        double[][] moves = {{24, 0}, {-24, 0}, {0, 24}, {0, -24}, {12, 12}};
        for (double[] m : moves) {
            SimMecanumDrive sim = new SimMecanumDrive();
            boolean reached = new MotionController(sim).driveRelative(m[0], m[1], 4.0);
            assertTrue("reached " + m[0] + "," + m[1], reached);
            assertEquals(m[0], sim.x, 1.0);
            assertEquals(m[1], sim.rightOfStart(), 1.0);
            assertEquals(0, sim.headingDeg(), 2.0);
        }
    }

    @Test
    public void controllerTurnsToHeadingsBothWays() {
        for (double target : new double[]{90, -90, -34, 179}) {
            SimMecanumDrive sim = new SimMecanumDrive();
            assertTrue("turned to " + target, new MotionController(sim).turnToHeading(target, 4.0));
            assertEquals(0, MecanumMath.normalizeDegrees(target - sim.headingDeg()), 2.0);
        }
    }
}

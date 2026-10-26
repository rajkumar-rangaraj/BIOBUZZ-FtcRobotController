package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

/** Runs the real AutoRoute against the simulated robot and checks the AUTO scoring conditions. */
public class AutoRouteSimulationTest {

    /** Stands in for the launcher and windmill: firing just holds the robot still for some time. */
    private static class FakeMechanisms implements AutoRoute.Mechanisms {
        final SimMecanumDrive sim;
        final double fireSeconds;
        final List<String> steps = new ArrayList<>();
        boolean launcherOn = false;
        boolean fired = false;
        double headingAtFire = Double.NaN;
        double doneAt = Double.NaN;

        FakeMechanisms(SimMecanumDrive sim, double fireSeconds) {
            this.sim = sim;
            this.fireSeconds = fireSeconds;
        }

        @Override
        public void step(String name) {
            steps.add(name);
            if (name.equals("DONE")) {
                doneAt = sim.seconds();
            }
        }

        @Override
        public void startLauncher() {
            launcherOn = true;
        }

        @Override
        public void stopLauncher() {
            launcherOn = false;
        }

        @Override
        public void aimAndFire(double matchDeadlineSec) {
            assertTrue("launcher spun up before firing", launcherOn);
            fired = true;
            headingAtFire = sim.headingDeg();
            double end = Math.min(sim.seconds() + fireSeconds, matchDeadlineSec);
            while (sim.active() && sim.seconds() < end) {
                sim.drive(0, 0, 0);
                sim.loopTick();
            }
        }
    }

    private static void assertLeaveAndPark(SimMecanumDrive sim, FakeMechanisms mech) {
        assertTrue("route finished inside the 30 s AUTO, took " + mech.doneAt, mech.doneAt < 30.0);
        assertTrue("never pushed into the wall (max " + sim.maxX + ")", sim.maxX < 0.5);
        assertEquals("ends off the wall (LEAVE)", -AUTO_WALL_GAP_IN, sim.x, 1.0);
        assertEquals("slid right into the LOADING ZONE (PARK)", AUTO_PARK_STRAFE_IN, sim.rightOfStart(), 1.0);
        assertEquals("square to the wall again", 0, sim.headingDeg(), 3.0);
        assertFalse("launcher off at the end", mech.launcherOn);
    }

    @Test
    public void shootAndParkWithNoDelay() {
        SimMecanumDrive sim = new SimMecanumDrive();
        FakeMechanisms mech = new FakeMechanisms(sim, 6.0);
        AutoRoute.run(sim, mech, 0, true);

        assertTrue(mech.fired);
        assertEquals("aimed with the IMU before the camera", AUTO_AIM_HEADING_DEG, mech.headingAtFire, 2.0);
        assertLeaveAndPark(sim, mech);
    }

    @Test
    public void longestDelayAndSlowFiringStillParksInTime() {
        SimMecanumDrive sim = new SimMecanumDrive();
        FakeMechanisms mech = new FakeMechanisms(sim, 60.0);   // firing would never finish by itself
        AutoRoute.run(sim, mech, AUTO_MAX_START_DELAY_SEC, true);

        assertTrue(mech.fired);
        assertLeaveAndPark(sim, mech);
    }

    @Test
    public void parkOnlyNeverFires() {
        SimMecanumDrive sim = new SimMecanumDrive();
        FakeMechanisms mech = new FakeMechanisms(sim, 6.0);
        AutoRoute.run(sim, mech, 0, false);

        assertFalse(mech.fired);
        assertFalse(mech.steps.contains("FIRE"));
        assertLeaveAndPark(sim, mech);
    }
}

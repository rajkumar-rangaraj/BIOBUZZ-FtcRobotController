package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

/**
 * The AUTO route, shared by the robot OpModes and the laptop simulation tests.
 * The IMU heading must be zeroed at the start of the MATCH.
 */
public final class AutoRoute {
    private AutoRoute() {
    }

    /** The non-drive parts of AUTO. */
    public interface Mechanisms {
        void step(String name);

        void startLauncher();

        void stopLauncher();

        /** Camera fine-aim and feed the preloads; must return before matchDeadlineSec (DriveIO clock). */
        void aimAndFire(double matchDeadlineSec);
    }

    public static void run(DriveIO io, Mechanisms mech, double startDelaySec, boolean shoot) {
        MotionController motion = new MotionController(io);
        double matchStart = io.seconds();

        mech.step("DELAY");
        motion.waitSeconds(startDelaySec);
        if (shoot) {
            mech.startLauncher();
        }

        mech.step("LEAVE");
        motion.driveRelative(-AUTO_LEAVE_IN, 0, 2.5);

        if (shoot && io.seconds() - matchStart < AUTO_PARK_DEADLINE_SEC) {
            mech.step("AIM");
            motion.turnToHeading(AUTO_AIM_HEADING_DEG, 2.0);
            mech.step("FIRE");
            mech.aimAndFire(matchStart + AUTO_PARK_DEADLINE_SEC);
        }
        mech.stopLauncher();

        mech.step("RETURN");
        motion.turnToHeading(0, 2.0);
        motion.driveRelative(AUTO_LEAVE_IN - AUTO_WALL_GAP_IN, 0, 2.0);
        mech.step("PARK");
        motion.driveRelative(0, AUTO_PARK_STRAFE_IN, 3.0);
        mech.step("DONE");
    }
}

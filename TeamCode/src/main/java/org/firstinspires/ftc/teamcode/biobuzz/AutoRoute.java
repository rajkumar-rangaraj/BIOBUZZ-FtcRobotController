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

        /**
         * Shooting position taught in TeleOp: {forward in, right in, heading deg} from the AUTO start, or null to use
         * the fixed LEAVE + AUTO_AIM_HEADING_DEG route.
         */
        double[] shootPose();
    }

    public static void run(DriveIO io, Mechanisms mech, double startDelaySec, boolean shoot) {
        MotionController motion = new MotionController(io);
        double matchStart = io.seconds();
        double f0 = io.forwardInches(), s0 = io.strafeRightInches();
        double[] pose = shoot ? mech.shootPose() : null;

        mech.step("DELAY");
        motion.waitSeconds(startDelaySec);
        if (shoot) {
            mech.startLauncher();
        }

        mech.step("LEAVE");
        motion.driveRelative(-AUTO_LEAVE_IN, 0, 2.5);

        if (shoot && io.seconds() - matchStart < AUTO_PARK_DEADLINE_SEC) {
            if (pose != null) {
                // translate only while square to the start (robot frame = start frame), then turn on the spot
                mech.step("GOTO");
                motion.driveRelative(pose[0] - (io.forwardInches() - f0), pose[1] - (io.strafeRightInches() - s0), 6.0);
            }
            mech.step("AIM");
            motion.turnToHeading(pose != null ? pose[2] : AUTO_AIM_HEADING_DEG, 2.0);
            mech.step("FIRE");
            mech.aimAndFire(matchStart + AUTO_PARK_DEADLINE_SEC);
        }
        mech.stopLauncher();

        mech.step("RETURN");
        motion.turnToHeading(0, 2.0);
        if (pose != null) {
            motion.driveRelative(-AUTO_WALL_GAP_IN - (io.forwardInches() - f0), -(io.strafeRightInches() - s0), 6.0);
        } else {
            motion.driveRelative(AUTO_LEAVE_IN - AUTO_WALL_GAP_IN, 0, 2.0);
        }
        mech.step("PARK");
        motion.driveRelative(0, AUTO_PARK_STRAFE_IN, 3.0);
        mech.step("DONE");
    }
}

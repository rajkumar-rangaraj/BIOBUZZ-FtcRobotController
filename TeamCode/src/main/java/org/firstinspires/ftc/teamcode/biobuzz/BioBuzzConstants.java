package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Every tuned number for the BIOBUZZ robot lives here.
 *
 * Robot frame used by all code: FRONT = intake / flower-arm side, BACK = launcher / camera side.
 * Values marked CALIBRATE are first estimates; measure them with "BioBuzz Calibration".
 */
public final class BioBuzzConstants {
    private BioBuzzConstants() {
    }

    // ---------------- Robot configuration names ----------------
    // Wiring: LEFT side -> Control Hub, RIGHT side -> Expansion Hub (port table in README_2.md).
    // Control Hub: motors 0 left_front_drive, 1 left_back_drive, 2 launcher;
    //              servo 0 left_intake_servo; built-in imu; USB Webcam 1
    // Expansion Hub: motors 0 right_front_drive, 1 right_back_drive, 2 intake, 3 indexer;
    //                servos 0 right_intake_servo, 1 flower_arm
    public static final String LEFT_FRONT_DRIVE = "left_front_drive";
    public static final String RIGHT_FRONT_DRIVE = "right_front_drive";
    public static final String LEFT_BACK_DRIVE = "left_back_drive";
    public static final String RIGHT_BACK_DRIVE = "right_back_drive";
    public static final String INTAKE = "intake";
    public static final String LAUNCHER = "launcher";
    public static final String INDEXER = "indexer";
    public static final String LEFT_INTAKE_SERVO = "left_intake_servo";
    public static final String RIGHT_INTAKE_SERVO = "right_intake_servo";
    public static final String FLOWER_ARM = "flower_arm";
    public static final String IMU_NAME = "imu";
    public static final String WEBCAM = "Webcam 1";

    // ---------------- goBILDA Pinpoint odometry (optional, AUTO only) ----------------
    // Control Hub I2C bus 1, configured as "goBILDA Pinpoint Odometry Computer" named "pinpoint".
    // Not in the configuration -> AUTO drives on the wheel encoders exactly as before.
    public static final String PINPOINT = "pinpoint";
    public static final boolean PINPOINT_SWINGARM_PODS = false;                   // false = 4-Bar pods
    // Pod positions from the robot's turning centre, in mm (CAD layout: make_odometry_mount.py; measure on the robot,
    // then check with "BioBuzz Calibration": spin in place, X and Y must stay near 0):
    public static final double PINPOINT_X_POD_LEFT_MM = 104;                     // CALIBRATE: forward pod, + = left
    public static final double PINPOINT_Y_POD_FORWARD_MM = -164;                 // CALIBRATE: strafe pod, + = forward
    public static final boolean PINPOINT_X_REVERSED = false;                     // CALIBRATE: push forward -> X +
    public static final boolean PINPOINT_Y_REVERSED = false;                     // CALIBRATE: slide left -> Y +

    // ---------------- Control Hub IMU mounting ----------------
    // Hub stands upright at the back: its logo faces the launcher side, its USB ports face outward (robot left).
    public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    public static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

    // ---------------- Drivetrain: 4 x goBILDA 5203-2402-0019 (312 RPM) ----------------
    public static final double DRIVE_TICKS_PER_REV = 537.7;
    public static final double WHEEL_DIAMETER_IN = 104.0 / 25.4;                 // CALIBRATE (push 48 in)
    public static final double DRIVE_TICKS_PER_INCH = DRIVE_TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);
    public static final double STRAFE_MULTIPLIER = 1.10;                         // CALIBRATE (strafe 48 in)
    public static final double SLOW_DRIVE_SCALE = 0.35;
    public static final double ARM_DEPLOYED_DRIVE_SCALE = 0.30;

    // ---------------- Launcher: Yellow Jacket converted to 1:1 (6000 RPM, 28 ticks/rev) ----------------
    public static final double LAUNCHER_TICKS_PER_REV = 28.0;
    public static final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(40, 0, 0, 12.5);
    public static final double LAUNCHER_NEAR_VELOCITY = 1250;                    // CALIBRATE by test shots
    public static final double LAUNCHER_FAR_VELOCITY = 1450;                     // CALIBRATE by test shots
    public static final double LAUNCHER_TRIM_STEP = 25;
    public static final double LAUNCHER_MAX_VELOCITY = 2600;
    public static final double LAUNCHER_READY_MARGIN = 100;
    public static final double LAUNCHER_ENCODER_WARNING_SEC = 1.0;

    // ---------------- Indexer: polycarbonate windmill on a 5203-2402-0014 (435 RPM) ----------------
    public static final double INDEXER_TICKS_PER_REV = 384.5;
    public static final int INDEXER_POCKETS = 4;
    public static final double INDEXER_TICKS_PER_POCKET = INDEXER_TICKS_PER_REV / INDEXER_POCKETS;
    // If FIRE turns the windmill away from the launcher, change this to REVERSE.
    public static final DcMotorSimple.Direction INDEXER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final double INDEXER_FEED_POWER = 0.6;                         // CALIBRATE with shot recovery
    // Robot log 2026-10-30: at 0.6 power the windmill turns ~1500 ticks/s and a pocket finished 1.06-1.76 pockets
    // late (one 13-54 ms loop = up to 80 ticks at that speed). Power now falls in proportion to the ticks left over the\r
    // whole pocket, so the boundary is reached slowly (simulated: 87..110 ticks per shot, error never adds up):
    public static final double INDEXER_RAMP_TICKS = INDEXER_TICKS_PER_REV / INDEXER_POCKETS;   // one pocket
    public static final double INDEXER_MIN_POWER = 0.15;                         // CALIBRATE: must still turn with a ball
    public static final double INDEXER_UNJAM_POWER = 0.35;
    // One ball per shot (ShotFeeder): one pocket, then wait for the flywheel before the next
    public static final double SHOT_MIN_INTERVAL_SEC = 0.35;                     // CALIBRATE with BioBuzz Shot Log
    public static final double SHOT_POCKET_TIMEOUT_SEC = 1.0;                    // a pocket taking longer = jam
    public static final double SHOT_UNJAM_SEC = 0.25;                            // back-off time after a jam
    public static final int SHOT_MAX_JAMS = 3;

    // ---------------- Intake ----------------
    public static final double FEED_INTAKE_BOOST = 0.5;

    // ---------------- Flower arm: goBILDA 2000-0025-0002 in SERVO mode ----------------
    // Negative means "not calibrated": TeleOp then refuses to move the arm.
    public static final double ARM_DEPLOY_POSITION = -1;                         // CALIBRATE
    public static final double ARM_STOW_OFFSET = 179.0 / 300.0;                  // 179 deg of a 300 deg servo
    public static final int ARM_STOW_DIRECTION = +1;                             // CALIBRATE (+1 or -1)
    public static final double ARM_PWM_MIN_US = 500;
    public static final double ARM_PWM_MAX_US = 2500;
    public static final double ARM_DEPLOY_MAX_STICK = 0.10;

    // ---------------- HIVE AprilTags (36h11, 3.25 in) ----------------
    public static final double TAG_SIZE_IN = 3.25;
    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 480;
    public static final double TARGET_MAX_AGE_SEC = 0.30;
    public static final double CAMERA_TILT_DEG = 30.0;                           // CALIBRATE: bracket setting
    public static final double CAMERA_LENS_HEIGHT_IN = 12.2;                     // CAD v1 mount; RE-MEASURE on the current mount
    public static final double AIM_KP = 0.015;                                  // turn power per degree
    public static final double AIM_MAX_TURN = 0.30;
    public static final double AIM_MIN_TURN = 0.06;
    public static final double AIM_TOLERANCE_DEG = 2.0;

    // ---------------- Autonomous motion ----------------
    public static final double AUTO_DRIVE_POWER = 0.45;
    public static final double AUTO_DRIVE_MIN_POWER = 0.12;
    public static final double AUTO_DRIVE_KP = 0.05;                             // power per inch of error
    public static final double AUTO_DRIVE_TOLERANCE_IN = 0.5;
    public static final double HEADING_KP = 0.02;
    public static final double TURN_MAX_POWER = 0.40;
    public static final double TURN_MIN_POWER = 0.08;
    public static final double TURN_TOLERANCE_DEG = 1.5;

    // ---------------- Autonomous route (identical for RED and BLUE: the field is point-symmetric) ----------------
    // Start: intake side touching our ALLIANCE wall, robot fully outside the LOADING ZONE on its audience-side
    // (RED) / rear-side (BLUE) edge, with about 1 in between the robot and the LOADING ZONE tape.
    public static final double AUTO_LEAVE_IN = 12;                               // back away from the wall
    public static final double AUTO_AIM_HEADING_DEG = -34;                       // CALIBRATE; + is counter-clockwise
    public static final double AUTO_WALL_GAP_IN = 4;                             // stay off the wall for LEAVE
    public static final double AUTO_PARK_STRAFE_IN = 14;                         // CALIBRATE; strafe right into zone
    public static final double AUTO_LAUNCH_VELOCITY = LAUNCHER_NEAR_VELOCITY;    // CALIBRATE at the shot spot
    public static final int AUTO_FEED_POCKETS = 5;                               // 4 preloads + 1 clearing pocket
    public static final double AUTO_FEED_TIMEOUT_SEC = 8.0;
    public static final double AUTO_AIM_TIMEOUT_SEC = 2.0;
    public static final double AUTO_PARK_DEADLINE_SEC = 22.0;
    public static final int AUTO_MAX_START_DELAY_SEC = 10;

    // ---------------- Diagnostics ----------------
    public static final boolean DIAG_DEFAULT_ON = true;                         // Back / Share in INIT toggles
    public static final double DIAG_SAMPLE_MS = 50;                              // 20 rows per second
    public static final double DIAG_SLOW_SAMPLE_MS = 500;                        // battery + IMU (each read costs ~2 ms)
}

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
    // Any Control Hub I2C port (ours: port 0, shared with the built-in IMU - different address), configured as
    // "goBILDA Pinpoint Odometry Computer" named "pinpoint". Not in the configuration -> AUTO uses the wheel encoders.
    // Pod directions and positions are MEASURED by "BioBuzz Pinpoint Setup" and saved on the hub
    // (/sdcard/biobuzz_pinpoint.txt); the values below are only used until that has been run.
    public static final String PINPOINT = "pinpoint";
    public static final boolean PINPOINT_SWINGARM_PODS = false;                   // false = 4-Bar pods
    // Measured 2026-10-30 (track 410, wheelbase 270): forward pod 245 mm from the left wheels, strafe pod 310 mm behind
    // the front axle. Robot test: forward 12 in -> forward pod +11.94, left 12 in -> strafe pod +11.83 (both normal).
    public static final double PINPOINT_X_POD_LEFT_MM = -40;                     // forward pod, + = left of centre
    public static final double PINPOINT_Y_POD_FORWARD_MM = -175;                 // strafe pod, + = forward of centre
    public static final boolean PINPOINT_X_REVERSED = false;                     // push forward -> X +
    public static final boolean PINPOINT_Y_REVERSED = false;                     // slide left -> Y +
    public static final double PINPOINT_MAX_HEADING_ERROR_DEG = 15;             // AUTO drops the Pinpoint if its heading drifts from the IMU

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
    // P 40 -> 100 (5 Nov): P 40 added only ~2 W after a shot, so the flywheel took ~0.6 s to recover and drooped
    // ~20 t/s on a tired battery. Flywheel + motor time constant ~1.3 s: P 100 gives a ~0.13 s closed loop, far from
    // unstable. If the speed hunts (READY flickers with no balls), step back toward 40.
    public static final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(100, 0, 0, 12.5);
    // Printed hood PPL1 (74 deg exit, 9 in high), upward CELL lips 53.5 / 65.1 in (field CAD), tools/shot_model.py:
    // 5.52 m/s scores from about 28 to 42 in (exit to the CELL opening, horizontal; +/-4.7 % at 30-36 in), 5.94 m/s
    // from about 46 to 54 in (+/-3.6-3.8 %). Ball speed ~0.89 x half the tread speed -> 1150 / 1240 t/s. The 0.89 is
    // the uncertain part (0.85-0.93; the old hood's 1250 t/s spot implies 0.86-0.89): CALIBRATE with one-ball test
    // shots (BioBuzz Shot Log) and the D-pad trim. Old polycarbonate hood (66 deg): 1250 / 1450.
    public static final double LAUNCHER_NEAR_VELOCITY = 1150;                    // CALIBRATE by test shots
    public static final double LAUNCHER_FAR_VELOCITY = 1240;                     // CALIBRATE by test shots
    public static final double LAUNCHER_TRIM_STEP = 15;                          // D-pad RIGHT/LEFT; encoder reads in ~20 t/s steps
    public static final double LAUNCHER_MAX_VELOCITY = 2600;
    // Ready to fire (LauncherGate): within +/-30 t/s (2.6 %; the scoring window is about +/-4 %) for 0.1 s. If it can't
    // settle there for 1 s, the fallback fires anyway when the speed is between target - 60 and target + 30, so a weak
    // battery or a hunting speed can't block shooting. (P 100 keeps the steady error under ~16 t/s, so normally the
    // fallback never runs: the log shows READY_FALLBACK if it does.)
    public static final double LAUNCHER_READY_TOLERANCE = 30;
    public static final double LAUNCHER_READY_HOLD_SEC = 0.10;
    public static final double LAUNCHER_FALLBACK_MARGIN = 60;
    public static final double LAUNCHER_READY_FALLBACK_SEC = 1.0;
    public static final double LAUNCHER_ENCODER_WARNING_SEC = 1.0;

    // ---------------- Indexer: polycarbonate windmill on a 5203-2402-0014 (435 RPM) ----------------
    public static final double INDEXER_TICKS_PER_REV = 384.5;
    public static final int INDEXER_POCKETS = 4;
    public static final double INDEXER_TICKS_PER_POCKET = INDEXER_TICKS_PER_REV / INDEXER_POCKETS;
    // If FIRE turns the windmill away from the launcher, change this to REVERSE.
    public static final DcMotorSimple.Direction INDEXER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final double INDEXER_FEED_POWER = 0.40;                        // 0.6 -> 0.45 -> 0.40: lone balls hit the pocket corners
    // Robot log 2026-10-30: at 0.6 power the windmill turns ~1500 ticks/s and a pocket finished 1.06-1.76 pockets
    // late (one 13-54 ms loop = up to 80 ticks at that speed). Power now falls in proportion to the ticks left over the\r
    // whole pocket, so the boundary is reached slowly (simulated: 87..110 ticks per shot, error never adds up):
    public static final double INDEXER_RAMP_TICKS = INDEXER_TICKS_PER_REV / INDEXER_POCKETS;   // one pocket
    public static final double INDEXER_MIN_POWER = 0.15;                         // CALIBRATE: must still turn with a ball (stuck 0.15 s: ShotFeeder adds 0.1 steps)
    public static final double INDEXER_UNJAM_POWER = 0.35;
    // One ball per shot (ShotFeeder): one pocket, then wait for the flywheel before the next
    public static final double SHOT_MIN_INTERVAL_SEC = 0.35;                     // CALIBRATE with BioBuzz Shot Log
    public static final double SHOT_POCKET_TIMEOUT_SEC = 1.0;                    // a pocket taking longer = jam
    public static final double SHOT_UNJAM_SEC = 0.25;                            // back-off time after a jam (stops after a quarter pocket)
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
    public static final double AIM_MIN_TURN = 0.10;                                   // 0.06 stalled 4 deg short (31 Oct AUTO)
    public static final double AIM_TOLERANCE_DEG = 1.5;                         // 2.0 = ~2 in sideways at 55 in
    // Aim gate (TeleOp FIRE, see AimGate): no ball is fed unless the aim (trim included) is within this and the robot
    // isn't turning. 1 Nov: 285 of 447 feeds were outside the opening sideways. The bearing is in the tilted camera's
    // frame: 2.0 deg there is ~2.6-3 deg of robot yaw, ~2 in sideways at 36 in. Must stay above AIM_TOLERANCE_DEG (X).
    // If the CELL drops out of view, the hold lasts AIM_LOST_HOLD_SEC; after that (no CELL) it is the driver's call.
    // The gate gives way after AIM_GATE_GIVEWAY_SEC of holding a ready shot (logged AIM_GIVEWAY), so a knocked camera
    // or a stale trim can only slow the shooting, not stop it. X in TeleOp INIT turns the gate off for the match.
    public static final boolean AIM_GATE_ENABLED = true;
    public static final double AIM_FIRE_TOLERANCE_DEG = 2.0;
    public static final double AIM_LOST_HOLD_SEC = 0.75;
    public static final double AIM_GATE_GIVEWAY_SEC = 1.5;
    public static final double AIM_GATE_MAX_TURN = 0.05;                        // turn power still counted as "not turning"
    public static final double HOLD_TOLERANCE_DEG = 0.75;                       // AUTO holds the aimed heading while firing
    public static final double HOLD_MAX_TURN = 0.20;
    // SHOT SPOT (TeleOp: Share saves the camera's FLOOR distance to the CELL + speed, holding X drives back to it)
    public static final double SPOT_KP = 0.03;                                  // drive power per inch of error
    public static final double SPOT_MIN_POWER = 0.12;
    public static final double SPOT_MAX_POWER = 0.35;
    public static final double SPOT_TOLERANCE_IN = 1.5;

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
    public static final double AUTO_AIM_HEADING_DEG = -50;                       // camera found the BLUE CELL at about -50 (31 Oct); + is CCW
    public static final double AUTO_WALL_GAP_IN = 4;                             // stay off the wall for LEAVE
    public static final double AUTO_PARK_STRAFE_IN = 14;                         // CALIBRATE; strafe right into zone
    public static final double AUTO_LAUNCH_VELOCITY = LAUNCHER_NEAR_VELOCITY;    // CALIBRATE at the shot spot
    public static final int AUTO_FEED_POCKETS = 12;                              // up to 3 full turns (not every pocket launches)
    public static final double AUTO_FIRE_SETTLE_SEC = 0.5;                       // still robot before firing: balls settle
    public static final double AUTO_FEED_POWER = 0.3;                            // ContinuousFeeder power (not used by AUTO now)
    public static final int AUTO_PRELOADS = 4;                                   // stop early once this many balls left
    // Flywheel slowdown that means a ball left. The old hood dipped 40-130 t/s; the printed hood at 1150 t/s is expected
    // to dip ~50-70 (simulated), so 60 was borderline. CALIBRATE: ~0.6 x the dips BioBuzz Shot Log measures.
    public static final double AUTO_SHOT_DIP_TPS = 35;
    // ... and only while a pocket turns or up to this long after it (BallCounter), so reading noise, spin-up and speed
    // changes are never counted as balls. The dip is measured from the pre-feed speed and must last AUTO_DIP_MIN_SEC:
    // one odd hub reading (~50 ms) never counts, a real ball keeps the flywheel down for 100 ms or more.
    public static final double AUTO_DIP_WINDOW_SEC = 0.6;
    public static final double AUTO_DIP_MIN_SEC = 0.06;
    public static final double AUTO_FEED_TIMEOUT_SEC = 8.0;
    public static final double AUTO_AIM_TIMEOUT_SEC = 3.5;                              // camera aim + drive to the SHOT SPOT distance
    public static final double AUTO_SPOT_MAX_IN = 24;                              // AUTO never drives further than this to the spot
    public static final double AUTO_PARK_DEADLINE_SEC = 20.0;                           // leaves time for SPOT BACK + RETURN + PARK
    public static final int AUTO_MAX_START_DELAY_SEC = 10;

    // ---------------- Diagnostics ----------------
    public static final boolean DIAG_DEFAULT_ON = true;                         // Back / Share in INIT toggles
    public static final double DIAG_SAMPLE_MS = 50;                              // 20 rows per second
    public static final double DIAG_SLOW_SAMPLE_MS = 500;                        // battery + IMU (each read costs ~2 ms)
}

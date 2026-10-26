package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * BIOBUZZ competition TeleOp - ONE gamepad (Xbox name / PlayStation symbol).
 *
 *   Left stick            drive + strafe          (intake side is FRONT)
 *   Right stick X         turn
 *   Left bumper  (hold)   precision drive
 *   Right trigger         intake IN (analog)
 *   Left trigger          intake OUT (analog) - half pull = the flower "back-out" reverse
 *   Right bumper (hold)   FIRE: spins the launcher up and feeds the windmill when at speed
 *   X / Square   (hold)   HIVE auto-aim: camera turns the robot, you still drive with the left stick
 *   Y / Triangle          launcher pre-spin ON/OFF
 *   A / Cross             flower arm DEPLOY / STOW (deploys only while the sticks are released)
 *   B / Circle   (hold)   unjam: windmill slowly backwards
 *   D-pad up / down       launcher FAR / NEAR preset
 *   D-pad right / left    launcher trim +/- 25 ticks/s
 *
 * During INIT: B / Circle = RED, A / Cross = BLUE (defaults to the alliance the last AUTO ran).
 * While the arm is DEPLOYED, driving is limited to precision speed.
 */
@TeleOp(name = "BioBuzz TeleOp", group = "BioBuzz")
public class BioBuzzTeleOp extends OpMode {

    private BioBuzzRobot robot;
    private HiveVision vision;
    private String visionError = null;
    private HiveVision.Alliance alliance = BioBuzzMatchState.alliance;

    private double launcherPreset = LAUNCHER_NEAR_VELOCITY;
    private double launcherTrim = 0;
    private boolean launcherToggledOn = false;
    private boolean lastReady = false;
    private boolean endgameWarned = false;
    private String status = "";
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime statusTimer = new ElapsedTime();

    @Override
    public void init() {
        robot = new BioBuzzRobot(hardwareMap);
        try {
            vision = new HiveVision(hardwareMap);
        } catch (Exception e) {
            vision = null;
            visionError = e.getMessage();
        }
    }

    @Override
    public void init_loop() {
        if (gamepad1.bWasPressed()) {
            alliance = HiveVision.Alliance.RED;
        } else if (gamepad1.aWasPressed()) {
            alliance = HiveVision.Alliance.BLUE;
        }
        telemetry.addData("Alliance", "%s   (B/Circle = RED, A/Cross = BLUE)", alliance);
        telemetry.addData("Flower arm", BioBuzzRobot.armCalibrated() ? "calibrated" : "NOT CALIBRATED - arm disabled");
        telemetry.addData("Camera", vision == null ? "NOT FOUND: " + visionError : "ok");
        telemetry.addData("Battery", "%.2f V", robot.battery.getVoltage());
    }

    @Override
    public void start() {
        BioBuzzMatchState.alliance = alliance;
        matchTimer.reset();
        robot.stowArm();
    }

    @Override
    public void loop() {
        if (vision != null) {
            vision.update();
        }

        // ---------------- Launcher ----------------
        if (gamepad1.yWasPressed()) {
            launcherToggledOn = !launcherToggledOn;
        }
        if (gamepad1.dpadUpWasPressed()) {
            launcherPreset = LAUNCHER_FAR_VELOCITY;
            launcherTrim = 0;
        } else if (gamepad1.dpadDownWasPressed()) {
            launcherPreset = LAUNCHER_NEAR_VELOCITY;
            launcherTrim = 0;
        } else if (gamepad1.dpadRightWasPressed()) {
            launcherTrim += LAUNCHER_TRIM_STEP;
        } else if (gamepad1.dpadLeftWasPressed()) {
            launcherTrim -= LAUNCHER_TRIM_STEP;
        }
        boolean fire = gamepad1.right_bumper;
        double launcherTarget = Range.clip(launcherPreset + launcherTrim, 0, LAUNCHER_MAX_VELOCITY);
        robot.setLauncherVelocity(launcherToggledOn || fire ? launcherTarget : 0);
        boolean ready = robot.launcherReady();
        if (ready && !lastReady) {
            gamepad1.rumble(250);
        }
        lastReady = ready;

        // ---------------- Flower arm ----------------
        boolean sticksReleased = Math.abs(gamepad1.left_stick_x) < ARM_DEPLOY_MAX_STICK
                && Math.abs(gamepad1.left_stick_y) < ARM_DEPLOY_MAX_STICK
                && Math.abs(gamepad1.right_stick_x) < ARM_DEPLOY_MAX_STICK;
        if (gamepad1.aWasPressed()) {
            if (!BioBuzzRobot.armCalibrated()) {
                setStatus("Flower arm not calibrated - run BioBuzz Calibration");
            } else if (robot.armState() == BioBuzzRobot.ArmState.DEPLOYED) {
                robot.stowArm();
            } else if (sticksReleased) {
                robot.deployArm();
            } else {
                setStatus("Stop the robot before deploying the arm");
                gamepad1.rumbleBlips(2);
            }
        }
        boolean armDeployed = robot.armState() == BioBuzzRobot.ArmState.DEPLOYED;

        // ---------------- Drive ----------------
        double scale = gamepad1.left_bumper ? SLOW_DRIVE_SCALE : 1.0;
        if (armDeployed) {
            scale = Math.min(scale, ARM_DEPLOYED_DRIVE_SCALE);
        }
        double forward = -gamepad1.left_stick_y * scale;
        double strafe = gamepad1.left_stick_x * scale;
        double turn = gamepad1.right_stick_x * scale;
        HiveVision.Target target = null;
        if (gamepad1.x && vision != null) {
            target = vision.target(alliance, null);
            if (target != null) {
                turn = HiveVision.aimTurnCw(target.bearingDeg);
            }
        }
        robot.drive(forward, strafe, turn, 1.0);

        // ---------------- Intake + windmill ----------------
        double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
        boolean feeding = fire && ready;
        if (feeding) {
            robot.setIndexerPower(INDEXER_FEED_POWER);
            intakePower = Math.max(intakePower, 0) + FEED_INTAKE_BOOST;
        } else if (gamepad1.b) {
            robot.setIndexerPower(-INDEXER_UNJAM_POWER);
        } else {
            robot.setIndexerPower(0);
        }
        robot.setIntake(intakePower);

        // ---------------- Match reminders ----------------
        if (!endgameWarned && matchTimer.seconds() >= 100) {
            endgameWarned = true;
            gamepad1.rumbleBlips(3);
            setStatus("20 s left - PARK in the LOADING ZONE");
        }

        // ---------------- Telemetry ----------------
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Launcher", "%s  target %.0f  actual %.0f t/s (%s%+.0f)",
                !robot.isLauncherOn() ? "OFF" : (ready ? "READY" : "spinning"),
                robot.launcherTarget(), robot.launcherVelocity(),
                launcherPreset == LAUNCHER_FAR_VELOCITY ? "FAR" : "NEAR", launcherTrim);
        telemetry.addData("Windmill", feeding ? "FEEDING" : (gamepad1.b ? "UNJAM" : "parked"));
        telemetry.addData("Flower arm", BioBuzzRobot.armCalibrated() ? robot.armState().toString() : "NOT CALIBRATED");
        telemetry.addData("Drive", "%s", armDeployed ? "ARM OUT - precision" : (gamepad1.left_bumper ? "precision" : "normal"));
        if (vision == null) {
            telemetry.addData("HIVE aim", "camera not found");
        } else if (target != null) {
            telemetry.addData("HIVE aim", "%s  %d tags  bearing %.1f  range %.0f in",
                    target.cell, target.tagCount, target.bearingDeg, target.rangeIn);
        } else {
            telemetry.addData("HIVE aim", gamepad1.x ? "no %s CELL tags in view" : "hold X to aim", alliance);
        }
        String warning = robot.launcherWarning();
        if (warning != null) {
            telemetry.addData("WARNING", warning);
        }
        if (statusTimer.seconds() < 3 && !status.isEmpty()) {
            telemetry.addData("Note", status);
        }
        telemetry.addData("Battery", "%.2f V", robot.battery.getVoltage());
    }

    @Override
    public void stop() {
        robot.stopAll();
        if (vision != null) {
            vision.close();
        }
    }

    private void setStatus(String s) {
        status = s;
        statusTimer.reset();
    }
}

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
 * During INIT: B / Circle = RED, A / Cross = BLUE (defaults to the alliance the last AUTO ran),
 *               Back / Share = diagnostics log on/off (files in /sdcard/BioBuzzLogs, see README_2).
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
    private BioBuzzDiag diag = new BioBuzzDiag("teleop", false);
    private final ShotFeeder feeder = ShotFeeder.standard();
    private boolean lastFire = false;
    private boolean lastAim = false;
    private boolean lastAimHadTarget = false;
    private boolean lastUnjam = false;

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
        if (gamepad1.backWasPressed()) {
            BioBuzzMatchState.diagnostics = !BioBuzzMatchState.diagnostics;
        }
        telemetry.addData("Alliance", "%s   (B/Circle = RED, A/Cross = BLUE)", alliance);
        telemetry.addData("Diagnostics log", "%s   (Back/Share toggles)", BioBuzzMatchState.diagnostics ? "ON" : "off");
        telemetry.addData("Flower arm", BioBuzzRobot.armCalibrated() ? "calibrated" : "NOT CALIBRATED - arm disabled");
        telemetry.addData("Camera", vision == null ? "NOT FOUND: " + visionError : "ok");
        telemetry.addData("Battery", "%.2f V", robot.battery.getVoltage());
    }

    @Override
    public void start() {
        BioBuzzMatchState.alliance = alliance;
        matchTimer.reset();
        diag = new BioBuzzDiag("teleop", BioBuzzMatchState.diagnostics);
        diag.event("CONFIG", String.format("alliance=%s arm_calibrated=%s camera=%s launcher_near=%.0f far=%.0f battery=%.2f",
                alliance, BioBuzzRobot.armCalibrated(), vision == null ? "MISSING " + visionError : "ok",
                LAUNCHER_NEAR_VELOCITY, LAUNCHER_FAR_VELOCITY, robot.battery.getVoltage()));
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
            diag.event("LAUNCHER", launcherToggledOn ? "pre-spin ON" : "pre-spin OFF");
        }
        if (gamepad1.dpadUpWasPressed()) {
            launcherPreset = LAUNCHER_FAR_VELOCITY;
            launcherTrim = 0;
            diag.event("PRESET", "FAR " + LAUNCHER_FAR_VELOCITY);
        } else if (gamepad1.dpadDownWasPressed()) {
            launcherPreset = LAUNCHER_NEAR_VELOCITY;
            launcherTrim = 0;
            diag.event("PRESET", "NEAR " + LAUNCHER_NEAR_VELOCITY);
        } else if (gamepad1.dpadRightWasPressed()) {
            launcherTrim += LAUNCHER_TRIM_STEP;
            diag.event("TRIM", String.format("%+.0f", launcherTrim));
        } else if (gamepad1.dpadLeftWasPressed()) {
            launcherTrim -= LAUNCHER_TRIM_STEP;
            diag.event("TRIM", String.format("%+.0f", launcherTrim));
        }
        boolean fire = gamepad1.right_bumper;
        double launcherTarget = Range.clip(launcherPreset + launcherTrim, 0, LAUNCHER_MAX_VELOCITY);
        robot.setLauncherVelocity(launcherToggledOn || fire ? launcherTarget : 0);
        boolean ready = robot.launcherReady();
        if (ready && !lastReady) {
            gamepad1.rumble(250);
            diag.event("LAUNCHER_READY", String.format("%.0f t/s", robot.launcherVelocity()));
        }
        lastReady = ready;
        if (fire != lastFire) {
            diag.event(fire ? "FIRE_PRESS" : "FIRE_RELEASE", String.format("ready=%s vel=%.0f", ready, robot.launcherVelocity()));
            lastFire = fire;
        }

        // ---------------- Flower arm ----------------
        boolean sticksReleased = Math.abs(gamepad1.left_stick_x) < ARM_DEPLOY_MAX_STICK
                && Math.abs(gamepad1.left_stick_y) < ARM_DEPLOY_MAX_STICK
                && Math.abs(gamepad1.right_stick_x) < ARM_DEPLOY_MAX_STICK;
        if (gamepad1.aWasPressed()) {
            if (!BioBuzzRobot.armCalibrated()) {
                setStatus("Flower arm not calibrated - run BioBuzz Calibration");
                diag.event("ARM_REFUSED", "not calibrated");
            } else if (robot.armState() == BioBuzzRobot.ArmState.DEPLOYED) {
                robot.stowArm();
                diag.event("ARM", "STOW");
            } else if (sticksReleased) {
                robot.deployArm();
                diag.event("ARM", "DEPLOY");
            } else {
                setStatus("Stop the robot before deploying the arm");
                gamepad1.rumbleBlips(2);
                diag.event("ARM_REFUSED", "robot moving");
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
        HiveVision.Target target = vision == null ? null : vision.target(alliance, null);   // every loop: logged + shown
        if (gamepad1.x && target != null) {
            turn = HiveVision.aimTurnCw(target.bearingDeg);
        }
        boolean aim = gamepad1.x;
        if (aim != lastAim) {
            diag.event(aim ? "AIM_PRESS" : "AIM_RELEASE", vision == null ? "camera missing" : "");
            lastAim = aim;
        }
        if (aim && (target != null) != lastAimHadTarget) {
            diag.event(target != null ? "AIM_TARGET_FOUND" : "AIM_TARGET_LOST",
                    target != null ? String.format("%s %d tags bearing %.1f", target.cell, target.tagCount, target.bearingDeg) : "");
        }
        lastAimHadTarget = aim && target != null;
        robot.drive(forward, strafe, turn, 1.0);

        // ---------------- Intake + windmill (one ball per shot, see ShotFeeder) ----------------
        double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
        if (gamepad1.b) {
            feeder.reset();
            robot.setIndexerPower(-INDEXER_UNJAM_POWER);
        } else {
            robot.setIndexerPower(feeder.update(matchTimer.seconds(), robot.indexerTicks(), ready, fire));
            String feedEvent = feeder.event();
            if (feedEvent != null) {
                diag.event("FEED", feedEvent + String.format(" | launcher %.0f t/s", robot.launcherVelocity()));
                if (feedEvent.startsWith("JAM")) {
                    setStatus("Windmill " + feedEvent + " - hold B to unjam");
                    gamepad1.rumbleBlips(1);
                }
            }
        }
        boolean feeding = feeder.feeding();
        if (fire) {
            intakePower = Math.max(intakePower, 0) + FEED_INTAKE_BOOST;
        }
        robot.setIntake(intakePower);
        if (gamepad1.b != lastUnjam) {
            diag.event(gamepad1.b ? "UNJAM_PRESS" : "UNJAM_RELEASE", "indexer ticks " + robot.indexerTicks());
            lastUnjam = gamepad1.b;
        }

        // ---------------- Match reminders ----------------
        if (!endgameWarned && matchTimer.seconds() >= 100) {
            endgameWarned = true;
            gamepad1.rumbleBlips(3);
            setStatus("20 s left - PARK in the LOADING ZONE");
            diag.event("ENDGAME_WARNING", "20 s left");
        }
        diag.sample(robot, gamepad1, target, feeding ? "FEEDING" : (gamepad1.b ? "UNJAM" : ""));

        // ---------------- Telemetry ----------------
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Shot speed", "%s %.0f %+.0f = %.0f t/s   (D-pad: UP far, DOWN near, RIGHT +25, LEFT -25)",
                launcherPreset == LAUNCHER_FAR_VELOCITY ? "FAR" : "NEAR", launcherPreset, launcherTrim, launcherTarget);
        telemetry.addData("Launcher", "%s  actual %.0f t/s",
                !robot.isLauncherOn() ? "OFF" : (ready ? "READY" : "spinning"), robot.launcherVelocity());
        telemetry.addData("Windmill", gamepad1.b ? "UNJAM" : String.format("%s  shots %d%s", feeder.state(), feeder.shots(),
                feeder.jams() > 0 ? "  jams " + feeder.jams() : ""));
        telemetry.addData("Flower arm", BioBuzzRobot.armCalibrated() ? robot.armState().toString() : "NOT CALIBRATED");
        telemetry.addData("Drive", "%s", armDeployed ? "ARM OUT - precision" : (gamepad1.left_bumper ? "precision" : "normal"));
        if (vision == null) {
            telemetry.addData("HIVE aim", "camera not found");
        } else if (target != null) {
            telemetry.addData("HIVE aim", "%s  %d tags  bearing %.1f  range %.0f in%s",
                    target.cell, target.tagCount, target.bearingDeg, target.rangeIn, gamepad1.x ? "" : "  (hold X to aim)");
        } else {
            telemetry.addData("HIVE aim", "no %s CELL tags in view (any tags: %d)", alliance, vision.visibleTagCount());
        }
        String warning = robot.launcherWarning();
        if (warning != null) {
            telemetry.addData("WARNING", warning);
        }
        if (statusTimer.seconds() < 3 && !status.isEmpty()) {
            telemetry.addData("Note", status);
        }
        telemetry.addData("Battery", "%.2f V", robot.battery.getVoltage());
        if (diag.enabled()) {
            telemetry.addData("Diag log", diag.status());
        }
    }

    @Override
    public void stop() {
        robot.stopAll();
        if (vision != null) {
            vision.close();
        }
        diag.close();
    }

    private void setStatus(String s) {
        status = s;
        statusTimer.reset();
    }
}

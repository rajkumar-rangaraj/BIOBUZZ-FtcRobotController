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
 *   Right bumper (hold)   FIRE: spins the launcher up and feeds the windmill when at speed, and (with the CELL in
 *                         view) aimed and not turning; a held ball goes anyway after 1.5 s (AimGate)
 *   X / Square   (hold)   HIVE auto-aim: camera turns the robot; with a saved SHOT SPOT it also drives to that
 *                         distance and sets the spot's shot speed (push the left stick to take over)
 *   Back / Share          save the SHOT SPOT here: camera distance + current shot speed (needs HIVE tags in view)
 *   LB + Back / Share     same, AND teach AUTO this exact position (TeleOp must be INIT-ed at the AUTO start)
 *   Y / Triangle          launcher pre-spin ON/OFF
 *   A / Cross             flower arm DEPLOY / STOW (deploys only while the sticks are released)
 *   B / Circle   (hold)   unjam: windmill slowly backwards
 *   D-pad up / down       launcher FAR / NEAR preset
 *   D-pad right / left    launcher trim +/- LAUNCHER_TRIM_STEP ticks/s (15)
 *
 * During INIT: B / Circle = RED, A / Cross = BLUE (defaults to the alliance the last AUTO ran),
 *               Back / Share = diagnostics log on/off (files in /sdcard/BioBuzzLogs, see README_2),
 *               X / Square = aim gate on/off (off = FIRE never waits for the camera).
 * While the arm is DEPLOYED, driving is limited to precision speed.
 */
@TeleOp(name = "BioBuzz TeleOp", group = "BioBuzz")
public class BioBuzzTeleOp extends OpMode {

    private BioBuzzRobot robot;
    private HiveVision vision;
    private String visionError = null;
    private HiveVision.Alliance alliance = BioBuzzMatchState.alliance;

    private double launcherPreset = LAUNCHER_NEAR_VELOCITY;
    private String presetName = "NEAR";
    private TeleOpSettings settings = new TeleOpSettings();
    private boolean lastOnSpot = false;
    private double launcherTrim = 0;
    private boolean launcherToggledOn = false;
    private boolean readyRumbled = false;
    private boolean lastFallback = false;
    private boolean lastAimHold = false;
    private boolean lastGiveWay = false;
    private final AimGate aimGate = AimGate.standard();
    private boolean endgameWarned = false;
    private String status = "";
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime statusTimer = new ElapsedTime();
    private BioBuzzDiag diag = new BioBuzzDiag("teleop", false);
    private ShotFeeder feeder = ShotFeeder.standard();
    private boolean lastFire = false;
    private boolean lastAim = false;
    private boolean lastAimHadTarget = false;
    private boolean lastUnjam = false;
    private Pinpoint pinpoint;                                  // zeroed at INIT: SHARE teaches AUTO where to shoot
    private double imuAtInit;

    @Override
    public void init() {
        robot = new BioBuzzRobot(hardwareMap);
        settings = TeleOpSettings.load();
        feeder = ShotFeeder.fromSettings(settings);
        pinpoint = Pinpoint.tryCreate(hardwareMap);
        if (pinpoint != null) {
            pinpoint.reset();                                   // robot still at INIT (AUTO start position to teach AUTO)
        }
        imuAtInit = robot.headingDeg();
        try {
            vision = new HiveVision(hardwareMap);
            vision.setAimTrimDeg(settings.aimTrimDeg);
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
        if (gamepad1.xWasPressed()) {
            aimGate.setEnabled(!aimGate.enabled());
        }
        telemetry.addData("Alliance", "%s   (B/Circle = RED, A/Cross = BLUE)  <-- CHECK", alliance);
        telemetry.addData("Aim gate", "%s   (X/Square toggles; OFF = fire without the camera check)",
                aimGate.enabled() ? "ON" : "OFF");
        telemetry.addData("Shot spot", settings.hasSpot()
                ? String.format("%.0f in @ %.0f t/s (hold X to go there)", settings.spotRangeIn, settings.spotSpeed)
                : "none saved (press Share at a spot that scores)");
        telemetry.addData("Diagnostics log", "%s   (Back/Share toggles)", BioBuzzMatchState.diagnostics ? "ON" : "off");
        telemetry.addData("Flower arm", BioBuzzRobot.armCalibrated() ? "calibrated" : "NOT CALIBRATED - arm disabled");
        telemetry.addData("Camera", vision == null ? "NOT FOUND: " + visionError : "ok");
        telemetry.addData("Battery", "%.2f V", robot.battery.getVoltage());
    }

    @Override
    public void start() {
        BioBuzzMatchState.rememberAlliance(alliance);
        settings.alliance = alliance;
        matchTimer.reset();
        diag = new BioBuzzDiag("teleop", BioBuzzMatchState.diagnostics);
        diag.event("CONFIG", String.format("alliance=%s arm_calibrated=%s camera=%s launcher_near=%.0f far=%.0f battery=%.2f"
                        + " aim_trim=%.1f feed_power=%.2f feed_pause=%.2f spot=%s aim_gate=%s",
                alliance, BioBuzzRobot.armCalibrated(), vision == null ? "MISSING " + visionError : "ok",
                LAUNCHER_NEAR_VELOCITY, LAUNCHER_FAR_VELOCITY, robot.battery.getVoltage(),
                settings.aimTrimDeg, settings.feedPower, settings.feedPauseSec,
                settings.hasSpot() ? String.format("%.1fin@%.0f", settings.spotRangeIn, settings.spotSpeed) : "none",
                aimGate.enabled() ? "on" : "off"));
        robot.stowArm();
    }

    @Override
    public void loop() {
        if (vision != null) {
            vision.update();
        }
        if (pinpoint != null) {
            pinpoint.update();
        }

        // ---------------- Launcher ----------------
        if (gamepad1.yWasPressed()) {
            launcherToggledOn = !launcherToggledOn;
            diag.event("LAUNCHER", launcherToggledOn ? "pre-spin ON" : "pre-spin OFF");
        }
        if (gamepad1.dpadUpWasPressed()) {
            launcherPreset = LAUNCHER_FAR_VELOCITY;
            presetName = "FAR";
            launcherTrim = 0;
            diag.event("PRESET", "FAR " + LAUNCHER_FAR_VELOCITY);
        } else if (gamepad1.dpadDownWasPressed()) {
            launcherPreset = LAUNCHER_NEAR_VELOCITY;
            presetName = "NEAR";
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
        if (!robot.isLauncherOn()) {
            readyRumbled = false;
        } else if (ready && !readyRumbled) {                    // once per spin-up: READY now drops after every shot
            readyRumbled = true;
            gamepad1.rumble(250);
            diag.event("LAUNCHER_READY", String.format("%.0f t/s", robot.launcherVelocity()));
        }
        boolean fallback = ready && robot.launcherReadyByFallback();
        if (fallback && !lastFallback) {
            diag.event("READY_FALLBACK", String.format("target %.0f vel %.0f: speed never settled within %.0f t/s",
                    launcherTarget, robot.launcherVelocity(), LAUNCHER_READY_TOLERANCE));
        }
        lastFallback = fallback;
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
        boolean aim = gamepad1.x;
        if (aim && !lastAim && settings.hasSpot()) {
            launcherPreset = settings.spotSpeed;
            presetName = "SPOT";
            launcherTrim = 0;
            diag.event("PRESET", "SPOT " + settings.spotSpeed);
        }
        boolean onSpot = false;
        double floorIn = target == null ? Double.NaN : target.horizontalIn();
        if (aim && target != null) {
            turn = HiveVision.aimTurnCw(target.bearingDeg);
            if (settings.hasSpot()) {
                double toSpot = MotionController.spotForward(floorIn, settings.spotRangeIn);
                if (Math.abs(gamepad1.left_stick_y) < 0.1 && !armDeployed) {
                    forward = toSpot;
                }
                onSpot = toSpot == 0 && turn == 0;
            }
        }
        if (onSpot && !lastOnSpot) {
            gamepad1.rumble(400);
            diag.event("ON_SPOT", String.format("floor %.1f bearing %.1f", floorIn, target.bearingDeg));
        }
        lastOnSpot = onSpot;
        if (gamepad1.backWasPressed()) {
            if (target == null) {
                setStatus("Can't save the spot: no " + alliance + " HIVE tags in view");
                gamepad1.rumbleBlips(2);
            } else {
                settings.alliance = alliance;
                settings.spotRangeIn = floorIn;
                settings.spotSpeed = launcherTarget;
                String poseNote = "";
                if (gamepad1.left_bumper) {                     // LB + SHARE: also teach AUTO this exact position
                    double imuHeading = MecanumMath.normalizeDegrees(robot.headingDeg() - imuAtInit);
                    if (pinpoint == null || !pinpoint.ok()) {
                        poseNote = " - AUTO position NOT saved (Pinpoint not ready)";
                    } else if (Math.abs(MecanumMath.normalizeDegrees(pinpoint.headingDeg() - imuHeading)) > 5) {
                        poseNote = " - AUTO position NOT saved (Pinpoint moved during INIT: restart TeleOp)";
                    } else {
                        settings.shootFwdIn = pinpoint.xIn();
                        settings.shootLeftIn = pinpoint.yIn();
                        settings.shootHeadingDeg = imuHeading;
                        poseNote = String.format(" + AUTO position fwd %.0f left %.0f hdg %.0f",
                                pinpoint.xIn(), pinpoint.yIn(), imuHeading);
                    }
                }
                boolean saved = settings.trySave();
                setStatus(saved ? String.format("SHOT SPOT saved: %.0f in @ %.0f t/s%s", floorIn, launcherTarget, poseNote)
                        : "SHOT SPOT save FAILED");
                diag.event("SPOT_SAVED", String.format("floor %.1f range %.1f speed %.0f saved=%s%s", floorIn, target.rangeIn,
                        launcherTarget, saved, poseNote));
                gamepad1.rumble(600);
            }
        }
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
        // Aim gate (AimGate): with our CELL in view the next ball waits for a good, steady aim; holding X with a saved
        // spot also waits until the robot is on the spot.
        boolean spotHold = aim && settings.hasSpot() && target != null && !onSpot;
        boolean aimOk = aimGate.update(matchTimer.seconds(), fire, ready, target != null,
                target != null ? target.bearingDeg : 0, turn) && !spotHold;
        boolean aimHold = fire && ready && !aimOk;
        if (aimHold != lastAimHold) {
            diag.event(aimHold ? "AIM_HOLD" : "AIM_HOLD_END", (spotHold ? "to spot " : aimGate.state() + " ")
                    + (target != null ? String.format("bearing %.1f", target.bearingDeg) : "no target"));
            if (aimHold) {
                gamepad1.rumbleBlips(1);
            }
            lastAimHold = aimHold;
        }
        boolean giveWay = aimGate.gaveWay();
        if (giveWay && !lastGiveWay) {
            diag.event("AIM_GIVEWAY", target != null ? String.format("bearing %.1f", target.bearingDeg) : "CELL lost");
        }
        lastGiveWay = giveWay;
        double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
        if (gamepad1.b) {
            feeder.reset();
            robot.setIndexerPower(-INDEXER_UNJAM_POWER);
        } else {
            robot.setIndexerPower(feeder.update(matchTimer.seconds(), robot.indexerTicks(), ready && aimOk, fire));
            String feedEvent = feeder.event();
            if (feedEvent != null) {
                diag.event("FEED", feedEvent + String.format(" | launcher %.0f t/s", robot.launcherVelocity()));
                if (feedEvent.startsWith("SHOT")) {
                    aimGate.ballFed();                          // the next ball earns its own give-way time
                }
                if (feedEvent.startsWith("JAM")) {
                    setStatus("Windmill " + feedEvent + " - hold B to unjam");
                    gamepad1.rumbleBlips(1);
                }
            }
        }
        boolean feeding = feeder.feeding();
        if (fire && !aimHold) {                                 // don't press balls into a windmill that is held
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
        telemetry.addData("Shot speed", "%s %.0f %+.0f = %.0f t/s   (D-pad: UP far, DOWN near, RIGHT +%.0f, LEFT -%.0f)",
                presetName, launcherPreset, launcherTrim, launcherTarget, LAUNCHER_TRIM_STEP, LAUNCHER_TRIM_STEP);
        telemetry.addData("Launcher", "%s  actual %.0f t/s",
                !robot.isLauncherOn() ? "OFF" : (ready ? (fallback ? "READY (speed not settled)" : "READY") : "settling"),
                robot.launcherVelocity());
        if (aimHold) {
            String why;
            if (spotHold) {
                why = String.format("driving to the SHOT SPOT (%+.0f in)", floorIn - settings.spotRangeIn);
            } else if (aimGate.state() == AimGate.State.HOLD_AIM) {
                why = String.format("aim %.1f deg off: hold X, or turn %s", target.bearingDeg,
                        target.bearingDeg > 0 ? "LEFT" : "RIGHT");
            } else if (aimGate.state() == AimGate.State.HOLD_TURNING) {
                why = "robot still turning";
            } else {
                why = "CELL just lost - wait";
            }
            telemetry.addData("FIRE HELD", "%s  (fires anyway after %.1f s)", why, AIM_GATE_GIVEWAY_SEC);
        } else if (!aimGate.enabled()) {
            telemetry.addData("Aim gate", "OFF (X in INIT)");
        }
        telemetry.addData("Windmill", gamepad1.b ? "UNJAM" : String.format("%s  shots %d%s", feeder.state(), feeder.shots(),
                feeder.jams() > 0 ? "  jams " + feeder.jams() : ""));
        telemetry.addData("Flower arm", BioBuzzRobot.armCalibrated() ? robot.armState().toString() : "NOT CALIBRATED");
        telemetry.addData("Drive", "%s", armDeployed ? "ARM OUT - precision" : (gamepad1.left_bumper ? "precision" : "normal"));
        if (vision == null) {
            telemetry.addData("HIVE aim", "camera not found");
        } else if (target != null) {
            telemetry.addData("HIVE aim", "%s  %d tags  bearing %.1f  floor %.0f in%s",
                    target.cell, target.tagCount, target.bearingDeg, floorIn, gamepad1.x ? "" : "  (hold X to aim)");
        } else {
            telemetry.addData("HIVE aim", "no %s CELL tags in view (any tags: %d)", alliance, vision.visibleTagCount());
        }
        if (settings.hasSpot()) {
            telemetry.addData("Shot spot", "%.0f in @ %.0f t/s%s", settings.spotRangeIn, settings.spotSpeed,
                    onSpot ? "  ** ON SPOT - FIRE **" : (target != null
                            ? String.format("  now %.0f in (%+.0f)", floorIn, floorIn - settings.spotRangeIn) : ""));
        } else {
            telemetry.addData("Shot spot", "none - press Share where shots score");
        }
        if (pinpoint != null) {
            telemetry.addData("From INIT", "fwd %.0f  left %.0f in  hdg %.0f  %s   (LB+Share teaches AUTO)%s",
                    pinpoint.xIn(), pinpoint.yIn(), MecanumMath.normalizeDegrees(robot.headingDeg() - imuAtInit),
                    pinpoint.ok() ? "" : pinpoint.status(),
                    settings.hasShootPose() ? String.format("  AUTO: %.0f/%.0f/%.0f", settings.shootFwdIn,
                            settings.shootLeftIn, settings.shootHeadingDeg) : "  AUTO: not taught");
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

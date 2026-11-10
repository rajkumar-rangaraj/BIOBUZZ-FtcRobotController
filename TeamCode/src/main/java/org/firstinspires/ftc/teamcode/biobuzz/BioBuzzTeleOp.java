package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * BIOBUZZ competition TeleOp - ONE gamepad (Xbox name / PlayStation symbol).
 *
 *   Left stick            drive + strafe. FIELD-CENTRIC by default: stick up = AWAY FROM THE DRIVERS, whichever way the
 *                         robot faces (FieldHeading: Pinpoint heading, IMU backup). While X aims at a CELL in view the
 *                         stick is robot-centric (sideways = sideways to the CELL). Robot-centric (L1 in INIT): stick
 *                         up = intake side (FRONT)
 *   Options               field-centric reset: point the INTAKE away from you and press (after a big hit or a hub
 *                         restart; the start heading comes from AUTO when it ran just before, else "as placed")
 *   Right stick X         turn
 *   Left bumper  (hold)   precision drive
 *   Right trigger         intake IN (analog)
 *   Left trigger          intake OUT (analog) - half pull = the flower "back-out" reverse
 *   Right bumper (hold)   FIRE: spins the launcher up and feeds the windmill when at speed, and (with the CELL in
 *                         view) aimed, not turning and - while X drives to a SHOT SPOT - at its distance; a held ball
 *                         goes anyway 1.5 s after the hold starts (AimGate)
 *   X / Square   (hold)   HIVE auto-aim: camera turns the robot; with a saved SHOT SPOT it also drives to that
 *                         distance (push the left stick to take over). Also switches the shot speed back to AUTO
 *   Back / Share          save the SHOT SPOT here: camera distance + current shot speed (needs HIVE tags in view)
 *   LB + Back / Share     same, AND teach AUTO this exact position (TeleOp must be INIT-ed at the AUTO start)
 *   Y / Triangle          launcher pre-spin ON/OFF
 *   A / Cross             flower arm DEPLOY / STOW (deploys only while the sticks are released)
 *   B / Circle   (hold)   unjam: windmill slowly backwards
 *   D-pad up / down       MANUAL shot speed FAR / NEAR (the camera distance is ignored until X is pressed)
 *   D-pad right / left    shot speed trim +/- LAUNCHER_TRIM_STEP ticks/s (5); stays on top of AUTO for the match
 *   LB + D-pad            what the ball did, for the diagnostics log (ShotResults): UP = IN, DOWN = SHORT (too slow),
 *                         RIGHT = LONG (too fast), LEFT = SIDE (left / right / bounced out). Goes to the oldest ball of
 *                         the last 10 s without a result, so a volley is labelled in the order the balls flew.
 *                         tools/shot_results.py turns the labels into speed-table corrections.
 *
 * Shot speed AUTO (default): set from the camera's floor distance to our CELL (ShotSpeed, AUTO_SPEED_* table), kept
 * 3 s after the CELL drops out of view, then the NEAR / FAR preset; it doesn't change while FIRE is held.
 *
 * During INIT: B / Circle = RED, A / Cross = BLUE (defaults to the alliance the last AUTO ran),
 *               Back / Share = diagnostics log on/off (files in /sdcard/BioBuzzLogs, see README_2),
 *               X / Square = aim gate on/off (off = FIRE never waits for the camera),
 *               Y / Triangle = auto shot speed on/off (off = NEAR / FAR presets only, as before 7 Nov),
 *               LB / L1 = field-centric driving on/off (ON by default; kept until the robot app restarts),
 *               Options = the intake points away from you now (field-centric start heading).
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
    private boolean aimHoldBlipped = false;
    private boolean lastGiveWay = false;
    private final AimGate aimGate = AimGate.standard();
    private final ShotSpeed shotSpeed = ShotSpeed.standard();
    private double lastBaseSpeed = Double.NaN;
    private boolean lastSpeedAuto = false;
    private boolean endgameWarned = false;
    private String status = "";
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime statusTimer = new ElapsedTime();
    private BioBuzzDiag diag = new BioBuzzDiag("teleop", false);
    private ShotFeeder feeder = ShotFeeder.standard();
    private final BallCounter ballCounter = BallCounter.forTuning();      // launched balls (flywheel dips), diagnostics
    private final GrabTracker grabs = GrabTracker.standard();             // how the flywheel took each ball (GRAB event)
    private double lastFeedingT = -1e9;
    private final ShotResults shotResults = new ShotResults(SHOT_LABEL_WINDOW_SEC);
    private final double[] velT = new double[24], velV = new double[24];   // recent flywheel readings: speed at launch
    private int velN = 0;
    private HiveVision.Target lastSeenTarget = null;                       // the CELL as last seen (a ball can leave
    private double lastSeenTargetT = -1e9;                                 // just after it drops out of view)
    private boolean lastFire = false;
    private boolean lastAim = false;
    private boolean lastAimHadTarget = false;
    private boolean lastUnjam = false;
    private Pinpoint pinpoint;                                  // zeroed at INIT: SHARE teaches AUTO where to shoot
    private final PinpointInitCheck pinpointCheck = PinpointInitCheck.standard();
    private int pinpointRecalibrations = 0;
    private double imuAtInit;
    private FieldHeading fieldHeading;                          // field-centric driving: heading relative to the drivers
    private String headingFrom;
    private double optionsDownSince = Double.NaN;               // Options held 1 s: field-centric on/off
    private boolean optionsHoldDone = false;

    @Override
    public void init() {
        robot = new BioBuzzRobot(hardwareMap);
        settings = TeleOpSettings.load();
        feeder = ShotFeeder.fromSettings(settings).skipEmptyPockets(SHOT_SKIP_EMPTY_PAUSE, settings.feedEmptyWaitSec);
        pinpoint = Pinpoint.tryCreate(hardwareMap);
        if (pinpoint != null) {
            pinpoint.reset();                                   // robot still at INIT (AUTO start position to teach AUTO)
        }
        imuAtInit = robot.headingDeg();
        double fromAuto = BioBuzzMatchState.autoHeading();      // AUTO just ran: the robot hasn't moved since
        fieldHeading = FieldHeading.standard(Double.isNaN(fromAuto) ? 0 : fromAuto);
        headingFrom = Double.isNaN(fromAuto) ? "as placed: the INTAKE must point AWAY from you"
                : String.format("from AUTO: intake %.0f deg %s of away", Math.abs(fromAuto), fromAuto > 0 ? "left" : "right");
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
        if (gamepad1.yWasPressed()) {
            shotSpeed.setAutoEnabled(!shotSpeed.autoEnabled());
        }
        if (gamepad1.leftBumperWasPressed()) {
            BioBuzzMatchState.fieldCentric = !BioBuzzMatchState.fieldCentric;
        }
        if (gamepad1.optionsWasPressed()) {                      // (SDK 12: options and start are one button with
            fieldHeading.reset();                                // two latches: reading both would reset twice)
            BioBuzzMatchState.clearAutoHeading();                // the drivers' choice wins, even after a re-INIT
            headingFrom = "set in INIT: the intake points away from you";
        }
        if (pinpoint != null) {                                  // a gyro calibrated while moved drifts all match
            pinpoint.update();
            if (pinpointCheck.update(pinpoint.ok(), pinpoint.headingDeg(), robot.headingDeg())) {
                pinpoint.reset();
                pinpointRecalibrations++;
                imuAtInit = robot.headingDeg();
            }
        }
        telemetry.addData("Alliance", "%s   (B/Circle = RED, A/Cross = BLUE)  <-- CHECK", alliance);
        telemetry.addData("Drive", BioBuzzMatchState.fieldCentric
                ? "FIELD-CENTRIC: stick up = away from you   (L1 toggles; in the match: hold Options 1 s)"
                : "robot-centric: stick up = intake side   (L1 toggles; in the match: hold Options 1 s)");
        if (BioBuzzMatchState.fieldCentric) {
            telemetry.addData("Start heading", "%s   (Options = the intake points away from you now)", headingFrom);
        }
        if (pinpoint != null) {
            telemetry.addData("Pinpoint gyro", !pinpoint.ok() ? "calibrating: keep the robot still"
                    : pinpointRecalibrations == 0 ? "ok"
                    : String.format("was drifting: recalibrated %d x - keep the robot STILL in INIT", pinpointRecalibrations));
        }
        telemetry.addData("Aim gate", "%s   (X/Square toggles; OFF = fire without the camera check)",
                aimGate.enabled() ? "ON" : "OFF");
        telemetry.addData("Shot speed", "%s   (Y/Triangle toggles; OFF = NEAR/FAR presets only)",
                !shotSpeed.autoEnabled() ? "presets only" : (vision == null ? "AUTO - camera missing, presets"
                        : "AUTO from the camera distance"));
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
        BioBuzzMatchState.clearAutoHeading();                   // used by this TeleOp: a later one starts "as placed"
        settings.alliance = alliance;
        matchTimer.reset();
        diag = new BioBuzzDiag("teleop", BioBuzzMatchState.diagnostics);
        diag.event("CONFIG", String.format("alliance=%s arm_calibrated=%s camera=%s launcher_near=%.0f far=%.0f battery=%.2f"
                        + " aim_trim=%.1f feed_power=%.2f feed_pause=%.2f feed_empty_wait=%s spot=%s aim_gate=%s auto_speed=%s"
                        + " field_centric=%s start_heading=\"%s\" pinpoint_recal=%d",
                alliance, BioBuzzRobot.armCalibrated(), vision == null ? "MISSING " + visionError : "ok",
                LAUNCHER_NEAR_VELOCITY, LAUNCHER_FAR_VELOCITY, robot.battery.getVoltage(),
                settings.aimTrimDeg, settings.feedPower, settings.feedPauseSec,
                SHOT_SKIP_EMPTY_PAUSE ? String.format("%.2f", settings.feedEmptyWaitSec) : "off",
                settings.hasSpot() ? String.format("%.1fin@%.0f", settings.spotRangeIn, settings.spotSpeed) : "none",
                aimGate.enabled() ? "on" : "off", shotSpeed.autoEnabled() ? "on" : "off",
                BioBuzzMatchState.fieldCentric ? "on" : "off", headingFrom, pinpointRecalibrations));
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
        // Field-centric: the IMU is read right after the Pinpoint, so the two headings agree while turning (FieldHeading
        // compares them); the value is reused by the drive and telemetry below. Read even while robot-centric: holding
        // Options can turn field-centric on at any time (review 10 Nov).
        double imuNow = robot.headingDeg();
        HiveVision.Target target = vision == null ? null : vision.target(alliance, null);   // every loop: logged + shown
        double floorIn = target == null ? Double.NaN : target.horizontalIn();

        // ---------------- Launcher ----------------
        if (gamepad1.yWasPressed()) {
            launcherToggledOn = !launcherToggledOn;
            diag.event("LAUNCHER", launcherToggledOn ? "pre-spin ON" : "pre-spin OFF");
        }
        if (gamepad1.left_bumper) {                             // LB + D-pad: what the ball did (diagnostics log)
            ShotResults.Result r = gamepad1.dpadUpWasPressed() ? ShotResults.Result.IN
                    : gamepad1.dpadDownWasPressed() ? ShotResults.Result.SHORT
                    : gamepad1.dpadRightWasPressed() ? ShotResults.Result.LONG
                    : gamepad1.dpadLeftWasPressed() ? ShotResults.Result.SIDE : null;
            if (r != null) {
                labelShot(r);
            }
        } else if (gamepad1.dpadUpWasPressed()) {
            launcherPreset = LAUNCHER_FAR_VELOCITY;
            presetName = "FAR";
            launcherTrim = 0;
            shotSpeed.setManual(true);
            diag.event("PRESET", "FAR " + LAUNCHER_FAR_VELOCITY + " (manual)");
        } else if (gamepad1.dpadDownWasPressed()) {
            launcherPreset = LAUNCHER_NEAR_VELOCITY;
            presetName = "NEAR";
            launcherTrim = 0;
            shotSpeed.setManual(true);
            diag.event("PRESET", "NEAR " + LAUNCHER_NEAR_VELOCITY + " (manual)");
        } else if (gamepad1.dpadRightWasPressed()) {
            launcherTrim += LAUNCHER_TRIM_STEP;
            diag.event("TRIM", String.format("%+.0f", launcherTrim));
        } else if (gamepad1.dpadLeftWasPressed()) {
            launcherTrim -= LAUNCHER_TRIM_STEP;
            diag.event("TRIM", String.format("%+.0f", launcherTrim));
        }
        boolean fire = gamepad1.right_bumper;
        double baseSpeed = shotSpeed.update(matchTimer.seconds(), target == null ? null : target.cell, floorIn, fire,
                launcherPreset);
        boolean speedAuto = shotSpeed.source() == ShotSpeed.Source.AUTO || shotSpeed.source() == ShotSpeed.Source.AUTO_KEPT;
        if (baseSpeed != lastBaseSpeed || speedAuto != lastSpeedAuto) {
            diag.event("SPEED", String.format("%s %.0f%s", shotSpeed.source(), baseSpeed,
                    speedAuto ? String.format(" (%.1f in %s)", shotSpeed.lastDistanceIn(), shotSpeed.cell()) : ""));
            lastBaseSpeed = baseSpeed;
            lastSpeedAuto = speedAuto;
        }
        double launcherTarget = Range.clip(baseSpeed + launcherTrim, 0, LAUNCHER_MAX_VELOCITY);
        robot.setLauncherVelocity(launcherToggledOn || fire ? launcherTarget : 0);
        boolean ready = robot.updateLauncherReady();            // one fresh decision per loop; logs reuse it
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
        boolean aim = gamepad1.x;
        // Field-centric: the heading relative to the drivers, every loop (ready even while robot-centric). The Pinpoint
        // and the IMU were read at the top of the loop.
        double driverHeading = fieldHeading.update(matchTimer.seconds(),
                pinpoint != null && pinpoint.ok() ? pinpoint.headingDeg() : Double.NaN, imuNow);
        if (fieldHeading.event() != null) {
            diag.event("DRIVE_HEADING", fieldHeading.event());
        }
        if (gamepad1.optionsWasPressed()) {
            double before = driverHeading;
            fieldHeading.reset();
            driverHeading = fieldHeading.heading();
            gamepad1.rumble(200);
            setStatus("Field-centric reset: the way the intake points now = away from you");
            diag.event("DRIVE_HEADING", String.format("reset by the driver (was %.0f deg)", before));
        }
        if (gamepad1.options) {                                 // held: field-centric on/off (if "up" ever goes wrong)
            if (Double.isNaN(optionsDownSince)) {
                optionsDownSince = matchTimer.seconds();
            } else if (!optionsHoldDone && matchTimer.seconds() - optionsDownSince >= FIELD_CENTRIC_HOLD_SEC) {
                optionsHoldDone = true;
                BioBuzzMatchState.fieldCentric = !BioBuzzMatchState.fieldCentric;
                gamepad1.rumbleBlips(BioBuzzMatchState.fieldCentric ? 1 : 2);
                setStatus(BioBuzzMatchState.fieldCentric ? "FIELD-CENTRIC on: stick up = away from you"
                        : "robot-centric: stick up = intake side (hold Options 1 s for field-centric)");
                diag.event("DRIVE_MODE", BioBuzzMatchState.fieldCentric ? "field-centric (Options held)"
                        : "robot-centric (Options held)");
            }
        } else {
            optionsDownSince = Double.NaN;
            optionsHoldDone = false;
        }
        boolean fieldDrive = BioBuzzMatchState.fieldCentric && !Double.isNaN(driverHeading) && !(aim && target != null);
        if (fieldDrive) {                                       // stick up = away from the drivers
            double[] r = MecanumMath.driverToRobot(forward, strafe, driverHeading);
            forward = r[0];
            strafe = r[1];
        }
        robot.lastDriveHeadingDeg = driverHeading;
        robot.lastFieldDrive = fieldDrive;
        if (aim && !lastAim) {
            if (shotSpeed.autoEnabled() && vision != null) {
                if (shotSpeed.manual()) {                       // aiming with the camera = the camera's shot speed again
                    shotSpeed.setManual(false);
                    launcherTrim = 0;
                    diag.event("PRESET", "AUTO (camera distance)");
                }
            } else if (settings.hasSpot()) {                    // presets only: the SHOT SPOT's saved speed
                launcherPreset = settings.spotSpeed;
                presetName = "SPOT";
                launcherTrim = 0;
                diag.event("PRESET", "SPOT " + settings.spotSpeed);
            }
        }
        boolean onSpot = false;
        boolean spotHold = false;                               // X is still driving to the saved distance
        if (aim && target != null) {
            turn = HiveVision.aimTurnCw(target.bearingDeg);
            if (settings.hasSpot()) {
                double toSpot = MotionController.spotForward(floorIn, settings.spotRangeIn);
                if (Math.abs(gamepad1.left_stick_y) < 0.1 && !armDeployed) {
                    forward = toSpot;
                    spotHold = toSpot != 0;                     // the heading is AimGate's own check
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
        // Aim gate (AimGate): with our CELL in view the next ball waits for a good, steady aim and, while X drives to a
        // saved spot, for the right distance. Every hold gives way after AIM_GATE_GIVEWAY_SEC; X in INIT turns it off.
        boolean aimOk = aimGate.update(matchTimer.seconds(), fire, ready, target != null,
                target != null ? target.bearingDeg : 0, turn, spotHold);
        boolean aimHold = fire && ready && !aimOk;
        if (aimHold != lastAimHold) {
            diag.event(aimHold ? "AIM_HOLD" : "AIM_HOLD_END", aimGate.state() + " "
                    + (target != null ? String.format("bearing %.1f floor %.1f", target.bearingDeg, floorIn) : "no target"));
            if (aimHold && !aimHoldBlipped) {                   // one blip per FIRE press, not per hold
                gamepad1.rumbleBlips(1);
                aimHoldBlipped = true;
            }
            lastAimHold = aimHold;
        }
        if (!fire) {
            aimHoldBlipped = false;
        }
        boolean giveWay = aimGate.gaveWay();
        if (giveWay && !lastGiveWay) {
            diag.event("AIM_GIVEWAY", target != null ? String.format("bearing %.1f", target.bearingDeg) : "CELL lost");
        }
        lastGiveWay = giveWay;
        double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
        int indexerTicks = robot.indexerTicks();                // once per loop: the feeder and the grab log share it
        if (gamepad1.b) {
            feeder.reset();
            robot.setIndexerPower(-INDEXER_UNJAM_POWER);
        } else {
            robot.setIndexerPower(feeder.update(matchTimer.seconds(), indexerTicks, ready, aimOk, fire));
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

        // ---------------- Launched balls (diagnostics: what each shot had; LB + D-pad adds the result) ----------------
        double now = matchTimer.seconds();
        double flywheel = robot.launcherVelocity();
        velT[velN % velT.length] = now;
        velV[velN % velV.length] = flywheel;
        velN++;
        if (target != null) {
            lastSeenTarget = target;
            lastSeenTargetT = now;
        }
        // The grab (GrabTracker): the launcher current is read only around a pocket and while a dip is measured, so the
        // extra ~2-3 ms per loop is spent only while firing.
        if (feeding) {
            lastFeedingT = now;
        }
        boolean watchGrab = robot.isLauncherOn() && (feeding || grabs.active() || now - lastFeedingT <= GRAB_WATCH_SEC);
        double launcherAmps = watchGrab ? robot.launcherCurrentAmps() : Double.NaN;
        robot.lastLauncherAmps = launcherAmps;
        double flywheelTarget = robot.isLauncherOn() ? robot.launcherTarget() : 0;
        grabs.sample(now, flywheel, flywheelTarget, indexerTicks, feeder.targetTicks(), feeding, launcherAmps);
        if (ballCounter.update(now, feeding, flywheelTarget, flywheel)) {
            HiveVision.Target seen = now - lastSeenTargetT <= 1.0 ? lastSeenTarget : null;
            double ballFloorIn = seen == null ? Double.NaN : seen.horizontalIn();
            double table = Double.isNaN(ballFloorIn) ? Double.NaN : shotSpeed.tableSpeed(ballFloorIn);
            double launchVel = Double.NEGATIVE_INFINITY, lowVel = Double.POSITIVE_INFINITY;
            for (int i = 0; i < Math.min(velN, velT.length); i++) {          // the dip started up to ~0.3 s ago
                if (now - velT[i] <= 0.35) {
                    launchVel = Math.max(launchVel, velV[i]);
                    lowVel = Math.min(lowVel, velV[i]);
                }
            }
            ShotResults.Ball ball = shotResults.launched(now, ballFloorIn, launcherTarget, launcherTrim, table);
            grabs.ballCounted(now, ball.number);
            double move = 0;
            for (double p : robot.lastDrivePower) {
                move = Math.max(move, Math.abs(p));
            }
            diag.event("BALL", String.format("ball=%d cell=%s tags=%d floor_in=%.1f range_in=%.1f bearing=%.1f seen_ago_s=%.1f"
                            + " source=%s base=%.0f trim=%+.0f target=%.0f table=%.0f vel=%.0f dip=%.0f ready=%s fallback=%s"
                            + " aim_gate=%s battery=%.2f imu_hdg=%.1f move=%.2f turn_cmd=%+.2f aim_x=%s%s",
                    ball.number, seen == null ? "none" : seen.cell, seen == null ? 0 : seen.tagCount,
                    ballFloorIn, seen == null ? Double.NaN : seen.rangeIn, seen == null ? Double.NaN : seen.bearingDeg,
                    seen == null ? Double.NaN : now - lastSeenTargetT, shotSpeed.source(), baseSpeed, launcherTrim,
                    launcherTarget, table, launchVel, launchVel - lowVel, ready, fallback,
                    !aimGate.enabled() ? "off" : (aimGate.gaveWay() ? "gave_way" : "ok"), robot.battery.getVoltage(),
                    MecanumMath.normalizeDegrees(robot.headingDeg() - imuAtInit), move, turn, gamepad1.x,
                    pinpoint != null && pinpoint.ok() ? String.format(" pinpoint_fwd_in=%.1f pinpoint_left_in=%.1f pinpoint_hdg=%.1f",
                            pinpoint.xIn(), pinpoint.yIn(), pinpoint.headingDeg()) : ""));
        }
        for (GrabTracker.Grab g = grabs.poll(); g != null; g = grabs.poll()) {
            diag.event("GRAB", String.format("ball=%d t_grab=%.3f gap_s=%.3f pre=%.0f target=%.0f launch_err=%+.0f min=%.0f"
                            + " depth=%.0f energy=%.3f t_min_s=%.3f below_s=%.3f recovery_s=%.3f drops=%d overlapped=%s"
                            + " target_changed=%s windmill_ticks_left=%.0f feeding=%s since_pocket_end_s=%.3f"
                            + " amps_base=%.2f amps_peak=%.2f",
                    g.ball, g.tStart, g.startGapSec, g.preTps, g.targetTps, g.launchErrorTps(), g.minTps, g.depthTps(),
                    g.energyFraction(), g.tMin - g.tStart, g.belowSec, g.recoverySec, g.drops, g.overlapped,
                    g.targetChanged, g.ticksToBoundary, g.feedingAtStart, g.sincePocketEndSec, g.baseAmps, g.peakAmps));
        }

        // ---------------- Match reminders ----------------
        if (!endgameWarned && matchTimer.seconds() >= 100) {
            endgameWarned = true;
            gamepad1.rumbleBlips(3);
            setStatus("20 s left - PARK in the LOADING ZONE");
            diag.event("ENDGAME_WARNING", "20 s left");
        }
        diag.sample(robot, gamepad1, target, feeding ? "FEEDING " + Math.round(feeder.targetTicks()) : (gamepad1.b ? "UNJAM" : ""));

        // ---------------- Telemetry ----------------
        telemetry.addData("Alliance", alliance);
        String speedName;
        switch (shotSpeed.source()) {
            case AUTO:
                speedName = String.format("AUTO %.0f in", shotSpeed.lastDistanceIn());
                break;
            case AUTO_KEPT:
                speedName = String.format("AUTO %.0f in (CELL lost)", shotSpeed.lastDistanceIn());
                break;
            case MANUAL:
                speedName = "MANUAL " + presetName;
                break;
            default:
                speedName = presetName + (shotSpeed.autoEnabled() ? " (no CELL)" : "");
        }
        telemetry.addData("Shot speed", "%s %.0f %+.0f = %.0f t/s   (D-pad: UP far, DOWN near, RIGHT +%.0f, LEFT -%.0f%s)",
                speedName, baseSpeed, launcherTrim, launcherTarget, LAUNCHER_TRIM_STEP, LAUNCHER_TRIM_STEP,
                shotSpeed.manual() ? "; X = camera speed" : "");
        telemetry.addData("Launcher", "%s  actual %.0f t/s",
                !robot.isLauncherOn() ? "OFF" : (ready ? (fallback ? "READY (speed not settled)" : "READY") : "settling"),
                robot.launcherVelocity());
        if (aimHold) {
            String why;
            if (aimGate.state() == AimGate.State.HOLD_SPOT) {
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
        int waiting = shotResults.waiting(matchTimer.seconds());
        telemetry.addData("Balls", "%d out, results %d (IN %d, short %d, long %d, side %d)%s", shotResults.balls(),
                shotResults.labelled(), shotResults.count(ShotResults.Result.IN), shotResults.count(ShotResults.Result.SHORT),
                shotResults.count(ShotResults.Result.LONG), shotResults.count(ShotResults.Result.SIDE),
                waiting > 0 ? String.format("   %d waiting: LB + D-pad UP in, DOWN short, RIGHT long, LEFT side", waiting) : "");
        telemetry.addData("Flower arm", BioBuzzRobot.armCalibrated() ? robot.armState().toString() : "NOT CALIBRATED");
        telemetry.addData("Drive", "%s, %s", armDeployed ? "ARM OUT - precision" : (gamepad1.left_bumper ? "precision" : "normal"),
                !BioBuzzMatchState.fieldCentric ? "robot-centric   (hold Options 1 s: field-centric)"
                        : Double.isNaN(driverHeading) ? "robot-centric: NO HEADING (Pinpoint + IMU)"
                        : String.format("FIELD-CENTRIC, intake %+.0f deg from away (%s)%s   Options = intake away now,"
                                        + " hold 1 s = off",
                                driverHeading, fieldHeading.source(), fieldDrive ? "" : ", robot-centric while aiming"));
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
                    pinpoint.xIn(), pinpoint.yIn(), MecanumMath.normalizeDegrees(imuNow - imuAtInit),
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

    /** LB + D-pad: the result of the oldest launched ball of the last SHOT_LABEL_WINDOW_SEC that has none yet. */
    private void labelShot(ShotResults.Result r) {
        double now = matchTimer.seconds();
        ShotResults.Ball b = shotResults.label(now, r);
        if (b == null) {
            setStatus(String.format("%s not saved: no ball from the last %.0f s waits for a result", r, SHOT_LABEL_WINDOW_SEC));
            gamepad1.rumbleBlips(2);
            diag.event("SHOT_RESULT_IGNORED", r + " (no unlabelled ball in the window)");
            return;
        }
        gamepad1.rumble(150);
        setStatus(String.format("Ball %d: %s  (%.0f in, %.0f t/s)", b.number, r, b.floorIn, b.targetTps));
        diag.event("SHOT_RESULT", String.format("ball=%d result=%s floor_in=%.1f target=%.0f trim=%+.0f table=%.0f"
                        + " offset=%+.0f age_s=%.1f", b.number, r, b.floorIn, b.targetTps, b.trimTps, b.tableTps,
                b.targetTps - b.tableTps, now - b.t));
    }
}

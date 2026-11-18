package org.firstinspires.ftc.teamcode.biobuzz.pp;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.AutoField;
import org.firstinspires.ftc.teamcode.biobuzz.AimGate;
import org.firstinspires.ftc.teamcode.biobuzz.BallCounter;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzDiag;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzMatchState;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzRobot;
import org.firstinspires.ftc.teamcode.biobuzz.FieldHeading;
import org.firstinspires.ftc.teamcode.biobuzz.GrabTracker;
import org.firstinspires.ftc.teamcode.biobuzz.HiveVision;
import org.firstinspires.ftc.teamcode.biobuzz.MecanumMath;
import org.firstinspires.ftc.teamcode.biobuzz.MotionController;
import org.firstinspires.ftc.teamcode.biobuzz.Pinpoint;
import org.firstinspires.ftc.teamcode.biobuzz.PinpointInitCheck;
import org.firstinspires.ftc.teamcode.biobuzz.ShotFeeder;
import org.firstinspires.ftc.teamcode.biobuzz.ShotResults;
import org.firstinspires.ftc.teamcode.biobuzz.ShotSpeed;
import org.firstinspires.ftc.teamcode.biobuzz.TapHold;
import org.firstinspires.ftc.teamcode.biobuzz.TeleOpSettings;

import java.util.List;

/*
 * BIOBUZZ TeleOp PP - aims and sets the flywheel speed from the PINPOINT field pose, corrected by the HIVE AprilTags.
 * ONE gamepad. The camera TeleOp ("BioBuzz TeleOp") is unchanged: switch back to it any time.
 *
 * What is different from BioBuzz TeleOp:
 *   - The robot knows where it is (Pinpoint, every loop) and where our UP CELL is, so the flywheel is at the right speed
 *     while driving there and the aim turns onto the CELL before the camera sees it. Whenever the camera sees a CELL,
 *     the position is corrected. The flywheel table (ShotSpeed) and the camera distance are exactly as in BioBuzz TeleOp.
 *   - RB = VOLLEY: one tap fires every ball (no holding, no re-pressing). Tap RB again, or B, to stop. It stops by
 *     itself when the windmill turns VOLLEY_EMPTY_POCKETS pockets with nothing in them, or on a jam.
 *   - Each ball waits until the robot is aimed (Pinpoint heading, and the camera when it sees the CELL) and the
 *     flywheel has settled (no overshoot after the last ball). A ball held 2 s goes anyway.
 *   - The aim holds against a push (the heading lock runs every loop on the Pinpoint heading).
 *   - Faster loop: hub reads cached once per loop, IMU only at 10 Hz (backup), telemetry at 4 Hz.
 *   - If the Pinpoint fails it falls back to the camera-only aim of BioBuzz TeleOp (screen: "CAMERA ONLY").
 *
 *   Left stick            drive (robot-centric; field-centric: hold Options 1 s)
 *   Right stick X         turn - always wins over the auto aim
 *   Left bumper  (hold)   precision drive
 *   Right trigger / Left  intake IN / OUT
 *   Right bumper (TAP)    VOLLEY on / off
 *   X / Square   (hold)   aim only (no firing); also the camera's shot speed again (clears a D-pad preset)
 *   Y / Triangle          auto pre-spin on / off (on: the flywheel runs at the shot speed while near our CELL)
 *   A / Cross             flower arm DEPLOY / STOW
 *   B / Circle   (hold)   unjam (windmill backwards) and stop the volley
 *   D-pad up / down       MANUAL shot speed FAR / NEAR;  right / left: trim +/- 10
 *   LB + D-pad            what the ball did (IN / SHORT / LONG / SIDE), for the log
 *   Back / Share          save the SHOT SPOT (camera distance + speed) - RB / X drive to it, as in BioBuzz TeleOp
 *   Options               tap = the intake points AWAY from you now (sets the heading); hold 1 s = field-centric on/off
 *
 * INIT: B = RED, A = BLUE; D-pad LEFT / RIGHT = where the robot starts (AUTO's pose, CELL wall, ALLIANCE wall, or
 * "camera": the first CELL sighting places it, intake pointing AWAY from you); Y = auto pre-spin; X = aim gate;
 * LB = field-centric; Back = diagnostics log; Options = the intake points away from you now. KEEP THE ROBOT STILL
 * (the Pinpoint calibrates its gyro).
 */
@TeleOp(name = "BioBuzz TeleOp PP", group = "BioBuzz")
public class BioBuzzPinpointTeleOp extends OpMode {

    enum Start { AUTO("AUTO's last pose"), CAMERA("CAMERA: first CELL sighting places the robot (intake AWAY from you)"),
        CELL_WALL("CELL wall start (as AUTO)"), ALLIANCE_WALL("ALLIANCE wall start (as AUTO)");

        final String label;

        Start(String label) {
            this.label = label;
        }
    }

    enum Mode { POSE, HEADING_ONLY, CAMERA_ONLY }

    private BioBuzzRobot robot;
    private List<LynxModule> hubs;
    private HiveVision vision;
    private String visionError = null;
    private HiveVision.Alliance alliance = BioBuzzMatchState.alliance;
    private TeleOpSettings settings = new TeleOpSettings();

    private Pinpoint pinpoint;
    private final PinpointInitCheck pinpointCheck = PinpointInitCheck.standard();
    private int pinpointRecalibrations = 0;
    private double[] autoPose;
    private Start start = Start.CAMERA;
    private boolean headingSetInInit = false;
    private final FieldPose pose = FieldPose.standard();
    private final CellMap cells = CellMap.standard();
    private final TagFix tagFix = TagFix.standard();
    private final HeadingLock lock = HeadingLock.standard();
    private final FlywheelSettle settle = FlywheelSettle.standard();
    private final VolleyFire volley = VolleyFire.standard();
    private FieldHeading fieldHeading;
    private final TapHold optionsButton = new TapHold(FIELD_CENTRIC_HOLD_SEC);

    private final ShotSpeed shotSpeed = ShotSpeed.standard();
    private ShotFeeder feeder = ShotFeeder.standard();
    private final AimGate cameraGate = AimGate.standard();     // CAMERA_ONLY mode: exactly BioBuzz TeleOp's gate
    private boolean aimGateOn = AIM_GATE_ENABLED;
    private boolean autoPrespin = true;
    private double launcherPreset = LAUNCHER_NEAR_VELOCITY;
    private String presetName = "NEAR";
    private double launcherTrim = 0;
    private boolean lastAimX = false;
    private boolean burstStarted = false;
    private final PoseFireGate poseGate = PoseFireGate.standard();
    private boolean lastGaveWay = false;
    private final java.util.Map<HiveVision.Cell, TagFix.Result> fixResults = new java.util.EnumMap<>(HiveVision.Cell.class);
    private double camOffX = 0, camOffY = 0;                   // camera position minus pose at the last frame
    private double camAimT = -1e9;
    private Mode lastMode = null;
    private boolean lastVolley = false;

    private final BallCounter ballCounter = BallCounter.forTuning();
    private final GrabTracker grabs = GrabTracker.standard();
    private final ShotResults shotResults = new ShotResults(SHOT_LABEL_WINDOW_SEC);
    private final double[] velT = new double[24], velV = new double[24];
    private int velN = 0;
    private double lastFeedingT = -1e9;
    private boolean lastUnjam = false;
    private boolean readyRumbled = false;
    private boolean endgameWarned = false;

    private BioBuzzDiag diag = new BioBuzzDiag("teleop_pp", false);
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime statusTimer = new ElapsedTime();
    private String status = "";
    private double lastTelemetryT = -1e9, lastPoseLogT = -1e9, lastImuT = -1e9, lastPinOkT = -1e9;
    private static final double PINPOINT_GRACE_SEC = 0.25;
    private int fixes = 0, outliers = 0, snaps = 0;
    private double lastResidual = Double.NaN;

    // ================================================================ INIT

    @Override
    public void init() {
        robot = new BioBuzzRobot(hardwareMap);
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {                           // one hub read per loop (BioBuzzRobot sets AUTO)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        settings = TeleOpSettings.load();
        feeder = ShotFeeder.fromSettings(settings).skipEmptyPockets(SHOT_SKIP_EMPTY_PAUSE, settings.feedEmptyWaitSec);
        autoPose = BioBuzzMatchState.autoPose();
        start = autoPose != null ? Start.AUTO : Start.CAMERA;
        pinpoint = Pinpoint.tryCreate(hardwareMap);
        if (pinpoint != null && autoPose == null) {
            pinpoint.reset();                                   // robot still: the gyro calibrates now
        }
        double fromAuto = BioBuzzMatchState.autoHeading();
        fieldHeading = FieldHeading.standard(Double.isNaN(fromAuto) ? 0 : fromAuto);
        try {
            vision = new HiveVision(hardwareMap);
            vision.setAimTrimDeg(settings.aimTrimDeg);
        } catch (Exception e) {
            vision = null;
            visionError = e.getMessage();
        }
        telemetry.setAutoClear(false);
    }

    @Override
    public void init_loop() {
        clearHubCache();
        if (gamepad1.bWasPressed()) {
            alliance = HiveVision.Alliance.RED;
        } else if (gamepad1.aWasPressed()) {
            alliance = HiveVision.Alliance.BLUE;
        }
        boolean startNext = gamepad1.dpadRightWasPressed(), startPrev = gamepad1.dpadLeftWasPressed();
        if (startNext || startPrev) {                           // each WasPressed() reads (and clears) its edge once
            Start[] all = Start.values();
            int step = startNext ? 1 : all.length - 1;
            do {
                start = all[(start.ordinal() + step) % all.length];
            } while (start == Start.AUTO && autoPose == null);
        }
        if (gamepad1.backWasPressed()) {
            BioBuzzMatchState.diagnostics = !BioBuzzMatchState.diagnostics;
        }
        if (gamepad1.xWasPressed()) {
            aimGateOn = !aimGateOn;
        }
        if (gamepad1.yWasPressed()) {
            autoPrespin = !autoPrespin;
        }
        if (gamepad1.leftBumperWasPressed()) {
            BioBuzzMatchState.fieldCentric = !BioBuzzMatchState.fieldCentric;
        }
        if (gamepad1.optionsWasPressed()) {
            headingSetInInit = true;                            // "intake away from me" beats AUTO's heading
            BioBuzzMatchState.clearAutoHeading();
            fieldHeading.reset();
        }
        if (pinpoint != null) {
            pinpoint.update();
            if (start != Start.AUTO && pinpointCheck.update(pinpoint.ok(), pinpoint.headingDeg(), robot.headingDeg())) {
                pinpoint.reset();                               // drifting gyro (moved while calibrating)
                pinpointRecalibrations++;
            }
        }
        if (matchTimer.seconds() - lastTelemetryT < PpConstants.TELEMETRY_PERIOD_SEC) {
            return;
        }
        lastTelemetryT = matchTimer.seconds();
        telemetry.clear();
        telemetry.addData("Alliance", "%s   (B/Circle = RED, A/Cross = BLUE)  <-- CHECK", alliance);
        telemetry.addData("Start", "%s   (D-pad LEFT/RIGHT)", start.label);
        if (start == Start.CAMERA) {
            telemetry.addData("Heading", headingSetInInit ? "set in INIT: intake AWAY from you"
                    : !Double.isNaN(BioBuzzMatchState.autoHeading()) ? "from AUTO"
                    : "as placed: the INTAKE must point AWAY from you (Options = it does now)");
        }
        telemetry.addData("Pinpoint", pinpoint == null ? "NOT CONFIGURED - camera-only TeleOp"
                : !pinpoint.ok() ? "calibrating: keep the robot STILL"
                : pinpointRecalibrations == 0 ? "ok" : "was drifting: recalibrated " + pinpointRecalibrations + " x");
        telemetry.addData("Auto pre-spin", "%s   (Y)", autoPrespin ? "ON" : "off");
        telemetry.addData("Aim gate", "%s   (X)", aimGateOn ? "ON" : "OFF - balls go when the flywheel is ready");
        telemetry.addData("Drive", "%s   (LB)", BioBuzzMatchState.fieldCentric ? "FIELD-CENTRIC" : "robot-centric");
        telemetry.addData("Camera", vision == null ? "NOT FOUND: " + visionError : "ok");
        telemetry.addData("Diagnostics log", "%s   (Back/Share)", BioBuzzMatchState.diagnostics ? "ON" : "off");
        telemetry.addData("Battery", "%.2f V", robot.battery.getVoltage());
    }

    @Override
    public void start() {
        BioBuzzMatchState.rememberAlliance(alliance);
        settings.alliance = alliance;
        matchTimer.reset();
        lastTelemetryT = -1e9;
        cameraGate.setEnabled(aimGateOn);
        diag = new BioBuzzDiag("teleop_pp", BioBuzzMatchState.diagnostics);
        if (pinpoint != null) {
            pinpoint.update();
            pose.update(now(), pinpoint.xIn(), pinpoint.yIn(), pinpoint.headingDeg(), pinpoint.ok());
        }
        double h;
        switch (start) {
            case AUTO:
                pose.setPose(autoPose[0], autoPose[1], autoPose[2]);
                tagFix.poseFromStart();
                h = autoPose[2];
                break;
            case CELL_WALL:
                pose.setPose(AutoField.START_CELL_WALL.x, AutoField.START_CELL_WALL.y, AutoField.START_CELL_WALL.headingDeg);
                tagFix.poseFromStart();
                h = AutoField.START_CELL_WALL.headingDeg;
                break;
            case ALLIANCE_WALL:
                pose.setPose(AutoField.START_ALLIANCE_WALL.x, AutoField.START_ALLIANCE_WALL.y,
                        AutoField.START_ALLIANCE_WALL.headingDeg);
                tagFix.poseFromStart();
                h = AutoField.START_ALLIANCE_WALL.headingDeg;
                break;
            default:
                double fromAuto = BioBuzzMatchState.autoHeading();
                h = headingSetInInit || Double.isNaN(fromAuto) ? 0 : fromAuto;
                pose.setHeading(h);
        }
        fieldHeading = FieldHeading.standard(h);
        BioBuzzMatchState.clearAutoHeading();
        BioBuzzMatchState.clearAutoPose();
        diag.event("CONFIG", String.format("alliance=%s start=%s start_heading=%.1f pose_known=%s camera=%s pinpoint=%s"
                        + " pinpoint_recal=%d auto_prespin=%s aim_gate=%s field_centric=%s aim_trim=%.1f feed_power=%.2f"
                        + " feed_pause=%.2f spot=%s cam_back=%.1f cell_a=(%.1f,%.1f) cell_b=(%.1f,%.1f) battery=%.2f",
                alliance, start, h, pose.positionKnown() || pose.pending(), vision == null ? "MISSING " + visionError : "ok",
                pinpoint == null ? "MISSING" : pinpoint.status(), pinpointRecalibrations, autoPrespin, aimGateOn,
                BioBuzzMatchState.fieldCentric, settings.aimTrimDeg, settings.feedPower, settings.feedPauseSec,
                settings.hasSpot() ? String.format("%.1fin@%.0f/%s", settings.spotRangeIn, settings.spotSpeed,
                        settings.spotCell) : "none", PpConstants.CAMERA_BACK_IN, PpConstants.CELL_A_X,
                PpConstants.CELL_A_Y, PpConstants.CELL_B_X, PpConstants.CELL_B_Y, robot.battery.getVoltage()));
        robot.stowArm();
    }

    // ================================================================ LOOP

    @Override
    public void loop() {
        clearHubCache();
        double t = matchTimer.seconds();
        double now = now();
        boolean freshFrame = vision != null && vision.update();
        if (pinpoint != null) {
            pinpoint.update();
        }
        boolean pinOk = pinpoint != null && pinpoint.ok();
        double imuNow = Double.NaN;
        if (t - lastImuT >= 0.1 || !pinOk) {                    // backup heading only: 10 Hz (each read ~2-3 ms)
            imuNow = robot.headingDeg();
            lastImuT = t;
        }
        double driverHeading = fieldHeading.update(t, pinOk ? pinpoint.headingDeg() : Double.NaN, imuNow);

        // ---------------- Pose ----------------
        if (pinpoint != null) {
            pose.update(now, pinpoint.xIn(), pinpoint.yIn(), pinpoint.headingDeg(), pinOk);
            if (pose.rawOk()) {
                lastPinOkT = t;
            }
            if (pose.event() != null) {
                camAimT = -1e9;                                 // the camera offset was measured against the old pose
                boolean expired = pose.event().startsWith("START_EXPIRED");
                diag.event(expired ? "START_EXPIRED" : "POSE_JUMP", pose.event());
                setStatus(expired ? "Start position dropped (Pinpoint not ready): camera places the robot"
                        : "Pinpoint jumped: aiming by camera until a CELL is seen");
            }
            if (!pose.headingKnown() && pose.rawOk() && !Double.isNaN(driverHeading)) {
                pose.setHeading(driverHeading);                 // after a Pinpoint restart: the backup heading
            }
        }
        List<HiveVision.Target> seen = vision == null ? java.util.Collections.<HiveVision.Target>emptyList()
                : vision.targets(alliance);
        boolean pinRecent = t - lastPinOkT <= PINPOINT_GRACE_SEC;   // one bad I2C read doesn't drop the pose aim
        fixResults.clear();
        if (freshFrame) {
            for (HiveVision.Target s : seen) {
                fixResults.put(s.cell, applyFix(now, s));
            }
        }
        Mode mode = !pinRecent ? Mode.CAMERA_ONLY : pose.positionKnown() ? Mode.POSE
                : pose.headingKnown() ? Mode.HEADING_ONLY : Mode.CAMERA_ONLY;
        if (mode != lastMode) {
            diag.event("MODE", String.valueOf(mode));
            cameraGate.ballFed();                               // no give-way time left over from another mode
            poseGate.ballFed();
            if (lastMode != null) {
                gamepad1.rumbleBlips(mode == Mode.POSE ? 1 : 2);
            }
            lastMode = mode;
        }

        // ---------------- Target: our UP CELL ----------------
        CellMap.Place place = cells.target();
        HiveVision.Target cam = null;                           // the camera's view of it, if any
        for (HiveVision.Target s : seen) {                      // one UP / DOWN decision: CellMap's
            if (cells.place(s.cell.name()) == place && Boolean.TRUE.equals(cells.up(place))
                    && (cam == null || s.tagCount > cam.tagCount)) {
                cam = s;
            }
        }
        if (cam == null && mode != Mode.POSE && !seen.isEmpty()) {
            cam = vision.target(alliance, (HiveVision.Cell) shotSpeed.cell());   // no pose: BioBuzz TeleOp's choice
        }
        HiveVision.Cell targetCell = cam != null ? cam.cell : cellNamed(cells.nameAt(place, alliance == HiveVision.Alliance.RED));
        TagFix.Result camFix = cam == null ? null : fixResults.get(cam.cell);
        if (cam != null && freshFrame && mode == Mode.POSE && TagFix.usableForAim(camFix)) {   // not rejected frames
            double[] at = pose.fieldAt(frameTime(cam, now));
            if (at != null) {                                   // where this frame puts the robot, vs the pose then
                double[] fv = TagFix.floorVector(cam.xIn, cam.yIn, cam.zIn, CAMERA_TILT_DEG);
                double[] m = TagFix.robotXY(at[2], fv[0], fv[1], cells.x(place), cells.y(place),
                        PpConstants.CAMERA_BACK_IN, PpConstants.CAMERA_LEFT_IN);
                camOffX = m[0] - at[0];
                camOffY = m[1] - at[1];
                camAimT = t;
            }
        }
        // Aim from the pose; for a moment after a frame, from the camera's position carried along by the odometry
        // (the offset follows every move made since the frame, so a sideways step still changes the aim)
        boolean camAimFresh = t - camAimT <= TARGET_MAX_AGE_SEC;
        ShotSolver.Aim aim = null;
        if (mode == Mode.POSE) {
            double cellUp = (cam != null ? cam.heightIn() : PpConstants.CELL_OPENING_HEIGHT_IN) - CAMERA_LENS_HEIGHT_IN;
            aim = ShotSolver.solve(pose.x() + (camAimFresh ? camOffX : 0), pose.y() + (camAimFresh ? camOffY : 0),
                    pose.headingDeg(), cells.x(place), cells.y(place), PpConstants.CAMERA_BACK_IN,
                    PpConstants.CAMERA_LEFT_IN, settings.aimTrimDeg, CAMERA_TILT_DEG, cellUp);
        }
        double camDist = cam == null ? Double.NaN : cam.horizontalIn();
        double distance = !Double.isNaN(camDist) ? camDist : aim != null ? aim.distanceIn : Double.NaN;
        String distSource = !Double.isNaN(camDist) ? "camera" : aim != null ? "pose" : "none";
        double aimError = aim == null ? Double.NaN : aim.errorDeg;
        // No pose aim (CAMERA_ONLY, or HEADING_ONLY before the first fix / after a jump): BioBuzz TeleOp's camera aim
        // and gate. With no CELL in view either, the gate leaves it to the driver, exactly as there.
        boolean useCameraAim = aim == null;
        double turnRate = pinRecent ? pose.turnRateDps() : 0;

        // ---------------- Buttons: speed presets, trim, labels ----------------
        if (gamepad1.yWasPressed()) {
            autoPrespin = !autoPrespin;
            diag.event("PRESPIN", autoPrespin ? "auto ON" : "auto OFF");
        }
        if (gamepad1.left_bumper) {
            ShotResults.Result r = gamepad1.dpadUpWasPressed() ? ShotResults.Result.IN
                    : gamepad1.dpadDownWasPressed() ? ShotResults.Result.SHORT
                    : gamepad1.dpadRightWasPressed() ? ShotResults.Result.LONG
                    : gamepad1.dpadLeftWasPressed() ? ShotResults.Result.SIDE : null;
            if (r != null) {
                labelShot(r);
            }
        } else if (gamepad1.dpadUpWasPressed()) {
            setPreset(LAUNCHER_FAR_VELOCITY, "FAR");
        } else if (gamepad1.dpadDownWasPressed()) {
            setPreset(LAUNCHER_NEAR_VELOCITY, "NEAR");
        } else if (gamepad1.dpadRightWasPressed()) {
            launcherTrim += LAUNCHER_TRIM_STEP;
            diag.event("TRIM", String.format("%+.0f", launcherTrim));
        } else if (gamepad1.dpadLeftWasPressed()) {
            launcherTrim -= LAUNCHER_TRIM_STEP;
            diag.event("TRIM", String.format("%+.0f", launcherTrim));
        }
        boolean aimX = gamepad1.x;
        if (aimX && !lastAimX && shotSpeed.manual()) {          // X = the camera / pose speed again
            shotSpeed.setManual(false);
            launcherTrim = 0;
            diag.event("PRESET", "AUTO");
        }
        lastAimX = aimX;

        // ---------------- Volley + shot speed ----------------
        boolean firePressed = gamepad1.rightBumperWasPressed();
        boolean volleyOn = volley.active();
        boolean aiming = volleyOn || aimX;
        double baseSpeed = shotSpeed.update(t, Double.isNaN(distance) ? null : targetCell, distance,
                volleyOn && (burstStarted || Double.isNaN(distance)), launcherPreset);
        // The speed follows one CELL and switches only after AUTO_SPEED_SWITCH_SEC: while it still follows another
        // CELL than the aim, no ball goes, and a speed latched for the old CELL is released (it qualifies again).
        boolean speedAuto = shotSpeed.source() == ShotSpeed.Source.AUTO || shotSpeed.source() == ShotSpeed.Source.AUTO_KEPT;
        CellMap.Place speedPlace = shotSpeed.cell() instanceof HiveVision.Cell
                ? cells.place(((HiveVision.Cell) shotSpeed.cell()).name()) : null;
        CellMap.Place aimPlace = targetCell == null ? null : cells.place(targetCell.name());
        boolean speedMismatch = speedAuto && speedPlace != null && aimPlace != null && speedPlace != aimPlace;
        if (speedMismatch && burstStarted) {
            burstStarted = false;
            shotSpeed.setManual(false);                         // only releases the latch (speedAuto: not manual)
            diag.event("SPEED_RELATCH", "aim moved to " + aimPlace + ", speed still on " + speedPlace);
        }
        double launcherTarget = Range.clip(baseSpeed + launcherTrim, 0, LAUNCHER_MAX_VELOCITY);
        boolean nearCell = cam != null || (aim != null && aim.distanceIn <= PpConstants.PRESPIN_MAX_DIST_IN);
        boolean spin = aiming || (autoPrespin && nearCell);
        robot.setLauncherVelocity(spin ? launcherTarget : 0);
        boolean ready = robot.updateLauncherReady();
        double flywheel = robot.launcherVelocity();
        boolean settled = settle.update(t, launcherTarget, flywheel);
        if (!robot.isLauncherOn()) {
            readyRumbled = false;
        } else if (ready && !readyRumbled) {
            readyRumbled = true;
            gamepad1.rumble(200);
            diag.event("LAUNCHER_READY", String.format("%.0f t/s", flywheel));
        }

        // ---------------- Flower arm ----------------
        boolean sticksReleased = Math.abs(gamepad1.left_stick_x) < ARM_DEPLOY_MAX_STICK
                && Math.abs(gamepad1.left_stick_y) < ARM_DEPLOY_MAX_STICK
                && Math.abs(gamepad1.right_stick_x) < ARM_DEPLOY_MAX_STICK;
        if (gamepad1.aWasPressed()) {
            if (!BioBuzzRobot.armCalibrated()) {
                setStatus("Flower arm not calibrated - run BioBuzz Calibration");
            } else if (robot.armState() == BioBuzzRobot.ArmState.DEPLOYED) {
                robot.stowArm();
                diag.event("ARM", "STOW");
            } else if (sticksReleased) {
                robot.deployArm();
                diag.event("ARM", "DEPLOY");
            } else {
                setStatus("Stop the robot before deploying the arm");
                gamepad1.rumbleBlips(2);
            }
        }
        boolean armDeployed = robot.armState() == BioBuzzRobot.ArmState.DEPLOYED;

        // ---------------- Drive + aim ----------------
        double scale = gamepad1.left_bumper ? SLOW_DRIVE_SCALE : 1.0;
        if (armDeployed) {
            scale = Math.min(scale, ARM_DEPLOYED_DRIVE_SCALE);
        }
        double forward = -gamepad1.left_stick_y * scale;
        double strafe = gamepad1.left_stick_x * scale;
        double turn = gamepad1.right_stick_x * scale;
        TapHold.Event options = optionsButton.update(t, gamepad1.optionsWasPressed(), gamepad1.options);
        if (options == TapHold.Event.HOLD) {
            BioBuzzMatchState.fieldCentric = !BioBuzzMatchState.fieldCentric;
            gamepad1.rumbleBlips(BioBuzzMatchState.fieldCentric ? 1 : 2);
            diag.event("DRIVE_MODE", BioBuzzMatchState.fieldCentric ? "field-centric" : "robot-centric");
        } else if (options == TapHold.Event.TAP) {
            fieldHeading.reset();
            driverHeading = 0;
            pose.setHeading(0);
            gamepad1.rumble(200);
            setStatus("Heading set: the intake points AWAY from you now");
            diag.event("HEADING_SET", "driver: intake away (Options)");
        }
        double driveHeading = pose.headingKnown() && pose.rawOk() ? pose.headingDeg() : driverHeading;
        if (BioBuzzMatchState.fieldCentric && !Double.isNaN(driveHeading) && !aiming) {
            double[] r = MecanumMath.driverToRobot(forward, strafe, driveHeading);
            forward = r[0];
            strafe = r[1];
        }
        boolean driverTurns = Math.abs(gamepad1.right_stick_x) > PpConstants.STICK_OVERRIDE;
        boolean spotHold = false;
        if (aiming && !driverTurns) {
            if (useCameraAim) {
                if (cam != null) {
                    turn = HiveVision.aimTurnCw(cam.bearingDeg);
                }
            } else {
                turn = lock.turnCw(aimError, turnRate);
            }
        }
        if (aiming && settings.hasSpot() && settings.spotIsFor(targetCell) && !burstStarted && !poseGate.spotGaveWay()
                && !armDeployed && !Double.isNaN(distance) && Math.abs(gamepad1.left_stick_y) < 0.1
                && (useCameraAim ? cam != null : Math.abs(aimError) < 10)) {
            double toSpot = MotionController.spotForward(distance, settings.spotRangeIn);
            forward = toSpot;
            spotHold = toSpot != 0;
        }
        robot.lastDriveHeadingDeg = driveHeading;
        robot.lastFieldDrive = BioBuzzMatchState.fieldCentric && !aiming;
        robot.drive(forward, strafe, turn, 1.0);

        // ---------------- Spot save ----------------
        if (gamepad1.backWasPressed()) {
            if (cam == null) {
                setStatus("Can't save the spot: our UP CELL isn't in view");
                gamepad1.rumbleBlips(2);
            } else {
                settings.alliance = alliance;
                settings.spotRangeIn = camDist;
                settings.spotSpeed = launcherTarget;
                settings.spotCell = cam.cell;
                boolean saved = settings.trySave();
                setStatus(saved ? String.format("SHOT SPOT saved: %.0f in to %s @ %.0f t/s", camDist, cam.cell,
                        launcherTarget) : "SHOT SPOT save FAILED");
                diag.event("SPOT_SAVED", String.format("floor %.1f cell %s speed %.0f saved=%s", camDist, cam.cell,
                        launcherTarget, saved));
                gamepad1.rumble(500);
            }
        }

        // ---------------- Fire gate + windmill ----------------
        boolean aimOk;
        String holdWhy = null;
        if (!aimGateOn) {
            aimOk = true;
        } else if (useCameraAim) {
            aimOk = cameraGate.update(t, volleyOn, ready, cam != null, cam != null ? cam.bearingDeg : 0, turn, spotHold);
            if (!aimOk) {
                holdWhy = "camera " + cameraGate.state();
            }
        } else {
            boolean spotBefore = poseGate.spotGaveWay();
            aimOk = poseGate.update(t, volleyOn, ready, aimError, turnRate, cam != null ? cam.bearingDeg : Double.NaN,
                    spotHold, driverTurns);
            switch (poseGate.hold()) {
                case SPOT:
                    holdWhy = "driving to the SHOT SPOT";
                    break;
                case DRIVER_TURNING:
                    holdWhy = "driver turning";
                    break;
                case TURNING:
                    holdWhy = "turning";
                    break;
                case AIM:
                    holdWhy = String.format("aim %.1f deg off", aimError);
                    break;
                case CAMERA:
                    holdWhy = String.format("camera says %.1f deg off (pose %.1f)", cam.bearingDeg, aimError);
                    break;
                default:
                    break;
            }
            if (poseGate.spotGaveWay() && !spotBefore) {
                diag.event("SPOT_GIVEWAY", String.format("%.1f in from the spot", distance - settings.spotRangeIn));
            }
            if (poseGate.gaveWay() && !lastGaveWay) {           // once per ball, not every loop until it moves
                diag.event("AIM_GIVEWAY", holdWhy + " hold=" + poseGate.hold());
            }
            lastGaveWay = poseGate.gaveWay();
        }
        int indexerTicks = robot.indexerTicks();
        if (speedMismatch && holdWhy == null) {
            holdWhy = "shot speed switching to the new CELL";
        }
        double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
        if (gamepad1.b) {
            feeder.reset();
            robot.setIndexerPower(-INDEXER_UNJAM_POWER);
        } else {
            robot.setIndexerPower(feeder.update(t, indexerTicks, ready, aimOk && settled && !speedMismatch, volleyOn));
            String feedEvent = feeder.event();
            if (feedEvent != null) {
                diag.event("FEED", feedEvent + String.format(" | launcher %.0f t/s target %.0f settled %s", flywheel,
                        launcherTarget, settled));
                if (feedEvent.startsWith("SHOT")) {
                    poseGate.ballFed();
                    cameraGate.ballFed();                       // each ball earns its own give-way time
                    burstStarted = volleyOn;
                }
                if (feedEvent.startsWith("JAM")) {
                    setStatus("Windmill " + feedEvent + " - hold B to unjam");
                    gamepad1.rumbleBlips(1);
                }
            }
        }
        boolean feeding = feeder.feeding();
        if (volleyOn && aimOk) {
            intakePower = Math.max(intakePower, 0) + FEED_INTAKE_BOOST;
        }
        robot.setIntake(intakePower);
        if (gamepad1.b != lastUnjam) {
            diag.event(gamepad1.b ? "UNJAM_PRESS" : "UNJAM_RELEASE", "indexer ticks " + indexerTicks);
            lastUnjam = gamepad1.b;
        }

        // ---------------- Launched balls ----------------
        velT[velN % velT.length] = t;
        velV[velN % velV.length] = flywheel;
        velN++;
        if (feeding) {
            lastFeedingT = t;
        }
        boolean watchGrab = robot.isLauncherOn() && (feeding || grabs.active() || t - lastFeedingT <= GRAB_WATCH_SEC);
        double amps = watchGrab ? robot.launcherCurrentAmps() : Double.NaN;
        double flywheelTarget = robot.isLauncherOn() ? robot.launcherTarget() : 0;
        grabs.sample(t, flywheel, flywheelTarget, indexerTicks, feeder.targetTicks(), feeding, amps);
        boolean ball = ballCounter.update(t, feeding, flywheelTarget, flywheel);
        if (ball) {
            logBall(t, distance, distSource, launcherTarget, baseSpeed, aimError, turnRate, mode, cam);
        }
        for (GrabTracker.Grab g = grabs.poll(); g != null; g = grabs.poll()) {
            diag.event("GRAB", String.format("ball=%d pre=%.0f target=%.0f launch_err=%+.0f depth=%.0f recovery_s=%.3f"
                            + " drops=%d overlapped=%s", g.ball, g.preTps, g.targetTps, g.launchErrorTps(), g.depthTps(),
                    g.recoverySec, g.drops, g.overlapped));
        }

        // ---------------- Volley state ----------------
        volley.update(t, firePressed, gamepad1.b, feeder.shots(), ball, feeder.state() == ShotFeeder.State.JAMMED);
        if (volley.event() != null) {
            diag.event(volley.active() ? "VOLLEY_START" : "VOLLEY_STOP", volley.event()
                    + String.format(" mode=%s dist=%.1f(%s) aim_err=%.1f", mode, distance, distSource, aimError));
        }
        if (volley.active() != lastVolley) {
            if (volley.active()) {
                gamepad1.rumble(150);
            } else {
                burstStarted = false;
                poseGate.reset();
                cameraGate.ballFed();
                if (volley.lastStop() == VolleyFire.Stop.EMPTY) {
                    gamepad1.rumbleBlips(2);
                }
            }
            lastVolley = volley.active();
        }

        // ---------------- Logs, reminders, telemetry ----------------
        if (t - lastPoseLogT >= PpConstants.POSE_LOG_PERIOD_SEC) {
            lastPoseLogT = t;
            diag.event("POSE", String.format("mode=%s x=%.1f y=%.1f hdg=%.1f target=%s dist=%.1f(%s) aim_err=%.1f"
                            + " fixes=%d outliers=%d snaps=%d last_residual=%.1f", mode,
                    pose.positionKnown() ? pose.x() : Double.NaN, pose.positionKnown() ? pose.y() : Double.NaN,
                    pose.headingKnown() ? pose.headingDeg() : Double.NaN, place, distance, distSource, aimError, fixes,
                    outliers, snaps, lastResidual));
        }
        if (!endgameWarned && t >= 100) {
            endgameWarned = true;
            gamepad1.rumbleBlips(3);
            setStatus("20 s left - PARK in the LOADING ZONE");
            diag.event("ENDGAME_WARNING", "20 s left");
        }
        diag.sample(robot, gamepad1, cam, feeding ? "FEEDING" : (gamepad1.b ? "UNJAM" : volley.active() ? "VOLLEY" : ""));
        if (t - lastTelemetryT >= PpConstants.TELEMETRY_PERIOD_SEC) {
            lastTelemetryT = t;
            showTelemetry(mode, place, distance, distSource, baseSpeed, launcherTarget, flywheel, ready, settled, aimError,
                    holdWhy, cam);
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

    // ================================================================ helpers

    private TagFix.Result applyFix(double now, HiveVision.Target s) {
        double[] fv = TagFix.floorVector(s.xIn, s.yIn, s.zIn, CAMERA_TILT_DEG);
        String before = cells.target().toString();
        TagFix.Result r = tagFix.apply(now, pose, cells, s.cell.name(), s.tagCount, fv[0], fv[1], s.heightIn(),
                frameTime(s, now));
        if (!before.equals(cells.target().toString())) {
            diag.event("TARGET", "UP CELL now at " + cells.target() + " (" + s.cell + ")");
        }
        switch (r) {
            case CORRECTED:
                fixes++;
                lastResidual = tagFix.residualIn();
                break;
            case OUTLIER:
                outliers++;
                lastResidual = tagFix.residualIn();
                break;
            case FIRST_FIX:
            case SNAPPED:
                snaps++;
                camAimT = -1e9;                                 // the pose moved: an earlier camera offset is stale
                gamepad1.rumble(100);
                break;
            default:
                break;
        }
        if (tagFix.event() != null) {
            diag.event("POSE_FIX", tagFix.event());
        }
        return r;
    }

    private static double frameTime(HiveVision.Target s, double now) {
        return s.frameNanos > 0 ? s.frameNanos * 1e-9 : now - PpConstants.FIX_DEFAULT_LATENCY_SEC;
    }

    private static double now() {
        return System.nanoTime() * 1e-9;                         // the camera's frame times use this clock
    }

    private void clearHubCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    private static HiveVision.Cell cellNamed(String name) {
        try {
            return name == null ? null : HiveVision.Cell.valueOf(name);
        } catch (IllegalArgumentException e) {
            return null;
        }
    }

    private void setPreset(double speed, String name) {
        launcherPreset = speed;
        presetName = name;
        launcherTrim = 0;
        shotSpeed.setManual(true);
        diag.event("PRESET", name + " " + speed + " (manual)");
    }

    private void logBall(double t, double distance, String distSource, double target, double base, double aimError,
                         double turnRate, Mode mode, HiveVision.Target cam) {
        double table = Double.isNaN(distance) ? Double.NaN : shotSpeed.tableSpeed(distance);
        double launchVel = Double.NEGATIVE_INFINITY, lowVel = Double.POSITIVE_INFINITY;
        for (int i = 0; i < Math.min(velN, velT.length); i++) {
            if (t - velT[i] <= 0.35) {
                launchVel = Math.max(launchVel, velV[i]);
                lowVel = Math.min(lowVel, velV[i]);
            }
        }
        ShotResults.Ball b = shotResults.launched(t, distance, target, launcherTrim, table);
        grabs.ballCounted(t, b.number);
        diag.event("BALL", String.format("ball=%d volley_ball=%d mode=%s floor_in=%.1f dist_source=%s cell=%s tags=%d"
                        + " bearing=%.1f source=%s base=%.0f trim=%+.0f target=%.0f table=%.0f vel=%.0f dip=%.0f aim_err=%.2f"
                        + " turn_dps=%.1f pose_x=%.1f pose_y=%.1f pose_hdg=%.1f battery=%.2f", b.number, volley.balls() + 1,
                mode, distance, distSource, cam == null ? "none" : cam.cell, cam == null ? 0 : cam.tagCount,
                cam == null ? Double.NaN : cam.bearingDeg, shotSpeed.source(), base, launcherTrim, target, table,
                launchVel, launchVel - lowVel, aimError, turnRate, pose.positionKnown() ? pose.x() : Double.NaN,
                pose.positionKnown() ? pose.y() : Double.NaN, pose.headingKnown() ? pose.headingDeg() : Double.NaN,
                robot.battery.getVoltage()));
    }

    private void labelShot(ShotResults.Result r) {
        double t = matchTimer.seconds();
        ShotResults.Ball b = shotResults.label(t, r);
        if (b == null) {
            setStatus(r + " not saved: no recent ball waits for a result");
            gamepad1.rumbleBlips(2);
            return;
        }
        gamepad1.rumble(150);
        setStatus(String.format("Ball %d: %s  (%.0f in, %.0f t/s)", b.number, r, b.floorIn, b.targetTps));
        diag.event("SHOT_RESULT", String.format("ball=%d result=%s floor_in=%.1f target=%.0f trim=%+.0f table=%.0f"
                        + " offset=%+.0f age_s=%.1f", b.number, r, b.floorIn, b.targetTps, b.trimTps, b.tableTps,
                b.targetTps - b.tableTps, t - b.t));
    }

    private void showTelemetry(Mode mode, CellMap.Place place, double distance, String distSource, double base,
                               double target, double flywheel, boolean ready, boolean settled, double aimError,
                               String holdWhy, HiveVision.Target cam) {
        telemetry.clear();
        String modeText = mode == Mode.POSE ? "PINPOINT + tags"
                : mode == Mode.HEADING_ONLY ? "camera aim (position not known yet - point the camera at our CELL)"
                : "CAMERA ONLY (no Pinpoint)";
        telemetry.addData("Aim", "%s   %s", modeText, volley.active() ? "** VOLLEY **" : "RB = volley");
        if (pose.positionKnown()) {
            telemetry.addData("Robot", "x %.0f  y %.0f  hdg %.0f   fixes %d, last %.1f in off", pose.x(), pose.y(),
                    pose.headingDeg(), fixes + snaps, lastResidual);
        }
        telemetry.addData("Target", "CELL %s (%s)   %s", place, cells.up(place) == null ? "UP?"
                : cells.up(place) ? "UP" : "DOWN", Double.isNaN(distance) ? "no distance"
                : String.format("%.0f in (%s)", distance, distSource));
        if (!Double.isNaN(aimError)) {
            telemetry.addData("Aim error", "%.1f deg%s", aimError, cam != null ? "  (camera sees it)" : "");
        }
        String speedName = shotSpeed.source() == ShotSpeed.Source.MANUAL ? "MANUAL " + presetName
                : shotSpeed.source() == ShotSpeed.Source.PRESET ? presetName : "AUTO";
        telemetry.addData("Shot speed", "%s %.0f %+.0f = %.0f t/s   (D-pad: UP far, DOWN near, RIGHT/LEFT trim)",
                speedName, base, launcherTrim, target);
        telemetry.addData("Launcher", "%s  actual %.0f t/s%s   pre-spin %s", !robot.isLauncherOn() ? "OFF"
                : ready ? (settled ? "READY" : "READY, settling") : "spinning up", flywheel,
                autoPrespin ? "" : " (Y: off)", autoPrespin ? "auto" : "off");
        if (holdWhy != null && volley.active()) {
            telemetry.addData("BALL HELD", "%s  (goes anyway after %.1f s)", holdWhy, PpConstants.FIRE_GIVEWAY_SEC);
        }
        telemetry.addData("Windmill", gamepad1.b ? "UNJAM" : String.format("%s  pockets %d%s  volley balls %d",
                feeder.state(), feeder.shots(), feeder.jams() > 0 ? "  jams " + feeder.jams() : "", volley.balls()));
        int waiting = shotResults.waiting(matchTimer.seconds());
        telemetry.addData("Balls", "%d out, results %d (IN %d)%s", shotResults.balls(), shotResults.labelled(),
                shotResults.count(ShotResults.Result.IN), waiting > 0 ? "  " + waiting + " waiting: LB + D-pad" : "");
        telemetry.addData("Drive", "%s%s", BioBuzzMatchState.fieldCentric ? "FIELD-CENTRIC" : "robot-centric",
                robot.armState() == BioBuzzRobot.ArmState.DEPLOYED ? ", ARM OUT - precision" : "");
        if (statusTimer.seconds() < 3 && !status.isEmpty()) {
            telemetry.addData("Note", status);
        }
        String warning = robot.launcherWarning();
        if (warning != null) {
            telemetry.addData("WARNING", warning);
        }
        telemetry.addData("Battery", "%.2f V", robot.battery.getVoltage());
        if (diag.enabled()) {
            telemetry.addData("Diag log", diag.status());
        }
    }

    private void setStatus(String s) {
        status = s;
        statusTimer.reset();
    }
}

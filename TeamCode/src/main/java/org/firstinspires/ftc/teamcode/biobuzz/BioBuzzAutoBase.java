package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * BIOBUZZ AUTO (TU06): LEAVE (3) + launch 4 preloads into our upward CELL (HIVE TIP = 20 if it tips)
 * + AUTO PARK in our LOADING ZONE (5).  LEAVE + PARK from both robots reaches the 16-point SWARM threshold.
 *
 * START (same for RED and BLUE - the field is point-symmetric):
 *   - intake side touching our ALLIANCE wall, launcher pointing into the FIELD,
 *   - robot completely outside the LOADING ZONE, next to it, about 1 in from its tape
 *     (RED: audience side of the zone, BLUE: rear side of the zone),
 *   - not touching a FLOWER, 4 POLLEN in the windmill, flower arm stowed.
 * ROUTE (AutoRoute): back 12 in (LEAVE) -> turn to aim -> camera fine-aim -> fire -> face the wall ->
 *        forward to 4 in off the wall -> strafe right into the LOADING ZONE.
 *
 * INIT options on gamepad 1: D-pad up/down = start delay, X = shooting on/off, Y = camera on/off (drill only).
 * Every run is logged to /sdcard/BioBuzzLogs/auto_*.csv
 */
public abstract class BioBuzzAutoBase extends LinearOpMode implements DriveIO, AutoRoute.Mechanisms {

    protected abstract HiveVision.Alliance alliance();

    private BioBuzzRobot robot;
    private HiveVision vision;
    private CsvLog log;
    private HiveVision.Target lastTarget;
    private final TeleOpSettings spot = TeleOpSettings.load();       // SHOT SPOT saved in TeleOp (Share)
    private final ElapsedTime clock = new ElapsedTime();
    private double matchStart = 0;
    private String step = "INIT";
    private Pinpoint pinpoint;
    private final OdometryAccumulator odo = new OdometryAccumulator();
    private double lastWheelF, lastWheelS, pinpointHeading0, imuHeading0;
    private boolean pinpointWasOk = false;
    private boolean pinpointDisabled = false;

    @Override
    public void runOpMode() {
        robot = new BioBuzzRobot(hardwareMap);
        String visionError = null;
        try {
            vision = new HiveVision(hardwareMap);
            vision.setAimTrimDeg(spot.aimTrimDeg);
        } catch (Exception e) {
            vision = null;
            visionError = e.getMessage();
        }
        BioBuzzMatchState.rememberAlliance(alliance());
        pinpoint = Pinpoint.tryCreate(hardwareMap);
        if (pinpoint != null) {
            pinpoint.reset();                                   // robot sits still during INIT: IMU calibrates here
        }

        int startDelaySec = 0;
        boolean shoot = true;
        boolean useCamera = true;
        while (opModeInInit()) {
            if (gamepad1.dpadUpWasPressed()) {
                startDelaySec = Math.min(AUTO_MAX_START_DELAY_SEC, startDelaySec + 1);
            } else if (gamepad1.dpadDownWasPressed()) {
                startDelaySec = Math.max(0, startDelaySec - 1);
            }
            if (gamepad1.xWasPressed()) {
                shoot = !shoot;
            }
            if (gamepad1.yWasPressed()) {
                useCamera = !useCamera;
            }
            if (vision != null) {
                vision.update();
            }
            telemetry.addData("Alliance", alliance());
            telemetry.addData("Plan", shoot ? "LEAVE + SHOOT 4 + PARK" : "LEAVE + PARK only");
            telemetry.addData("Start delay", "%d s   (D-pad up/down)", startDelaySec);
            telemetry.addData("Shooting", "%s   (X toggles)", shoot ? "ON" : "OFF");
            telemetry.addData("Camera aim", "%s   (Y toggles - keep ON for matches)", useCamera ? "ON" : "OFF (drill)");
            telemetry.addData("Heading", "%.1f deg", robot.headingDeg());
            if (pinpoint == null) {
                telemetry.addData("Odometry", "wheel encoders (no \"pinpoint\" in the configuration)");
            } else {
                pinpoint.update();
                telemetry.addData("Odometry", "Pinpoint %s  x %.1f  y %.1f in  %.1f deg%s", pinpoint.status(),
                        pinpoint.xIn(), pinpoint.yIn(), pinpoint.headingDeg(),
                        pinpoint.ok() ? "" : "  -> will use wheel encoders");
            }
            if (vision == null) {
                telemetry.addData("Camera", "NOT FOUND: %s (IMU aim only)", visionError);
            } else {
                HiveVision.Target t = vision.target(alliance(), HiveVision.startingUpCell(alliance()));
                telemetry.addData("Camera", t == null ? "no CELL tags in view (normal at the wall)"
                        : String.format("%s %d tags, bearing %.1f", t.cell, t.tagCount, t.bearingDeg));
            }
            telemetry.addData("Battery", "%.2f V", robot.battery.getVoltage());
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) {
            closeVision();
            return;
        }
        if (!useCamera) {
            closeVision();
        }

        log = new CsvLog("auto_" + alliance(), "t", "step", "fwd_in", "right_in", "heading_deg",
                "launcher_tps", "launcher_target", "launcher_ready", "indexer_ticks", "battery_v",
                "cell", "tags", "bearing_deg", "odometry", "range_in");
        log.row("CONFIG", "delay=" + startDelaySec, "shoot=" + shoot, "camera=" + (vision != null),
                "aim_heading=" + AUTO_AIM_HEADING_DEG, "launch_tps=" + launchTps(),
                spot.hasSpot() ? String.format("spot=%.1f in @ %.0f", spot.spotRangeIn, spot.spotSpeed) : "spot=none",
                spot.hasShootPose() ? String.format("pose=fwd %.1f left %.1f hdg %.1f", spot.shootFwdIn, spot.shootLeftIn,
                        spot.shootHeadingDeg) : "pose=none", String.format("aim_trim=%.1f", spot.aimTrimDeg));
        matchStart = clock.seconds();
        startOdometry();
        try {
            AutoRoute.run(this, this, startDelaySec, shoot);
            loopTick();
            log.row("RESULT", "done_s=" + (clock.seconds() - matchStart), "fwd_in=" + forwardInches(),
                    "right_in=" + strafeRightInches(), "heading=" + headingDeg(),
                    "odometry=" + (pinpoint == null ? "wheels" : "pinpoint " + pinpoint.status()),
                    "wheels_fwd_in=" + robot.forwardInches(), "wheels_right_in=" + robot.strafeRightInches());
        } finally {
            robot.stopAll();
            closeVision();
            log.close();
        }
    }

    // ---------------- DriveIO ----------------

    @Override
    public boolean active() {
        return opModeIsActive();
    }

    @Override
    public double seconds() {
        return clock.seconds();
    }

    @Override
    public double forwardInches() {
        return odo.forwardInches();
    }

    @Override
    public double strafeRightInches() {
        return odo.strafeRightInches();
    }

    /** Control Hub IMU, zeroed at START (no resetYaw: it applied late and gave -30 deg on 31 Oct). */
    @Override
    public double headingDeg() {
        return MecanumMath.normalizeDegrees(robot.headingDeg() - imuHeading0);
    }

    @Override
    public void drive(double forward, double strafeRight, double turnCw) {
        robot.drive(forward, strafeRight, turnCw, 1.0);
    }

    /** Called right after START: zero the robot-relative odometry on the current readings. */
    private void startOdometry() {
        imuHeading0 = robot.headingDeg();
        lastWheelF = robot.forwardInches();
        lastWheelS = robot.strafeRightInches();
        if (pinpoint != null) {
            pinpoint.update();
            pinpointHeading0 = pinpoint.headingDeg();
        }
        updateOdometry();
    }

    /** Pinpoint when it reports READY, otherwise the wheel encoders - switches per loop, never loses the distance. */
    private void updateOdometry() {
        double wf = robot.forwardInches(), ws = robot.strafeRightInches();
        if (pinpoint != null) {
            pinpoint.update();
        }
        boolean ok = pinpoint != null && !pinpointDisabled && pinpoint.ok();
        if (ok) {
            double drift = MecanumMath.normalizeDegrees(pinpoint.headingDeg() - pinpointHeading0 - headingDeg());
            if (Math.abs(drift) > PINPOINT_MAX_HEADING_ERROR_DEG) {    // bad Pinpoint gyro (moved during INIT): stop trusting it
                pinpointDisabled = true;
                ok = false;
                log.row(clock.seconds() - matchStart, "EVENT",
                        String.format("Pinpoint heading off by %.0f deg vs IMU: wheel encoders", drift));
            }
        }
        if (ok) {
            odo.fieldUpdate(pinpoint.xIn(), pinpoint.yIn(), pinpoint.headingDeg());
        } else {
            odo.robotStep(wf - lastWheelF, ws - lastWheelS);
        }
        if (pinpointWasOk && !ok) {
            log.row(clock.seconds() - matchStart, "EVENT", "Pinpoint lost (" + pinpoint.status() + "): wheel encoders");
        }
        pinpointWasOk = ok;
        lastWheelF = wf;
        lastWheelS = ws;
    }

    @Override
    public void loopTick() {
        updateOdometry();
        double t = clock.seconds() - matchStart;
        double heading = headingDeg();
        double fwd = forwardInches();
        double right = strafeRightInches();
        double launcherVel = robot.launcherVelocity();
        HiveVision.Target target = lastTarget;
        log.row(t, step, fwd, right, heading, launcherVel, robot.launcherTarget(), robot.launcherReady(),
                robot.indexerTicks(), robot.battery.getVoltage(), target == null ? "" : target.cell,
                target == null ? 0 : target.tagCount, target == null ? "" : target.bearingDeg,
                pinpointWasOk ? "pinpoint" : "wheels", target == null ? "" : target.rangeIn);
        lastTarget = null;

        telemetry.addData("Step", step);
        telemetry.addData("Time", "%.1f s", t);
        telemetry.addData("Heading", "%.1f deg", heading);
        telemetry.addData("Moved", "fwd %.1f  right %.1f in  (%s)", fwd, right, pinpointWasOk ? "Pinpoint" : "wheels");
        telemetry.addData("Launcher", "%.0f / %.0f t/s", launcherVel, robot.launcherTarget());
        telemetry.addData("Log", log.status());
        telemetry.update();
    }

    // ---------------- AutoRoute.Mechanisms ----------------

    @Override
    public void step(String name) {
        step = name;
    }

    @Override
    public void startLauncher() {
        robot.setLauncherVelocity(launchTps());
    }

    /** The TeleOp SHOT SPOT speed when one is saved, otherwise AUTO_LAUNCH_VELOCITY. */
    private double launchTps() {
        return spot.hasSpot() ? spot.spotSpeed : AUTO_LAUNCH_VELOCITY;
    }

    @Override
    public double[] shootPose() {
        return spot.hasShootPose() ? new double[]{spot.shootFwdIn, -spot.shootLeftIn, spot.shootHeadingDeg} : null;
    }

    @Override
    public void stopLauncher() {
        robot.setLauncherVelocity(0);
    }

    @Override
    public void aimAndFire(double matchDeadlineSec) {
        double f0 = forwardInches();
        aimWithCamera(Math.min(clock.seconds() + AUTO_AIM_TIMEOUT_SEC, matchDeadlineSec));
        double spotMoved = forwardInches() - f0;
        fire(matchDeadlineSec);
        if (Math.abs(spotMoved) > AUTO_DRIVE_TOLERANCE_IN) {    // undo the SHOT SPOT move so RETURN / PARK stay on their path
            step = "SPOT BACK";
            new MotionController(this).driveRelative(-spotMoved, 0, 2.0);
        }
    }

    /** Camera: turn onto our CELL and, with a saved SHOT SPOT, drive forward/back to its distance (max AUTO_SPOT_MAX_IN). */
    private void aimWithCamera(double endSec) {
        if (vision == null) {
            return;
        }
        HiveVision.Cell preferred = HiveVision.startingUpCell(alliance());
        double f0 = forwardInches();
        int settled = 0;
        while (opModeIsActive() && clock.seconds() < endSec) {
            vision.update();
            HiveVision.Target target = vision.target(alliance(), preferred);
            lastTarget = target;
            double turn = target == null ? 0 : HiveVision.aimTurnCw(target.bearingDeg);
            double fwd = 0;
            if (target != null && spot.hasSpot() && !spot.hasShootPose()) {   // taught position: already there
                fwd = MotionController.spotForward(target.horizontalIn(), spot.spotRangeIn);
                double moved = forwardInches() - f0;
                if ((fwd > 0 && moved >= AUTO_SPOT_MAX_IN) || (fwd < 0 && moved <= -AUTO_SPOT_MAX_IN)) {
                    fwd = 0;
                }
            }
            robot.drive(fwd, 0, turn, 1.0);
            if (target != null && turn == 0 && fwd == 0) {
                if (++settled >= 3) {
                    break;
                }
            } else {
                settled = 0;
            }
            telemetry.addData("Target", target == null ? "none" : String.format("%s bearing %.1f floor %.0f",
                    target.cell, target.bearingDeg, target.horizontalIn()));
            loopTick();
        }
        robot.stopDrive();
    }

    /** Same pocket-by-pocket feed as TeleOp (ShotFeeder), after a short settle, until 4 balls left (flywheel dips), 3 turns, or the deadline. */
    private void fire(double matchDeadlineSec) {
        double aimedHeading = headingDeg();                     // hold the camera-aimed heading with the IMU while firing
        double settleEnd = clock.seconds() + AUTO_FIRE_SETTLE_SEC;    // balls settle in their pockets after the drive + turn
        while (opModeIsActive() && clock.seconds() < settleEnd) {
            robot.drive(0, 0, MotionController.holdTurnCw(aimedHeading, headingDeg()), 1.0);
            step = "FIRE settle";
            loopTick();
        }
        ShotFeeder feeder = ShotFeeder.fromSettings(spot);
        double end = Math.min(clock.seconds() + AUTO_FEED_TIMEOUT_SEC, matchDeadlineSec);
        robot.setIntake(FEED_INTAKE_BOOST);                     // as TeleOp: the intake pushes the balls through
        BallCounter counter = BallCounter.forAuto();            // balls from flywheel slowdowns after a fed pocket
        while (opModeIsActive() && clock.seconds() < end && feeder.shots() < AUTO_FEED_POCKETS
                && feeder.state() != ShotFeeder.State.JAMMED
                && !(counter.balls() >= AUTO_PRELOADS && feeder.state() != ShotFeeder.State.FEEDING)) {
            int ticks = robot.indexerTicks();
            double now = clock.seconds();
            robot.drive(0, 0, MotionController.holdTurnCw(aimedHeading, headingDeg()), 1.0);
            robot.setIndexerPower(feeder.update(now, ticks, robot.updateLauncherReady(), true));
            double velocity = robot.launcherVelocity();
            if (counter.update(now, feeder.state() == ShotFeeder.State.FEEDING, launchTps(), velocity)) {
                log.row(now - matchStart, "EVENT", "BALL " + counter.balls(), "launcher", velocity);
            }
            if (feeder.event() != null) {
                log.row(now - matchStart, "EVENT", feeder.event(), "ticks", ticks);
            }
            step = String.format("FIRE %d balls, pocket %d/%d %s", counter.balls(), feeder.shots(), AUTO_FEED_POCKETS,
                    feeder.state());
            loopTick();
        }
        robot.setIndexerPower(0);
        robot.setIntake(0);
        log.row(clock.seconds() - matchStart, "EVENT", String.format("FIRE done: %d balls, %d pockets, %d jams, %d dips ignored, %s",
                counter.balls(), feeder.shots(), feeder.jams(), counter.ignoredDips(),
                feeder.state() == ShotFeeder.State.JAMMED ? "JAMMED" : (clock.seconds() >= end ? "timeout" : "ok")));
    }

    private void closeVision() {
        if (vision != null) {
            vision.close();
            vision = null;
        }
    }
}

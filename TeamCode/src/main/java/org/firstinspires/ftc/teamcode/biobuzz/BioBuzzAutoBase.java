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
    private final ElapsedTime clock = new ElapsedTime();
    private double matchStart = 0;
    private String step = "INIT";
    private Pinpoint pinpoint;
    private final OdometryAccumulator odo = new OdometryAccumulator();
    private double lastWheelF, lastWheelS, pinpointHeading0;
    private boolean pinpointWasOk = false;

    @Override
    public void runOpMode() {
        robot = new BioBuzzRobot(hardwareMap);
        String visionError = null;
        try {
            vision = new HiveVision(hardwareMap);
        } catch (Exception e) {
            vision = null;
            visionError = e.getMessage();
        }
        BioBuzzMatchState.alliance = alliance();
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
                "cell", "tags", "bearing_deg", "odometry");
        log.row("CONFIG", "delay=" + startDelaySec, "shoot=" + shoot, "camera=" + (vision != null),
                "aim_heading=" + AUTO_AIM_HEADING_DEG, "launch_tps=" + AUTO_LAUNCH_VELOCITY);
        robot.imu.resetYaw();
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

    @Override
    public double headingDeg() {
        if (pinpointWasOk) {
            return MecanumMath.normalizeDegrees(pinpoint.headingDeg() - pinpointHeading0);
        }
        return robot.headingDeg();
    }

    @Override
    public void drive(double forward, double strafeRight, double turnCw) {
        robot.drive(forward, strafeRight, turnCw, 1.0);
    }

    /** Called right after START: zero the robot-relative odometry on the current readings. */
    private void startOdometry() {
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
        boolean ok = pinpoint != null && pinpoint.ok();
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
                pinpointWasOk ? "pinpoint" : "wheels");
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
        robot.setLauncherVelocity(AUTO_LAUNCH_VELOCITY);
    }

    @Override
    public void stopLauncher() {
        robot.setLauncherVelocity(0);
    }

    @Override
    public void aimAndFire(double matchDeadlineSec) {
        aimWithCamera(Math.min(clock.seconds() + AUTO_AIM_TIMEOUT_SEC, matchDeadlineSec));
        fire(matchDeadlineSec);
    }

    private void aimWithCamera(double endSec) {
        if (vision == null) {
            return;
        }
        HiveVision.Cell preferred = HiveVision.startingUpCell(alliance());
        int settled = 0;
        while (opModeIsActive() && clock.seconds() < endSec) {
            vision.update();
            HiveVision.Target target = vision.target(alliance(), preferred);
            lastTarget = target;
            double turn = target == null ? 0 : HiveVision.aimTurnCw(target.bearingDeg);
            robot.drive(0, 0, turn, 1.0);
            if (target != null && turn == 0) {
                if (++settled >= 3) {
                    break;
                }
            } else {
                settled = 0;
            }
            telemetry.addData("Target", target == null ? "none" : String.format("%s bearing %.1f range %.0f",
                    target.cell, target.bearingDeg, target.rangeIn));
            loopTick();
        }
        robot.stopDrive();
    }

    /** Feed the windmill only while the flywheel is at speed, until the preloads (plus one pocket) have passed. */
    /** One ball per shot (ShotFeeder) until the preloads plus one clearing pocket have gone, a jam, or the deadline. */
    private void fire(double matchDeadlineSec) {
        ShotFeeder feeder = ShotFeeder.standard();
        double end = Math.min(clock.seconds() + AUTO_FEED_TIMEOUT_SEC, matchDeadlineSec);
        while (opModeIsActive() && clock.seconds() < end && feeder.shots() < AUTO_FEED_POCKETS
                && feeder.state() != ShotFeeder.State.JAMMED) {
            robot.setIndexerPower(feeder.update(clock.seconds(), robot.indexerTicks(), robot.launcherReady(), true));
            if (feeder.event() != null) {
                log.row(clock.seconds() - matchStart, "EVENT", feeder.event(), "launcher", robot.launcherVelocity());
            }
            step = String.format("FIRE %d/%d %s", feeder.shots(), AUTO_FEED_POCKETS, feeder.state());
            loopTick();
        }
        robot.setIndexerPower(0);
        log.row(clock.seconds() - matchStart, "EVENT", String.format("FIRE done: %d shots, %d jams, %s", feeder.shots(),
                feeder.jams(), feeder.state() == ShotFeeder.State.JAMMED ? "JAMMED" : (clock.seconds() >= end ? "timeout" : "ok")));
    }

    private void closeVision() {
        if (vision != null) {
            vision.close();
            vision = null;
        }
    }
}

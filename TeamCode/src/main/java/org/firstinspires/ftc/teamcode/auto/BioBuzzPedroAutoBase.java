package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.AutoField.Alliance;
import org.firstinspires.ftc.teamcode.auto.AutoField.Pose;
import org.firstinspires.ftc.teamcode.auto.AutoField.Slot;
import org.firstinspires.ftc.teamcode.auto.AutoField.Start;
import org.firstinspires.ftc.teamcode.biobuzz.BallCounter;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzMatchState;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzRobot;
import org.firstinspires.ftc.teamcode.biobuzz.CsvLog;
import org.firstinspires.ftc.teamcode.biobuzz.HiveVision;
import org.firstinspires.ftc.teamcode.biobuzz.ShotFeeder;
import org.firstinspires.ftc.teamcode.biobuzz.ShotSpeed;
import org.firstinspires.ftc.teamcode.biobuzz.TeleOpSettings;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

/*
 * BIOBUZZ AUTO with Pedro Pathing (plan and field places: AutoPlanner / AutoField, AUTO.md).
 *
 *   1. SHOT     drive to the straight-on spot in front of our up CELL, camera aim, fire the 4 pre-loads
 *   2. GARDEN   only when the pre-loads are gone (G407: max 4 balls): sweep our 4 GARDEN POLLEN with the intake
 *   3. STRETCH  back to the spot, fire the GARDEN POLLEN (only if the camera sees our CELL still UP)
 *   4. PARK     in LOADING ZONE slot A (corner end) or B (middle end), intake 2 in off the wall: LEAVE 3 + PARK 5
 *
 * PARK ALWAYS WINS: every step is cut short at the park start (the INIT "park by" time, earlier when the drive to the
 * slot is long). Blocked for 1 s (e.g. by our partner): before the park the step is dropped and the robot parks;
 * while parking it waits and tries again until the end of AUTO.
 *
 * INIT (gamepad 1): D-pad L/R = start, Square/X = slot A/B, Cross/A = shoot, Triangle/Y = GARDEN, Circle/B = stretch,
 *                   D-pad U/D = start delay, LB/RB = park-by time.
 * Log: /sdcard/BioBuzzLogs/pedroauto_*.csv
 */
public abstract class BioBuzzPedroAutoBase extends LinearOpMode {

    protected abstract Alliance alliance();

    static final int PARK_BY_DEFAULT_SEC = 24;
    static final int PARK_BY_MIN_SEC = 18;
    static final int PARK_BY_MAX_SEC = 27;
    /** Not moving 1 in (or turning 3 deg) for this long while following a path = blocked. */
    static final double BLOCKED_SEC = 1.0;
    /** While parking: wait this long after a block, then try again. */
    static final double BLOCKED_WAIT_SEC = 0.75;
    static final double GARDEN_INTAKE_POWER = 1.0;
    /** Intake keeps running at the end of the GARDEN sweep (9 Nov: 0.6 s was too short), then again on the way back. */
    static final double GARDEN_DWELL_SEC = 1.5;
    /** Stretch only when at least this much time is left for aim + settle + a few pockets at the spot. */
    static final double STRETCH_MIN_FIRE_SEC = 4.0;
    /** CELL opening height when UP is ~59 in, when tipped down ~30-35 in. CALIBRATE: logged as height_in. */
    static final double CELL_UP_MIN_HEIGHT_IN = 45.0;
    /** Fallback without Pedro: back straight off the wall this far (LEAVE). */
    static final double FALLBACK_LEAVE_IN = 8.0;

    private Start start = Start.CELL_WALL;
    private Slot slot = Slot.A;
    private boolean shoot = true, garden = true, stretch = true;
    private int delaySec = 0;
    private int parkBySec = PARK_BY_DEFAULT_SEC;

    private BioBuzzRobot robot;
    private HiveVision vision;
    private Follower follower;
    private GoBildaPinpointDriver pinpointDev;
    private CsvLog log;
    private final TeleOpSettings settings = TeleOpSettings.load();
    private final ElapsedTime clock = new ElapsedTime();
    private double matchStart;
    private String step = "INIT";
    private HiveVision.Target lastTarget;
    private double launchTps = AUTO_LAUNCH_VELOCITY;
    private boolean preloadsGone = false;

    @Override
    public void runOpMode() {
        robot = new BioBuzzRobot(hardwareMap);                  // first: Pedro below takes over the drive motors
        String visionError = null;
        try {
            vision = new HiveVision(hardwareMap);
            vision.setAimTrimDeg(settings.aimTrimDeg);
        } catch (Exception e) {
            vision = null;
            visionError = e.getMessage();
        }
        String pedroError = null;
        try {
            follower = Constants.create(hardwareMap);
        } catch (Exception e) {
            follower = null;
            pedroError = e.getMessage();
        }
        try {
            pinpointDev = hardwareMap.tryGet(GoBildaPinpointDriver.class, PINPOINT);
        } catch (Exception e) {
            pinpointDev = null;
        }
        BioBuzzMatchState.rememberAlliance(hiveAlliance());

        while (opModeInInit()) {
            readInitButtons();
            if (follower != null) {
                follower.update();                              // idle: motors 0, keeps the Pinpoint status fresh
            }
            if (vision != null) {
                vision.update();
            }
            showInit(visionError, pedroError);
            sleep(20);
        }
        if (isStopRequested()) {
            closeVision();
            return;
        }

        log = new CsvLog("pedroauto_" + alliance(), "t", "step", "x_in", "y_in", "heading_deg", "launcher_tps",
                "launcher_target", "launcher_ready", "indexer_ticks", "battery_v", "cell", "tags", "bearing_deg",
                "height_in", "floor_in");
        log.row("CONFIG", "start=" + start, "slot=" + slot, "shoot=" + shoot, "garden=" + garden, "stretch=" + stretch,
                "delay=" + delaySec, "park_by=" + parkBySec, "camera=" + (vision != null),
                String.format("aim_trim=%.1f", settings.aimTrimDeg));
        matchStart = clock.seconds();
        try {
            String why = pedroProblem();
            if (why != null) {
                log.row(0, "EVENT", "NO PEDRO: " + why + " -> fallback LEAVE only");
                fallbackLeave();
            } else {
                follower.setPose(pedro(AutoField.start(start).forAlliance(alliance())));
                runPlan();
            }
            log.row("RESULT", "t=" + now(), "step=" + step, "pose=" + (follower == null ? "?" : here()));
            while (opModeIsActive()) {                          // hold the park until the AUTO timer ends
                if (follower != null && pedroProblem() == null) {
                    follower.update();
                }
                loopTick();
            }
        } finally {
            if (follower != null) {
                halt();
            }
            robot.stopAll();
            closeVision();
            log.close();
        }
    }

    // ---------------- Plan ----------------

    private void runPlan() {
        step = "DELAY";
        while (opModeIsActive() && now() < delaySec) {
            follower.update();
            loopTick();
        }
        if (shoot) {
            robot.setLauncherVelocity(launchTps);
            step = "TO SHOT";
            if (drive(AutoPlanner.toShot(start), false)) {
                volley(true);
            }
        }
        boolean collected = false;
        if (garden && preloadsGone
                && AutoPlanner.fits(now(), redHere(), AutoPlanner.toGarden(), GARDEN_DWELL_SEC, slot, parkBySec)) {
            if (!stretch) {
                robot.setLauncherVelocity(0);
            }
            step = "GARDEN";
            collected = drive(AutoPlanner.toGarden(), false);
            if (collected) {
                double end = Math.min(now() + GARDEN_DWELL_SEC, parkStart());
                while (opModeIsActive() && now() < end) {
                    follower.update();
                    loopTick();
                }
            }
            robot.setIntake(0);
        } else if (garden) {
            log.row(now(), "EVENT", preloadsGone ? "GARDEN skipped: no time" : "GARDEN skipped: pre-loads not all fired (G407)");
        }
        if (stretch && collected
                && AutoPlanner.fits(now(), redHere(), AutoPlanner.gardenToShot(), STRETCH_MIN_FIRE_SEC, slot, parkBySec)) {
            step = "BACK TO SHOT";
            if (drive(AutoPlanner.gardenToShot(), false)) {
                volley(false);
            }
        }
        robot.setLauncherVelocity(0);
        robot.setIndexerPower(0);
        robot.setIntake(0);
        park();
    }

    private void park() {
        step = "PARK";
        log.row(now(), "EVENT", String.format("PARK %s from %s (deadline was %.1f s)", slot, redHere(), parkStart()));
        while (opModeIsActive() && now() < AutoPlanner.MATCH_SEC) {
            if (drive(AutoPlanner.park(redHere(), slot), true)) {
                step = "PARKED " + slot;
                follower.hold(pedro(AutoField.slot(slot).forAlliance(alliance())));
                return;
            }
            if (now() >= AutoPlanner.MATCH_SEC - 0.3 || poseLost()) {
                return;
            }
            if (AutoPlanner.partlyInZone(redHere())) {         // blocked but PARK already counts: stay, don't back out
                step = "PARKED (blocked) " + slot;
                log.row(now(), "EVENT", "blocked inside the LOADING ZONE: holding at " + here());
                follower.hold(follower.pose());
                return;
            }
        }
    }

    /**
     * Follows the legs (RED frame, mirrored for BLUE). Returns false when cut short: the park start (not while
     * parking), blocked (not while parking: there it waits and re-plans from where it is), or the end of AUTO.
     */
    private boolean drive(List<AutoPlanner.Leg> legs, boolean parking) {
        for (AutoPlanner.Leg leg : legs) {
            Pose target = leg.to.forAlliance(alliance());
            follower.follow(path(target, leg.speed));
            ElapsedTime legTimer = new ElapsedTime(), still = new ElapsedTime();
            Pose mark = here();
            double timeout = AutoPlanner.legSec(redHere(), leg) * 2 + 2;
            log.row(now(), "EVENT", "LEG " + leg.label + " -> " + target);
            while (true) {
                if (!opModeIsActive()) {
                    return false;
                }
                follower.update();
                robot.setIntake(leg.action == AutoPlanner.Action.INTAKE ? GARDEN_INTAKE_POWER : 0);
                if (poseLost()) {
                    log.row(now(), "EVENT", "POSE LOST " + here());
                    halt();
                    return false;
                }
                if (!parking && now() >= parkStart()) {
                    log.row(now(), "EVENT", "PARK TIME during " + leg.label);
                    halt();
                    return false;
                }
                if (parking && now() >= AutoPlanner.MATCH_SEC - 0.3) {
                    halt();
                    return false;
                }
                if (legTimer.seconds() > 0.2 && !follower.isBusy()) {
                    break;
                }
                Pose p = here();
                if (p.distanceTo(mark) >= 1.0
                        || Math.abs(AutoField.normalizeDeg(p.headingDeg - mark.headingDeg)) >= 3.0) {
                    mark = p;
                    still.reset();
                } else if ((legTimer.seconds() > 0.5 && still.seconds() > BLOCKED_SEC) || legTimer.seconds() > timeout) {
                    log.row(now(), "EVENT", "BLOCKED on " + leg.label + " at " + p);
                    halt();
                    if (!parking) {
                        return false;
                    }
                    ElapsedTime wait = new ElapsedTime();
                    while (opModeIsActive() && wait.seconds() < BLOCKED_WAIT_SEC) {
                        follower.update();
                        loopTick();
                    }
                    return false;                               // park() re-plans from here
                }
                step = (parking ? "PARK: " : "") + leg.label;
                loopTick();
            }
        }
        return true;
    }

    /** Camera aim (Pedro holds the spot and turns), CELL-up check, then fire. Sets preloadsGone after the first one. */
    private void volley(boolean preloads) {
        step = preloads ? "AIM (pre-loads)" : "AIM (GARDEN POLLEN)";
        HiveVision.Target seen = aim(Math.min(now() + AUTO_AIM_TIMEOUT_SEC, parkStart()));
        if (seen != null) {
            boolean up = seen.heightIn() >= CELL_UP_MIN_HEIGHT_IN;
            log.row(now(), "EVENT", String.format("CELL %s %s: height %.1f in, floor %.1f in, bearing %.1f",
                    seen.cell, up ? "UP" : "DOWN", seen.heightIn(), seen.horizontalIn(), seen.bearingDeg));
            if (!up) {
                return;                                         // G417: never launch into a downward CELL
            }
            launchTps = ShotSpeed.standard().tableSpeed(seen.horizontalIn());
            robot.setLauncherVelocity(launchTps);
        } else if (!preloads) {
            log.row(now(), "EVENT", "no CELL in view: GARDEN POLLEN kept");
            return;
        } else {
            log.row(now(), "EVENT", "no CELL in view: firing on the planned heading");
        }
        fire(preloads);
    }

    /** Turns on the spot (Pedro hold) until the camera bearing is within AIM_TOLERANCE_DEG on 3 frames. */
    private HiveVision.Target aim(double endSec) {
        if (vision == null) {
            return null;
        }
        HiveVision.Cell ours = HiveVision.startingUpCell(hiveAlliance());
        com.pedropathing.math.Pose spot = follower.pose();
        double targetHeading = spot.heading();
        follower.hold(spot);
        HiveVision.Target last = null;
        int settled = 0;
        while (opModeIsActive() && now() < endSec) {
            follower.update();
            if (vision.update()) {
                HiveVision.Target t = vision.target(hiveAlliance(), ours);
                if (t != null && t.cell != ours) {
                    t = null;
                }
                lastTarget = t;
                if (t != null) {
                    last = t;
                    double headingErr = AutoField.normalizeDeg(Math.toDegrees(follower.pose().heading() - targetHeading));
                    if (Math.abs(t.bearingDeg) <= AIM_TOLERANCE_DEG) {
                        if (++settled >= 3) {
                            break;
                        }
                    } else {
                        settled = 0;
                        if (Math.abs(headingErr) < AIM_TOLERANCE_DEG) {   // only re-aim on frames taken after the last turn
                            targetHeading = follower.pose().heading() + Math.toRadians(t.bearingDeg);
                            follower.hold(spot.withHeading(targetHeading));
                        }
                    }
                }
            }
            loopTick();
        }
        return last;
    }

    /** The TeleOp pocket-by-pocket feed (ShotFeeder) while Pedro holds the aimed pose, until the park start. */
    private void fire(boolean preloads) {
        follower.hold(follower.pose());
        double settleEnd = Math.min(now() + AUTO_FIRE_SETTLE_SEC, parkStart());
        while (opModeIsActive() && now() < settleEnd) {
            follower.update();
            step = "FIRE settle";
            loopTick();
        }
        ShotFeeder feeder = ShotFeeder.fromSettings(settings);
        BallCounter counter = BallCounter.forAuto();
        robot.setIntake(FEED_INTAKE_BOOST);
        double feedEnd = now() + AUTO_FEED_TIMEOUT_SEC;
        while (opModeIsActive() && now() < Math.min(feedEnd, parkStart()) && feeder.shots() < AUTO_FEED_POCKETS
                && feeder.state() != ShotFeeder.State.JAMMED
                && !(counter.balls() >= AUTO_PRELOADS && feeder.state() != ShotFeeder.State.FEEDING)) {
            follower.update();
            double t = now();
            int ticks = robot.indexerTicks();
            robot.setIndexerPower(feeder.update(t, ticks, robot.updateLauncherReady(), true));
            double velocity = robot.launcherVelocity();
            if (counter.update(t, feeder.state() == ShotFeeder.State.FEEDING, launchTps, velocity)) {
                log.row(t, "EVENT", "BALL " + counter.balls(), "launcher", velocity);
            }
            if (feeder.event() != null) {
                log.row(t, "EVENT", feeder.event(), "ticks", robot.indexerTicks());
            }
            step = String.format("FIRE %d balls, pocket %d/%d %s", counter.balls(), feeder.shots(), AUTO_FEED_POCKETS,
                    feeder.state());
            loopTick();
        }
        robot.setIndexerPower(0);
        robot.setIntake(0);
        if (preloads) {
            // G407: only collect more once the windmill is empty (4 balls counted, or 3 full turns of the windmill)
            preloadsGone = counter.balls() >= AUTO_PRELOADS || feeder.shots() >= AUTO_FEED_POCKETS;
        }
        log.row(now(), "EVENT", String.format("FIRE done: %d balls, %d pockets, %d jams, %s%s", counter.balls(),
                feeder.shots(), feeder.jams(), feeder.state() == ShotFeeder.State.JAMMED ? "JAMMED" : "ok",
                preloads ? (preloadsGone ? ", windmill empty" : ", balls may be left (no GARDEN)") : ""));
    }

    /** Without Pedro: back straight off the wall on the wheel encoders (both starts face the wall): LEAVE. */
    private void fallbackLeave() {
        step = "FALLBACK LEAVE";
        while (opModeIsActive() && now() < delaySec) {
            loopTick();
        }
        double f0 = robot.forwardInches();
        double end = now() + 2.0;
        while (opModeIsActive() && now() < end && robot.forwardInches() - f0 > -FALLBACK_LEAVE_IN) {
            robot.drive(-0.3, 0, 0, 1.0);
            loopTick();
        }
        robot.stopDrive();
        step = "FALLBACK DONE (LEAVE only)";
    }

    // ---------------- Helpers ----------------

    /** Latest time to start the park from where the robot is now. */
    private double parkStart() {
        return AutoPlanner.parkStartSec(redHere(), slot, parkBySec);
    }

    private double now() {
        return clock.seconds() - matchStart;
    }

    private Path path(Pose target, double speed) {
        com.pedropathing.math.Pose from = follower.pose();
        com.pedropathing.math.Pose to = pedro(target);
        return Paths.line(from, to)
                .linear(from.heading(), to.heading())                       // turns the short way
                .with(Constants.foresightConfig.maxPathSpeed.at(speed));     // a FRACTION of top speed
    }

    /** Field frame (AutoField / FieldMap) to Pedro (origin in a corner, same axes). */
    private static com.pedropathing.math.Pose pedro(Pose p) {
        return new com.pedropathing.math.Pose(p.x + 72, p.y + 72, Math.toRadians(p.headingDeg));
    }

    private Pose here() {
        com.pedropathing.math.Pose p = follower.pose();
        return new Pose(p.x() - 72, p.y() - 72, Math.toDegrees(p.heading()));
    }

    /** Where we are, in the RED frame the planner uses. */
    private Pose redHere() {
        return here().forAlliance(alliance());
    }

    private boolean poseLost() {
        Pose p = here();
        return Double.isNaN(p.x) || Double.isNaN(p.y) || Double.isNaN(p.headingDeg)
                || Math.abs(p.x) > AutoField.WALL_IN + 6 || Math.abs(p.y) > AutoField.WALL_IN + 6;
    }

    /** Follower.stop() only zeroes the motors on a later update(): cut them here, at once. */
    private void halt() {
        follower.stop();
        follower.drivetrain.stop();
    }

    private String pedroProblem() {
        if (follower == null) {
            return "Pedro could not start";
        }
        if (!Constants.FORESIGHT_TUNED) {
            return "Pedro not tuned";
        }
        if (pinpointDev == null) {
            return "no \"" + PINPOINT + "\" in the configuration";
        }
        GoBildaPinpointDriver.DeviceStatus s = pinpointDev.getDeviceStatus();
        return s == GoBildaPinpointDriver.DeviceStatus.READY ? null : "Pinpoint " + s;
    }

    private HiveVision.Alliance hiveAlliance() {
        return alliance() == Alliance.RED ? HiveVision.Alliance.RED : HiveVision.Alliance.BLUE;
    }

    private void closeVision() {
        if (vision != null) {
            vision.close();
            vision = null;
        }
    }

    private void loopTick() {
        double t = now();
        Pose p = follower == null ? null : here();
        HiveVision.Target target = lastTarget;
        if (log != null) {
            log.row(t, step, p == null ? "" : p.x, p == null ? "" : p.y, p == null ? "" : p.headingDeg,
                    robot.launcherVelocity(), robot.launcherTarget(), robot.launcherReady(), robot.indexerTicks(),
                    robot.battery.getVoltage(), target == null ? "" : target.cell, target == null ? 0 : target.tagCount,
                    target == null ? "" : target.bearingDeg, target == null ? "" : target.heightIn(),
                    target == null ? "" : target.horizontalIn());
        }
        lastTarget = null;
        telemetry.addData("Step", step);
        telemetry.addData("Time", "%.1f s   (park starts by %.1f s)", t, follower == null ? 0 : parkStart());
        if (p != null) {
            telemetry.addData("Where", "x %.1f  y %.1f in  heading %.0f", p.x, p.y, p.headingDeg);
        }
        telemetry.addData("Launcher", "%.0f / %.0f t/s", robot.launcherVelocity(), robot.launcherTarget());
        telemetry.addData("Log", log == null ? "-" : log.status());
        telemetry.update();
    }

    // ---------------- INIT ----------------

    private void readInitButtons() {
        if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed()) {
            start = start.next();
        }
        if (gamepad1.xWasPressed()) {
            slot = slot.other();
        }
        if (gamepad1.aWasPressed()) {
            shoot = !shoot;
        }
        if (gamepad1.yWasPressed()) {
            garden = !garden;
        }
        if (gamepad1.bWasPressed()) {
            stretch = !stretch;
        }
        if (gamepad1.dpadUpWasPressed()) {
            delaySec = Math.min(AUTO_MAX_START_DELAY_SEC, delaySec + 1);
        } else if (gamepad1.dpadDownWasPressed()) {
            delaySec = Math.max(0, delaySec - 1);
        }
        if (gamepad1.rightBumperWasPressed()) {
            parkBySec = Math.min(PARK_BY_MAX_SEC, parkBySec + 1);
        } else if (gamepad1.leftBumperWasPressed()) {
            parkBySec = Math.max(PARK_BY_MIN_SEC, parkBySec - 1);
        }
    }

    private void showInit(String visionError, String pedroError) {
        Pose s = AutoField.start(start);
        telemetry.addLine("BIOBUZZ AUTO " + alliance() + " - Pedro");
        telemetry.addData("Start (D-pad L/R)", start.label);
        telemetry.addData("Park slot (Square/X)", slot.label);
        telemetry.addLine(">> Tell your partner: we park in slot " + slot + ", they take slot " + slot.other());
        telemetry.addData("Shoot pre-loads (Cross/A)", shoot ? "ON" : "OFF");
        telemetry.addData("GARDEN POLLEN (Triangle/Y)", !garden ? "OFF" : shoot ? "ON" : "ON but needs shooting (G407)");
        telemetry.addData("Stretch: shoot again (Circle/B)", stretch ? "ON" : "OFF");
        telemetry.addData("Start delay (D-pad U/D)", "%d s", delaySec);
        telemetry.addData("Park by (LB/RB)", "%d s at the latest", parkBySec);

        double t = delaySec;
        Pose at = s;
        if (shoot) {
            t += AutoPlanner.estimateSec(at, AutoPlanner.toShot(start)) + AUTO_AIM_TIMEOUT_SEC / 2 + 4;
            at = AutoField.SHOT;
            telemetry.addData("Plan", "shot spot + 4 pre-loads done ~%.0f s", t);
        }
        if (shoot && garden && AutoPlanner.fits(t, at, AutoPlanner.toGarden(), GARDEN_DWELL_SEC, slot, parkBySec)) {
            t += AutoPlanner.estimateSec(at, AutoPlanner.toGarden()) + GARDEN_DWELL_SEC;
            at = AutoPlanner.end(at, AutoPlanner.toGarden());
            telemetry.addData("Plan", "GARDEN done ~%.0f s", t);
        }
        double parkAt = Math.max(t, Math.min(AutoPlanner.parkStartSec(at, slot, parkBySec), parkBySec));
        telemetry.addData("Plan", "PARK %s starts by %.0f s, takes ~%.0f s", slot,
                AutoPlanner.parkStartSec(at, slot, parkBySec), AutoPlanner.estimateSec(at, AutoPlanner.park(at, slot)));
        String route = AutoPlanner.routeProblem(s, AutoPlanner.toShot(start), null);
        if (route == null) {
            route = AutoPlanner.routeProblem(at, AutoPlanner.park(at, slot), slot.other());
        }
        telemetry.addData("Route check", route == null ? "OK" : "PROBLEM: " + route);
        if (parkAt > AutoPlanner.MATCH_SEC - AutoPlanner.PARK_MARGIN_SEC) {
            telemetry.addLine("WARNING: start delay too long, the park may not finish");
        }

        String why = pedroProblem();
        telemetry.addData("Pedro", why == null ? "ready" : "NOT READY: " + why + (pedroError != null ? " (" + pedroError
                + ")" : "") + " -> AUTO will only back off the wall (LEAVE)");
        if (vision == null) {
            telemetry.addData("Camera", "NOT FOUND: %s (fires pre-loads unaimed, no stretch)", visionError);
        } else {
            HiveVision.Target tg = vision.target(hiveAlliance(), HiveVision.startingUpCell(hiveAlliance()));
            telemetry.addData("Camera", tg == null ? "no CELL in view (normal at the wall)" : String.format(
                    "%s %d tags, height %.0f in", tg.cell, tg.tagCount, tg.heightIn()));
        }
        telemetry.addLine("Place: " + (start == Start.CELL_WALL
                ? "intake against the " + (alliance() == Alliance.RED ? "AUDIENCE" : "REAR")
                + " wall, centred on our up CELL, square to the wall"
                : "intake against our ALLIANCE wall, centred on the middle line, square to the wall"));
        telemetry.update();
    }
}

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
import org.firstinspires.ftc.teamcode.biobuzz.TeleOpSettings;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

/*
 * BIOBUZZ AUTO with Pedro Pathing (plan and field places: AutoPlanner / AutoField, AUTO.md).
 *
 *   1. SHOT     drive to the straight-on spot in front of our up CELL, camera aim, fire the 4 pre-loads
 *   2. GARDEN   only when the pre-loads are gone (G407: max 4 balls): sweep our 4 GARDEN POLLEN with the intake, then
 *               2 in back and forth at the corner while the intake keeps running
 *   3. STRETCH  back to the spot, fire the GARDEN POLLEN (only if the camera sees our CELL still UP)
 *   4. PARK     in LOADING ZONE slot A (corner end) or B (middle end), intake 2 in off the wall: LEAVE 3 + PARK 5
 *
 * PARK ALWAYS WINS: every step is cut short at the park start (the INIT "park by" time, earlier when the drive to the
 * slot is long). Blocked for 1 s (e.g. by our partner): before the park the step is dropped and the robot parks;
 * while parking it waits and tries again until the end of AUTO.
 *
 * NO PEDRO (the Pinpoint not READY at the start pose at START, after up to PINPOINT_START_WAIT_SEC: startPedro) or the
 * pose lost before the robot left the start: FALLBACK on the wheel encoders and the camera. S1: back off to the shot
 * spot, camera aim, fire the pre-loads (G417 check), back off for LEAVE. S2: LEAVE only.
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
    /**
     * Intake keeps running at the end of the GARDEN sweep, up to this long (9 Nov: 1 s at the corner did not take 4
     * POLLEN), the robot going AutoField.GARDEN_WIGGLE_IN back and forth meanwhile (12 Nov). Cut short at the park
     * start; the GARDEN is only started when at least GARDEN_MIN_DWELL_SEC fits.
     */
    static final double GARDEN_DWELL_SEC = 4.0;
    static final double GARDEN_MIN_DWELL_SEC = 1.0;
    /** One move of the GARDEN back-and-forth ends after this long even when it can't finish (POLLEN or the wall). */
    static final double GARDEN_WIGGLE_MOVE_SEC = 1.0;
    /** Stretch only when at least this much time is left for aim + settle + a few pockets at the spot. */
    static final double STRETCH_MIN_FIRE_SEC = 4.0;
    /** Fallback without Pedro: back straight off the wall this far (LEAVE). */
    static final double FALLBACK_LEAVE_IN = 8.0;
    /**
     * After a pose loss away from the start: back up until the intake is this far off the wall in front of it (LEAVE =
     * not touching a wall at 0:00; the last trusted pose can be ~2 in off).
     */
    static final double CLEAR_WALL_IN = 4.0;
    /** Fallback: firing ends by then, so the robot can still back off for LEAVE. */
    static final double FALLBACK_FIRE_END_SEC = AutoPlanner.MATCH_SEC - 3.0;
    /** One encoder back-off ends after this long even if it is short (stuck): never a long blind drive. */
    static final double BACK_OFF_MAX_SEC = 2.0;
    /**
     * Pose lost: the Pinpoint not READY this long while Pedro drives. On a bad read the driver keeps the last pose and
     * Pedro reads it without checking the status, so it would drive on a frozen pose.
     * Meanwhile the robot stands still (power cut on every bad reading); READY again in time = carry on.
     */
    static final double PINPOINT_LOST_SEC = 0.25;
    /**
     * Pose lost: it jumps more between two checks than the robot can move in that time (1.5 x top speed, HEADING_MAX_RATE_DPS)
     * plus these margins. A Pinpoint that restarts reports its origin, the RED GARDEN corner, which is inside the field.
     * Replayed on all 37 AUTO logs (35,762 loop pairs, 9-13 Nov): with a 1 in margin the closest normal pair had ~1 in to
     * spare (park legs: loop timing jitter), so 3 in; the heading margin was never closer than 8.6 deg.
     */
    static final double POSE_JUMP_IN = 3.0;
    static final double POSE_JUMP_DEG = 5.0;
    /** The jump check starts with the first check this close to the START pose. */
    static final double POSE_ARM_IN = 3.0;
    /** START: the pose read back after setPose (robot still) must be this close to the start pose, twice in a row. */
    static final double START_POSE_TOL_IN = 0.25;
    static final double START_POSE_TOL_DEG = 1.0;
    /** The aim without Pedro (raw turn power) stops this much before AutoField.AIM_MAX_YAW_DEG: it coasts on ... */
    static final double AIM_COAST_DEG = 1.5;
    /** ... and turns at AIM_MIN_TURN only once it is this close to that stop. */
    static final double AIM_SLOW_ZONE_DEG = 3.0;
    /** START: the Pinpoint gets this long to be READY at the start pose (startPedro) before AUTO falls back. */
    static final double PINPOINT_START_WAIT_SEC = 1.0;
    /** INIT: a Pinpoint in a FAULT for this long gets a reset (team rule: the robot stands still in INIT) ... */
    static final double PINPOINT_RESET_AFTER_SEC = 2.0;
    /** ... and again at most this often while the fault lasts. */
    static final double PINPOINT_RESET_EVERY_SEC = 5.0;
    /** INIT warning below this (13 Nov: from ~11.9 V on, the Expansion Hub dropped out and the hubs cut the motors). */
    static final double LOW_BATTERY_V = 12.5;

    private Start start = Start.CELL_WALL;
    private Slot slot = Slot.B;                                 // B: no driving past a partner parked in the other slot
    private boolean shoot = true, garden = true, stretch = false;
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
    private boolean preloadsTried = false;                      // the pre-load volley started (aim / fire)
    private boolean poseSet = false;                            // Pedro runs from the known start pose
    private boolean pedroDrives = false;                        // Pedro drives (false: no Pedro, or the pose was lost)
    private boolean poseLostOnce = false;
    private boolean poseJumped = false;
    private boolean pinpointLost = false;                       // not READY for PINPOINT_LOST_SEC (latched)
    private Pose lastPose;                                      // pose at the last check (jump check; set from the start)
    private double lastPoseSec;                                 // ... and its match time
    private double pinpointBadSince = Double.NaN;               // match time the Pinpoint stopped being READY
    private Pose lastGoodRed;                                   // last trusted pose, RED frame
    private boolean leftStart = false;                          // at least FALLBACK_LEAVE_IN from the start (LEAVE)
    private double startForwardIn;                              // wheel encoders at START (fallback back-offs)
    private final List<Double> aimHeights = new ArrayList<>();  // our CELL's height on each frame of the last aim
    private final List<Double> aimFloors = new ArrayList<>();   // ... and its floor distance (log only)
    private final List<Double> initHeights = new ArrayList<>(); // our CELL's height on the latest INIT frames
    private HiveVision.Target initCell;                         // our CELL on the latest INIT frame
    private double pinpointFaultSince = Double.NaN;
    private double lastPinpointReset = -1e9;
    private int pinpointResets = 0;

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
        if (pinpointDev != null) {
            // INIT reads unchecked: the driver's last taken position may be stale (e.g. from a TeleOp in its own frame) and
            // would be refused for good otherwise (see startPedro). The robot stands still in INIT.
            pinpointDev.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.NONE);
        }
        BioBuzzMatchState.rememberAlliance(hiveAlliance());

        while (opModeInInit()) {
            readInitButtons();
            if (follower != null) {
                follower.update();                              // idle: motors 0, keeps the Pinpoint status fresh
            }
            recoverPinpointInInit();
            if (vision != null && vision.update()) {
                noteInitCell();
            }
            showInit(visionError, pedroError);
            sleep(20);
        }
        if (isStopRequested()) {
            if (pinpointDev != null) {
                pinpointDev.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.LOCAL_TEST);
            }
            closeVision();
            return;
        }

        log = new CsvLog("pedroauto_" + alliance(), "t", "step", "x_in", "y_in", "heading_deg", "launcher_tps",
                "launcher_target", "launcher_ready", "indexer_ticks", "battery_v", "cell", "tags", "bearing_deg",
                "height_in", "floor_in");
        log.row("CONFIG", "start=" + start, "slot=" + slot, "shoot=" + shoot, "garden=" + garden, "stretch=" + stretch,
                "delay=" + delaySec, "park_by=" + parkBySec, "camera=" + (vision != null),
                String.format("aim_trim=%.1f", settings.aimTrimDeg), String.format("battery=%.1f", robot.battery.getVoltage()),
                "pinpoint=" + pinpointStatus(), "pinpoint_resets=" + pinpointResets,
                String.format("launcher=%.0f", launchTps));
        matchStart = clock.seconds();
        startForwardIn = robot.forwardInches();
        // TeleOp field-centric start: the robot's heading relative to its drivers. The RED frame's heading is exactly
        // that (RED drivers look along +X; BLUE poses are mirrored into it). The start pose holds for the no-Pedro
        // fallback too (it only backs straight off the wall); loopTick() updates it while Pedro drives.
        BioBuzzMatchState.recordAutoHeading(AutoField.start(start).headingDeg);
        try {
            String why = startPedro();
            if (why != null) {
                log.row(now(), "EVENT", "NO PEDRO: " + why + " -> " + fallbackPlan());
                fallback();
            } else {
                poseSet = true;
                pedroDrives = true;
                runPlan();
            }
            log.row("RESULT", "t=" + now(), "step=" + step, "pose=" + (follower == null ? "?" : here()));
            while (opModeIsActive()) {                          // hold the park until the AUTO timer ends
                holdTick();
                loopTick();
            }
        } finally {
            if (follower != null) {
                halt();
            }
            if (pinpointDev != null) {                          // leave the driver as the SDK made it (TeleOp next)
                pinpointDev.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.LOCAL_TEST);
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
            holdTick();
            loopTick();
        }
        if (shoot) {
            robot.setLauncherVelocity(launchTps);
            step = "TO SHOT";
            if (drive(AutoPlanner.toShot(start), false)) {
                volley(true);
            }
        }
        if (poseLostOnce) {                                     // no pose: no GARDEN, no park
            afterPoseLost();
            return;
        }
        boolean collected = false;
        if (garden && preloadsGone
                && AutoPlanner.fits(now(), redHere(), AutoPlanner.toGarden(), GARDEN_MIN_DWELL_SEC, slot, parkBySec)) {
            if (!stretch) {
                robot.setLauncherVelocity(0);
            }
            step = "GARDEN";
            collected = drive(AutoPlanner.toGarden(), false);
            if (collected) {
                gardenWiggle(Math.min(now() + GARDEN_DWELL_SEC, parkStart()));
            }
            robot.setIntake(0);
        } else if (garden) {
            log.row(now(), "EVENT", preloadsGone ? "GARDEN skipped: no time" : "GARDEN skipped: pre-loads not all fired (G407)");
        }
        if (poseLostOnce) {
            afterPoseLost();
            return;
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
        if (poseLostOnce) {
            afterPoseLost();
            return;
        }
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
            if (poseLostOnce) {
                afterPoseLost();
                return;
            }
            if (now() >= AutoPlanner.MATCH_SEC - 0.3) {
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
        if (!pedroDrives) {
            return false;                                       // never follow a path on a lost pose
        }
        for (AutoPlanner.Leg leg : legs) {
            if (poseLost()) {                                   // never start a path from a bad pose
                losePose();
                return false;
            }
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
                pedroUpdate();
                robot.setIntake(leg.action == AutoPlanner.Action.INTAKE ? GARDEN_INTAKE_POWER : 0);
                if (poseLost()) {
                    losePose();
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
                } else if (leg.intoWall() && legTimer.seconds() > 0.5 && still.seconds() > AutoPlanner.WALL_SETTLE_SEC
                        && AutoPlanner.atWall(redHere(), leg)) {
                    log.row(now(), "EVENT", "AGAINST THE WALL on " + leg.label + " at " + p);
                    break;                                      // aimed past the wall: the wall (or a POLLEN) stopped it
                } else if ((legTimer.seconds() > 0.5 && still.seconds() > BLOCKED_SEC) || legTimer.seconds() > timeout) {
                    if (AutoPlanner.atWall(redHere(), leg)) {
                        log.row(now(), "EVENT", "LEG TIME UP against the wall on " + leg.label + " at " + p);
                        break;
                    }
                    log.row(now(), "EVENT", "BLOCKED on " + leg.label + " at " + p);
                    halt();
                    if (!parking) {
                        return false;
                    }
                    ElapsedTime wait = new ElapsedTime();
                    while (opModeIsActive() && wait.seconds() < BLOCKED_WAIT_SEC && pedroDrives) {
                        holdTick();
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

    /**
     * At the GARDEN corner until {@code endSec}: GARDEN_WIGGLE_IN back along the wall and in again, over and over, with
     * the intake on. A move that can't finish (POLLEN or the wall in the way) ends after GARDEN_WIGGLE_MOVE_SEC: no
     * BLOCKED here, the corner is the end of the sweep anyway.
     */
    private void gardenWiggle(double endSec) {
        log.row(now(), "EVENT", String.format("GARDEN WIGGLE %.1f in until %.1f s", AutoField.GARDEN_WIGGLE_IN, endSec));
        int moves = 0;
        double deepest = redHere().x;                           // closest to the RED wall (wall contact, for calibration)
        while (opModeIsActive() && now() < endSec) {
            // out: GARDEN_WIGGLE_IN back from where the wall (or a POLLEN) stopped the robot; in: push past the wall
            for (AutoPlanner.Leg leg : AutoPlanner.gardenWiggle(redHere())) {
                follower.follow(path(leg.to.forAlliance(alliance()), leg.speed));
                ElapsedTime move = new ElapsedTime();
                step = leg.label;
                while (opModeIsActive() && now() < endSec && move.seconds() < GARDEN_WIGGLE_MOVE_SEC
                        && (move.seconds() < 0.2 || follower.isBusy())) {
                    pedroUpdate();
                    robot.setIntake(GARDEN_INTAKE_POWER);
                    if (poseLost()) {
                        losePose();
                        return;
                    }
                    deepest = Math.min(deepest, redHere().x);
                    loopTick();
                }
                moves++;
                if (!opModeIsActive() || now() >= endSec) {
                    break;
                }
            }
        }
        halt();
        log.row(now(), "EVENT", String.format("GARDEN WIGGLE done: %d moves, deepest x %.1f (bumper on the wall at "
                + "x %.1f), at %s", moves, deepest, AutoField.GARDEN_CORNER_X, here()));
    }

    /** Camera aim (Pedro holds the spot and turns), CELL-up check, then fire. Sets preloadsGone after the first one. */
    private void volley(boolean preloads) {
        if (preloads) {
            preloadsTried = true;
        }
        step = preloads ? "AIM (pre-loads)" : "AIM (GARDEN POLLEN)";
        HiveVision.Target seen = aim(Math.min(now() + AUTO_AIM_TIMEOUT_SEC, fireDeadline()));
        if (!preloads && seen != null
                && vision.targetCell(hiveAlliance(), HiveVision.startingUpCell(hiveAlliance())) == null) {
            // the GARDEN POLLEN go only into a CELL in view now: an UP frame from earlier in the aim may be from before a TIP
            log.row(now(), "EVENT", "our CELL out of view at the end of the aim: GARDEN POLLEN kept");
            return;
        }
        if (cellAllowsFire(seen, preloads)) {
            fire(preloads);
        }
    }

    /**
     * G417 check on our CELL from the last aim (median height of its last frames, CellCheck). No CELL seen: the pre-loads
     * still go on the planned heading, GARDEN POLLEN are kept. The speed stays AUTO_LAUNCH_VELOCITY: every AUTO shot is
     * from the one shot spot, where the camera distance only adds noise (the floor distance is logged for calibration).
     */
    private boolean cellAllowsFire(HiveVision.Target seen, boolean preloads) {
        if (seen == null) {
            log.row(now(), "EVENT", preloads ? "no CELL in view: firing on the planned heading"
                    : "no CELL in view: GARDEN POLLEN kept");
            return preloads;
        }
        double h = CellCheck.median(aimHeights);
        boolean up = CellCheck.up(aimHeights);
        log.row(now(), "EVENT", String.format("CELL %s %s: height %.1f in (median of %d frames, last %.1f), floor %.1f in,"
                + " bearing %.1f, launcher %.0f t/s", seen.cell, up ? "UP" : "DOWN", h,
                Math.min(aimHeights.size(), CellCheck.FRAMES), seen.heightIn(), CellCheck.median(aimFloors),
                seen.bearingDeg, launchTps));
        return up;                                              // G417: never launch into a downward CELL
    }

    /**
     * Turns on the spot until the camera bearing is within AIM_TOLERANCE_DEG on 3 frames: Pedro holds the spot and turns;
     * without Pedro (or once the pose is lost) the drive turns on the camera bearing alone (fallbackAim).
     */
    private HiveVision.Target aim(double endSec) {
        aimHeights.clear();
        aimFloors.clear();
        if (vision == null) {
            return null;
        }
        double imu0 = robot.headingDeg();                       // the whole aim, with or without Pedro, turns from here
        if (!pedroDrives) {
            return fallbackAim(endSec, null, imu0);
        }
        HiveVision.Cell ours = HiveVision.startingUpCell(hiveAlliance());
        com.pedropathing.math.Pose spot = follower.pose();
        double planned = Math.toRadians(AutoField.SHOT.forAlliance(alliance()).headingDeg);
        double targetHeading = spot.heading();
        follower.hold(spot);
        HiveVision.Target last = null;
        int settled = 0;
        while (opModeIsActive() && now() < endSec) {
            pedroUpdate();
            if (poseLost()) {
                losePose();
                return fallbackAim(endSec, last, imu0);         // keep aiming on the camera alone
            }
            if (vision.update()) {
                HiveVision.Target t = vision.targetCell(hiveAlliance(), ours);
                lastTarget = t;
                if (t != null) {
                    last = t;
                    noteAimFrame(t);
                    double headingErr = AutoField.normalizeDeg(Math.toDegrees(follower.pose().heading() - targetHeading));
                    if (Math.abs(t.bearingDeg) <= AIM_TOLERANCE_DEG) {
                        if (++settled >= 3) {
                            break;
                        }
                    } else {
                        settled = 0;
                        if (Math.abs(headingErr) < AIM_TOLERANCE_DEG) {   // only re-aim on frames taken after the last turn
                            double want = follower.pose().heading() + Math.toRadians(t.bearingDeg);
                            double next = clampAim(want, planned);
                            if (next != want && Math.abs(AutoField.normalizeDeg(Math.toDegrees(next - targetHeading)))
                                    < AIM_TOLERANCE_DEG) {
                                log.row(now(), "EVENT", String.format("AIM LIMIT: %.0f deg from the planned heading, "
                                        + "bearing still %.1f", AutoField.AIM_MAX_YAW_DEG, t.bearingDeg));
                                break;                          // already at the limit: fire from here
                            }
                            targetHeading = next;
                            follower.hold(spot.withHeading(targetHeading));
                        }
                    }
                }
            }
            loopTick();
        }
        return last;
    }

    /** A heading (radians) limited to AutoField.AIM_MAX_YAW_DEG either side of the planned one. */
    private static double clampAim(double headingRad, double plannedRad) {
        double off = AutoField.normalizeDeg(Math.toDegrees(headingRad - plannedRad));
        if (Math.abs(off) <= AutoField.AIM_MAX_YAW_DEG) {
            return headingRad;
        }
        return plannedRad + Math.toRadians(Math.signum(off) * AutoField.AIM_MAX_YAW_DEG);
    }

    /**
     * The TeleOp pocket-by-pocket feed (ShotFeeder) while Pedro holds the aimed pose (or, without Pedro, the drive
     * stopped), until the park start (fallback: FALLBACK_FIRE_END_SEC).
     */
    private void fire(boolean preloads) {
        if (pedroDrives) {
            follower.hold(follower.pose());
        } else {
            robot.stopDrive();
        }
        double settleEnd = Math.min(now() + AUTO_FIRE_SETTLE_SEC, fireDeadline());
        while (opModeIsActive() && now() < settleEnd) {
            holdTick();
            step = "FIRE settle";
            loopTick();
        }
        ShotFeeder feeder = ShotFeeder.fromSettings(settings);
        BallCounter counter = BallCounter.forAuto();
        robot.setIntake(FEED_INTAKE_BOOST);
        double feedEnd = now() + AUTO_FEED_TIMEOUT_SEC;
        while (opModeIsActive() && now() < Math.min(feedEnd, fireDeadline()) && feeder.shots() < AUTO_FEED_POCKETS
                && feeder.state() != ShotFeeder.State.JAMMED
                && !(counter.balls() >= AUTO_PRELOADS && feeder.state() != ShotFeeder.State.FEEDING)) {
            holdTick();
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

    /**
     * Without Pedro (no Pedro at START, or the pose lost before the robot left the start), on the wheel encoders and the
     * camera. Both starts face the wall, so backing straight up leaves it. S1 with shoot ON: back up to the shot spot
     * (SHOT_WALL_GAP_IN from the start), camera aim, G417 check, fire the pre-loads; then back up to FALLBACK_LEAVE_IN
     * for LEAVE. Otherwise LEAVE only. No GARDEN, no PARK.
     */
    private void fallback() {
        pedroDrives = false;
        if (follower != null) {
            halt();
        }
        boolean shootHere = shoot && start == Start.CELL_WALL && !preloadsTried;
        step = "FALLBACK DELAY";
        while (opModeIsActive() && now() < delaySec) {
            loopTick();
        }
        if (shootHere) {
            robot.setLauncherVelocity(launchTps);               // after the delay: no long spin on a weak battery
            step = "FALLBACK TO SHOT";
            backOffTo(AutoField.SHOT_WALL_GAP_IN);
            volley(true);
        }
        step = "FALLBACK LEAVE";
        backOffTo(FALLBACK_LEAVE_IN);
        robot.setLauncherVelocity(0);
        robot.setIndexerPower(0);
        robot.setIntake(0);
        step = shootHere ? "FALLBACK DONE (shot + LEAVE)" : "FALLBACK DONE (LEAVE only)";
        log.row(now(), "EVENT", String.format("%s: %.1f in back from the start (wheel encoders)%s", step, backedIn(),
                backedIn() < FALLBACK_LEAVE_IN - 1 ? ": SHORT, LEAVE not sure" : ""));
    }

    /**
     * The pose is gone (Pedro stopped): no GARDEN, no park. Still at the start with the pre-loads untried (S1): the
     * fallback shot. Otherwise make LEAVE (team test plan: not touching a wall at 0:00) and stand still. Our routes only
     * come near a wall with the intake (start, shot spot, GARDEN corner, park slot), so backing up clears it: from the
     * start to FALLBACK_LEAVE_IN, elsewhere until the intake is CLEAR_WALL_IN off the wall in front of it (last trusted
     * pose).
     */
    private void afterPoseLost() {
        robot.setIndexerPower(0);
        robot.setIntake(0);
        if (!leftStart && !preloadsTried && shoot && start == Start.CELL_WALL) {
            fallback();
            return;
        }
        robot.setLauncherVelocity(0);
        double back = 0;
        if (!leftStart) {
            back = FALLBACK_LEAVE_IN - backedIn();              // never left: it only backed straight up from START
        } else if (lastGoodRed != null) {
            back = CLEAR_WALL_IN - frontGapIn(lastGoodRed);
        }
        back = Math.min(back, FALLBACK_LEAVE_IN);
        String moved = "";
        if (back > 0.5) {
            step = "POSE LOST: off the wall";
            moved = String.format(", backed up %.1f of %.1f in (wheel encoders)", backOff(back), back);
        }
        step = "POSE LOST: stopped";
        log.row(now(), "EVENT", "POSE LOST: no GARDEN, no PARK" + moved + ", stopped (last trusted pose "
                + lastGoodRed + ")");
    }

    /** The pose can't be trusted (see poseLost): Pedro stops for the rest of AUTO. */
    private void losePose() {
        if (!poseLostOnce) {
            log.row(now(), "EVENT", "POSE LOST " + here() + " (Pinpoint " + pinpointStatus() + "): Pedro stops");
        }
        poseLostOnce = true;
        pedroDrives = false;
        halt();
    }

    /** Without Pedro: backs straight up until the wheel encoders are this far back from START. */
    private void backOffTo(double inches) {
        backOff(inches - backedIn());
    }

    /**
     * Without Pedro: backs straight up (launcher first) this far from where the robot is now, on the wheel encoders, at
     * most BACK_OFF_MAX_SEC. Returns how far it got.
     */
    private double backOff(double inches) {
        double f0 = robot.forwardInches();
        double end = now() + BACK_OFF_MAX_SEC;
        while (opModeIsActive() && now() < end && f0 - robot.forwardInches() < inches) {
            robot.drive(-0.3, 0, 0, 1.0);
            loopTick();
        }
        robot.stopDrive();
        return f0 - robot.forwardInches();
    }

    /** How far the robot backed up since START, from the wheel encoders (Pinpoint-free). */
    private double backedIn() {
        return startForwardIn - robot.forwardInches();
    }

    /** Gap from the intake bumper (RED-frame pose) to the field wall straight ahead of it. */
    private static double frontGapIn(Pose red) {
        double ux = Math.cos(Math.toRadians(red.headingDeg)), uy = Math.sin(Math.toRadians(red.headingDeg));
        double fx = red.x + AutoField.HALF_LENGTH_IN * ux, fy = red.y + AutoField.HALF_LENGTH_IN * uy;
        double gap = Double.POSITIVE_INFINITY;
        if (Math.abs(ux) > 1e-6) {
            gap = Math.min(gap, ((ux > 0 ? AutoField.WALL_IN : -AutoField.WALL_IN) - fx) / ux);
        }
        if (Math.abs(uy) > 1e-6) {
            gap = Math.min(gap, ((uy > 0 ? AutoField.WALL_IN : -AutoField.WALL_IN) - fy) / uy);
        }
        return gap;
    }

    /**
     * Without Pedro: the TeleOp camera aim (HiveVision.aimTurnCw on the latest frame of our CELL, turning on the spot)
     * until it needs no turn on 3 new frames. No recent frame of our CELL: the drive stops. All turning since the aim
     * began ({@code imu0}, hub IMU, either sign) stays under AutoField.AIM_MAX_YAW_DEG - AIM_COAST_DEG so a robot corner can't
     * reach the wall.
     */
    private HiveVision.Target fallbackAim(double endSec, HiveVision.Target seenBefore, double imu0) {
        HiveVision.Cell ours = HiveVision.startingUpCell(hiveAlliance());
        HiveVision.Target last = seenBefore;
        int settled = 0;
        while (opModeIsActive() && now() < endSec) {
            boolean fresh = vision.update();
            HiveVision.Target t = vision.targetCell(hiveAlliance(), ours);
            lastTarget = t;
            if (t == null) {
                robot.stopDrive();
            } else {
                last = t;
                double turned = Math.abs(AutoField.normalizeDeg(robot.headingDeg() - imu0));
                double allowed = AutoField.AIM_MAX_YAW_DEG - AIM_COAST_DEG;
                if (turned >= allowed) {
                    log.row(now(), "EVENT", String.format("AIM LIMIT (no Pedro): turned %.0f deg, bearing still %.1f",
                            turned, t.bearingDeg));
                    break;
                }
                double turn = HiveVision.aimTurnCw(t.bearingDeg);
                if (allowed - turned < AIM_SLOW_ZONE_DEG) {      // near the stop: slowest turn, so a loop + the coast stay short
                    turn = Math.signum(turn) * Math.min(Math.abs(turn), AIM_MIN_TURN);
                }
                robot.drive(0, 0, turn, 1.0);
                if (fresh) {
                    noteAimFrame(t);
                    settled = turn == 0 ? settled + 1 : 0;
                    if (settled >= 3) {
                        break;
                    }
                }
            }
            loopTick();
        }
        robot.stopDrive();
        return last;
    }

    private void noteAimFrame(HiveVision.Target t) {
        aimHeights.add(t.heightIn());
        aimFloors.add(t.horizontalIn());
    }

    /** A loop of standing still: Pedro holds the pose (a lost pose stops it); without Pedro the drive stays off. */
    private void holdTick() {
        if (!pedroDrives) {
            return;
        }
        pedroUpdate();
        if (poseLost()) {
            losePose();
        }
    }

    /**
     * One Pedro step. While the Pinpoint's last reading was not READY (the debounce in poseLost()) only the Pinpoint is
     * read and the drive stays stopped: Pedro would compute its power from a frozen pose. Pedro steps again from the first
     * READY reading (Follower.pose() reads the localizer).
     */
    private void pedroUpdate() {
        if (!Double.isNaN(pinpointBadSince)) {
            follower.localizer.update();
            if (pinpointDev.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
                follower.drivetrain.stop();
                return;
            }
        }
        follower.update();
    }

    /** Aim and fire end by then: the park start, or without Pedro FALLBACK_FIRE_END_SEC (time left to back up for LEAVE). */
    private double fireDeadline() {
        return pedroDrives ? parkStart() : FALLBACK_FIRE_END_SEC;
    }

    /**
     * START: puts the start pose into the Pinpoint and checks it took (READY, at the start pose), retrying for up to
     * PINPOINT_START_WAIT_SEC. Returns null when Pedro can drive, else why not (then: FALLBACK).
     * SDK 12's Pinpoint driver takes a new reading only within 5000 mm of the last one it took (else it keeps the old one
     * and reports FAULT_BAD_READ), and setPose only writes the device. After a TeleOp (own frame) the start pose could be
     * refused for good: 13 Nov 21:47 BLUE x took the start x, y stayed 253 in off -> POSE LOST at 0.2 s, and the next two
     * AUTOs saw FAULT_BAD_READ at START until a power cycle (code review, 14 Nov). So the first reading after setPose is
     * taken unchecked (error detection NONE, as in INIT), then the checks go back on for the match.
     */
    private String startPedro() {
        String why = pedroProblem();
        if (follower == null || pinpointDev == null || !Constants.FORESIGHT_TUNED) {
            return why;
        }
        Pose startPose = AutoField.start(start).forAlliance(alliance());
        step = "START PINPOINT";
        ElapsedTime wait = new ElapsedTime();
        while (opModeIsActive()) {
            why = null;
            pinpointDev.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.NONE);
            follower.setPose(pedro(startPose));                 // no path: Pedro keeps the motors at 0
            follower.update();
            pinpointDev.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.LOCAL_TEST);
            for (int read = 0; read < 2 && why == null; read++) {   // twice in a row, checked again
                follower.update();
                Pose p = here();
                why = pedroProblem();
                if (why == null && !(p.distanceTo(startPose) <= START_POSE_TOL_IN
                        && Math.abs(AutoField.normalizeDeg(p.headingDeg - startPose.headingDeg)) <= START_POSE_TOL_DEG)) {
                    why = "Pinpoint pose " + p + " is not the start pose";
                }
            }
            if (why == null) {
                log.row(now(), "EVENT", String.format("Pinpoint READY at the start pose after %.2f s", wait.seconds()));
                return null;
            }
            if (wait.seconds() >= PINPOINT_START_WAIT_SEC) {
                return why;
            }
            loopTick();
        }
        return "stopped";
    }

    /**
     * INIT: a Pinpoint not READY for PINPOINT_RESET_AFTER_SEC gets a reset (position + IMU; the robot stands still in
     * INIT), again every PINPOINT_RESET_EVERY_SEC while it lasts. START sets the pose anyway.
     */
    private void recoverPinpointInInit() {
        if (pinpointDev == null || follower == null) {
            return;                                             // no follower.update(): the status is never refreshed
        }
        double t = clock.seconds();
        if (pinpointDev.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
            pinpointFaultSince = Double.NaN;
            return;
        }
        if (Double.isNaN(pinpointFaultSince)) {
            pinpointFaultSince = t;
        } else if (t - pinpointFaultSince >= PINPOINT_RESET_AFTER_SEC && t - lastPinpointReset >= PINPOINT_RESET_EVERY_SEC) {
            pinpointDev.resetPosAndIMU();
            lastPinpointReset = t;
            pinpointResets++;
        }
    }

    private String pinpointStatus() {
        return pinpointDev == null ? "none" : String.valueOf(pinpointDev.getDeviceStatus());
    }

    /** What AUTO does without Pedro (INIT screen and log). */
    private String fallbackPlan() {
        return shoot && start == Start.CELL_WALL
                ? "FALLBACK: back up 2 in, camera aim, fire the pre-loads, back up (LEAVE); no GARDEN, no PARK"
                : "FALLBACK: back up off the wall (LEAVE only)";
    }

    /** INIT: our CELL on the new frame, for the UP / DOWN line (median of the last CellCheck.FRAMES frames). */
    private void noteInitCell() {
        initCell = vision.targetCell(hiveAlliance(), HiveVision.startingUpCell(hiveAlliance()));
        if (initCell == null) {
            initHeights.clear();
            return;
        }
        initHeights.add(initCell.heightIn());
        if (initHeights.size() > CellCheck.FRAMES) {
            initHeights.remove(0);
        }
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

    /**
     * The pose can't be trusted, so Pedro must not drive on it (reviews, 14 Nov):
     * - NaN, or the robot centre farther out than it can get (a Pinpoint that restarts, or an I2C read of zeros, reads
     *   Pedro (0, 0) = the field corner (-72, -72));
     * - the Pinpoint not READY for PINPOINT_LOST_SEC: on a bad read the driver keeps the last pose and Pedro reads it
     *   without looking at the status, so it would drive on a frozen pose. One bad read is normal and recovers on the
     *   next loop; until then the drive is stopped (pedroUpdate() then only reads the Pinpoint);
     * - between READY readings, a jump bigger than the robot can move in that time (checks start at the START pose).
     * Once lost, always lost.
     */
    private boolean poseLost() {
        if (poseLostOnce || poseJumped || pinpointLost) {
            return true;
        }
        Pose p = here();
        double t = now();
        double reach = AutoField.WALL_IN - AutoField.HALF_WIDTH_IN + 6;
        if (Double.isNaN(p.x) || Double.isNaN(p.y) || Double.isNaN(p.headingDeg)
                || Math.abs(p.x) > reach || Math.abs(p.y) > reach) {
            return true;
        }
        if (pinpointDev != null && pinpointDev.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            if (Double.isNaN(pinpointBadSince)) {
                pinpointBadSince = t;
            }
            if (t - pinpointBadSince >= PINPOINT_LOST_SEC) {
                pinpointLost = true;
                return true;
            }
            follower.drivetrain.stop();                         // never drive on a pose that may be frozen
            return false;
        }
        pinpointBadSince = Double.NaN;
        if (lastPose != null) {
            double dt = Math.max(0, t - lastPoseSec);           // since the last READY reading (spans a short freeze)
            double moved = p.distanceTo(lastPose);
            double turned = Math.abs(AutoField.normalizeDeg(p.headingDeg - lastPose.headingDeg));
            if (moved > POSE_JUMP_IN + 1.5 * AutoPlanner.TOP_SPEED_IN_PER_SEC * dt
                    || turned > POSE_JUMP_DEG + HEADING_MAX_RATE_DPS * dt) {
                poseJumped = true;
                log.row(t, "EVENT", String.format("POSE JUMP %.1f in / %.0f deg in %.2f s: from %s to %s", moved, turned,
                        dt, lastPose, p));
                return true;
            }
        }
        if (lastPose != null || p.distanceTo(AutoField.start(start).forAlliance(alliance())) <= POSE_ARM_IN) {
            lastPose = p;
            lastPoseSec = t;
        }
        return false;
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
        if (pedroDrives && poseSet) {
            if (poseLost()) {
                losePose();                                     // wherever it shows first: Pedro stops at once
            } else {
                Pose r = redHere();
                lastGoodRed = r;
                BioBuzzMatchState.recordAutoHeading(r.headingDeg);   // for TeleOp's field-centric start
                if (r.distanceTo(AutoField.start(start)) >= FALLBACK_LEAVE_IN) {
                    leftStart = true;
                }
            }
        }
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
        if (pedroDrives) {
            telemetry.addData("Time", "%.1f s   (park starts by %.1f s)", t, parkStart());
        } else {
            telemetry.addData("Time", "%.1f s   (Pedro off: no GARDEN, no park)", t);
        }
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
        String why = pedroProblem();
        if (why != null) {
            telemetry.addLine("!! PEDRO NOT READY (" + why + ") -> " + fallbackPlan());
        }
        double volts = robot.battery.getVoltage();
        if (volts < LOW_BATTERY_V) {
            telemetry.addLine(String.format("!! BATTERY %.1f V: put in a charged one (13 Nov: from ~12 V the hubs dropped out"
                    + " and the Pinpoint faulted)", volts));
        }
        double cellHeight = CellCheck.median(initHeights);
        boolean cellUp = CellCheck.up(initHeights);
        if (shoot && initCell != null && !cellUp) {
            telemetry.addLine(String.format("!! OUR CELL %s looks DOWN (%.0f in): AUTO will not shoot (G417). Practice: set"
                    + " the HIVE to the match start", initCell.cell, cellHeight));
        }
        telemetry.addData("Start (D-pad L/R)", start.label);
        telemetry.addData("Park slot (Square/X)", slot.label);
        telemetry.addLine(">> Tell your partner: we park in slot " + slot + ", they take slot " + slot.other());
        telemetry.addData("Shoot pre-loads (Cross/A)", shoot ? String.format("ON (%.0f t/s)", launchTps) : "OFF");
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
        if (shoot && garden && AutoPlanner.fits(t, at, AutoPlanner.toGarden(), GARDEN_MIN_DWELL_SEC, slot, parkBySec)) {
            t += AutoPlanner.estimateSec(at, AutoPlanner.toGarden());
            at = AutoPlanner.end(at, AutoPlanner.toGarden());
            t = Math.min(t + GARDEN_DWELL_SEC, AutoPlanner.parkStartSec(at, slot, parkBySec));
            telemetry.addData("Plan", "GARDEN done ~%.0f s (intake on, %.0f in back and forth at the corner until then)",
                    t, AutoField.GARDEN_WIGGLE_IN);
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

        telemetry.addData("Pedro", why == null ? "ready" : "NOT READY: " + why + (pedroError != null ? " (" + pedroError
                + ")" : "") + " -> " + fallbackPlan());
        telemetry.addData("Pinpoint", "%s%s", pinpointStatus(),
                pinpointResets > 0 ? String.format("  (reset %d x in INIT: keep the robot still)", pinpointResets) : "");
        telemetry.addData("Battery", "%.1f V%s", volts, volts < LOW_BATTERY_V ? "  LOW" : "");
        if (vision == null) {
            telemetry.addData("Camera", "NOT FOUND: %s (fires pre-loads unaimed, no stretch)", visionError);
        } else if (initCell == null) {
            telemetry.addData("Camera", "our CELL %s not in view", HiveVision.startingUpCell(hiveAlliance()));
        } else {
            telemetry.addData("Camera", "our CELL %s %s: height %.0f in, %d tags", initCell.cell,
                    cellUp ? "UP" : "DOWN", cellHeight, initCell.tagCount);
        }
        telemetry.addLine("Place: " + (start == Start.CELL_WALL
                ? "intake against the " + (alliance() == Alliance.RED ? "AUDIENCE" : "REAR")
                + " wall, centred on our up CELL, square to the wall"
                : "intake against our ALLIANCE wall, centred on the middle line, square to the wall"));
        telemetry.update();
    }
}

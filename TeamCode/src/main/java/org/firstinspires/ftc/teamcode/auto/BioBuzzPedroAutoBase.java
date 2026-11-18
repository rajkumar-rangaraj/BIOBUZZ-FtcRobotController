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
import org.firstinspires.ftc.teamcode.biobuzz.IntakeGuard;
import org.firstinspires.ftc.teamcode.biobuzz.PinpointInitCheck;
import org.firstinspires.ftc.teamcode.biobuzz.QueueFeeder;
import org.firstinspires.ftc.teamcode.biobuzz.TeleOpSettings;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

/*
 * BIOBUZZ AUTO with Pedro Pathing (plan and field places: AutoPlanner / AutoField, AUTO.md).
 *
 *   1. SHOT     drive to the straight-on spot in front of our up CELL, camera aim, fire the 4 pre-loads (indexer v2:
 *               one queue volley, QueueFeeder)
 *   2. GARDEN   only when the pre-loads are gone (G407: max 4 balls; the volley ended EMPTY): sweep our 4 GARDEN POLLEN
 *               with the intake, then 2 in back and forth at the corner while the intake keeps running (ends early
 *               once the queue is FULL)
 *   3. STRETCH  back to the spot, fire the GARDEN POLLEN (only if the camera sees the CELL in front still UP)
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
    /** Stretch only when at least this much time is left for aim + settle + a volley at the spot (indexer v2: 4 -> 2 s). */
    static final double STRETCH_MIN_FIRE_SEC = 2.0;
    /**
     * A volley's feed ends this long after the gate starts opening (or at the park start). The pre-loads must prove the
     * queue empty for the GARDEN (G407): the volley ends on QueueFeeder's EMPTY rule (both beams clear while the queue
     * runs). Indexer v2: 4 balls leave in ~1.5 s (gate 0.15 s, ~0.2 s per ball at QUEUE_FIRE_POWER, ~0.6 s EMPTY check),
     * so 2.5 s leaves room for a dip hold or one unjam (the windmill needed up to 12 s for 12 pockets).
     */
    static final double VOLLEY_FEED_SEC = 2.5;
    /** The flywheel gets this long to be ready (the gate stays shut meanwhile) before the volley is dropped. */
    static final double VOLLEY_READY_WAIT_SEC = 4.0;
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
    /** INIT warning: Tune's queue fire power (AUTO fires with it) outside this range. */
    static final double QUEUE_POWER_WARN_MIN = 0.4, QUEUE_POWER_WARN_MAX = 0.9;

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
    private final List<Double> aimHeights = new ArrayList<>();  // the CELL in front: its height on each frame of the last aim
    private final List<Double> aimFloors = new ArrayList<>();   // ... and its floor distance (log only)
    private final List<Double> initHeights = new ArrayList<>(); // the CELL in front: its height on the latest INIT frames
    private HiveVision.Target initCell;                         // the CELL in front on the latest INIT frame
    private double pinpointFaultSince = Double.NaN;
    private double lastPinpointReset = -1e9;
    private int pinpointResets = 0;
    private final PinpointInitCheck gyroCheck = PinpointInitCheck.standard();
    private int gyroRecalibrations = 0;                         // INIT: the Pinpoint gyro drifted and was recalibrated
    private HiveVision.Target otherCell;                        // INIT: the other ALLIANCE's CELL in front (wrong AUTO?)
    private double handoffHeadingRed = Double.NaN;              // without Pedro: TeleOp's heading, turned with ...
    private double handoffImuPrev;                              // ... the hub IMU (its last reading)
    private QueueFeeder queue;                                  // indexer v2: updated in every loopTick (loads, fires)
    private boolean queueFire = false;                          // fire() wants the volley
    private double intakeWanted = 0;                            // intake power asked for (before IntakeGuard: G407, unjam)
    private final IntakeGuard intakeGuard = IntakeGuard.standard();
    private String lastFaultLogged = null;
    private final BallCounter dipCounter = BallCounter.forQueue();   // flywheel dips: cross-check, logged only

    @Override
    public void runOpMode() {
        robot = new BioBuzzRobot(hardwareMap, true);            // first: Pedro below takes over the drive motors.
                                                                // MANUAL bulk cache: cleared once per loop (loopTick, INIT)
        queue = QueueFeeder.fromSettings(settings);
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
        robot.reapplyBulkCaching();                             // MANUAL again, whatever Pedro set
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
            robot.clearBulkCache();
            readInitButtons();
            if (follower != null) {
                follower.update();                              // idle: motors 0, keeps the Pinpoint status fresh
                checkGyroInInit();
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
                "height_in", "floor_in", "queue", "gate_open", "beam_staged", "beam_full", "launcher1_tps",
                "launcher2_tps");
        log.row("CONFIG", "start=" + start, "slot=" + slot, "shoot=" + shoot, "garden=" + garden, "stretch=" + stretch,
                "delay=" + delaySec, "park_by=" + parkBySec, "camera=" + (vision != null),
                String.format("aim_trim=%.1f", settings.aimTrimDeg), String.format("battery=%.1f", robot.battery.getVoltage()),
                "pinpoint=" + pinpointStatus(), "pinpoint_resets=" + pinpointResets,
                String.format("launcher=%.0f", launchTps), "gyro_recal=" + gyroRecalibrations,
                String.format("queue_fire_power=%.2f", settings.queueFirePower),
                "gate_calibrated=" + BioBuzzRobot.gateCalibrated(),
                "other_cell=" + (otherCell == null ? "none" : otherCell.cell));
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
            intake(0);
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
        intake(0);
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
            if (!waitPoseOk()) {
                afterPoseLost();
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
            if (!waitPoseOk()) {                                // never start a path from a bad pose
                return false;
            }
            Pose target = leg.to.forAlliance(alliance());
            follower.follow(path(target, leg.speed));
            ElapsedTime legTimer = new ElapsedTime(), still = new ElapsedTime();
            Pose mark = here();
            double timeout = AutoPlanner.legSec(redHere(), leg) * 2 + 2;
            log.row(now(), "EVENT", "LEG " + leg.label + " -> " + target);
            boolean refollow = false;                           // a Pinpoint fault on this leg (see below)
            while (true) {
                if (!opModeIsActive()) {
                    return false;
                }
                pedroUpdate();
                intake(leg.action == AutoPlanner.Action.INTAKE ? GARDEN_INTAKE_POWER : 0);
                if (poseLost()) {
                    losePose();
                    return false;
                }
                // the deadlines first: a Pinpoint that keeps faulting (each fault shorter than PINPOINT_LOST_SEC) must
                // not hold a leg past them (here() is the last trusted pose during a fault)
                if (!parking && now() >= parkStart()) {
                    log.row(now(), "EVENT", "PARK TIME during " + leg.label);
                    halt();
                    return false;
                }
                if (parking && now() >= AutoPlanner.MATCH_SEC - 0.3) {
                    halt();
                    return false;
                }
                if (pinpointFault()) {                          // drive stopped until it reads again (see poseLost)
                    refollow = true;
                    loopTick();
                    continue;
                }
                if (refollow) {
                    // Pedro took the fault's first (bad) reading and may count the path as done: the leg starts again
                    // from where the robot is, and the blocked timer too (the stop was ours)
                    refollow = false;
                    follower.follow(path(target, leg.speed));
                    mark = here();
                    still.reset();
                    loopTick();
                    continue;
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
        boolean full = false;
        double deepest = redHere().x;                           // closest to the RED wall (wall contact, for calibration)
        while (opModeIsActive() && now() < endSec && !full) {
            if (!waitPoseOk()) {
                return;
            }
            // out: GARDEN_WIGGLE_IN back from where the wall (or a POLLEN) stopped the robot; in: push past the wall
            for (AutoPlanner.Leg leg : AutoPlanner.gardenWiggle(redHere())) {
                if (!waitPoseOk()) {
                    return;
                }
                follower.follow(path(leg.to.forAlliance(alliance()), leg.speed));
                ElapsedTime move = new ElapsedTime();
                step = leg.label;
                while (opModeIsActive() && now() < endSec && move.seconds() < GARDEN_WIGGLE_MOVE_SEC
                        && (move.seconds() < 0.2 || follower.isBusy())) {
                    pedroUpdate();
                    intake(GARDEN_INTAKE_POWER);
                    if (poseLost()) {
                        losePose();
                        return;
                    }
                    deepest = Math.min(deepest, redHere().x);
                    loopTick();
                    if (queue.isFull()) {                       // 4 held: nothing more to take (G407)
                        full = true;
                        break;
                    }
                }
                moves++;
                if (!opModeIsActive() || now() >= endSec || full) {
                    break;
                }
            }
        }
        halt();
        log.row(now(), "EVENT", String.format("GARDEN WIGGLE done%s: %d moves, deepest x %.1f (bumper on the wall at "
                + "x %.1f), at %s", full ? " early (queue FULL)" : "", moves, deepest, AutoField.GARDEN_CORNER_X, here()));
    }

    /** Camera aim (Pedro holds the spot and turns), CELL-up check, then fire. Sets preloadsGone after the first one. */
    private void volley(boolean preloads) {
        if (preloads) {
            preloadsTried = true;
        }
        step = preloads ? "AIM (pre-loads)" : "AIM (GARDEN POLLEN)";
        HiveVision.Target seen = aim(Math.min(now() + AUTO_AIM_TIMEOUT_SEC, fireDeadline()));
        if (!preloads && seen != null && frontCell() == null) {
            // the GARDEN POLLEN go only into a CELL in view now: an UP frame from earlier in the aim may be from before a TIP
            log.row(now(), "EVENT", "CELL in front out of view at the end of the aim: GARDEN POLLEN kept");
            return;
        }
        if (cellAllowsFire(seen, preloads)) {
            fire(preloads);
        }
    }

    /**
     * G417 check on the CELL in front from the last aim (median height of its last frames, CellCheck). No CELL in front
     * seen: the pre-loads still go on the planned heading, GARDEN POLLEN are kept. The speed stays AUTO_LAUNCH_VELOCITY:
     * every AUTO shot is from the one shot spot, where the camera distance only adds noise (logged for calibration).
     */
    private boolean cellAllowsFire(HiveVision.Target seen, boolean preloads) {
        if (seen == null) {
            log.row(now(), "EVENT", preloads ? "no CELL in front in view: firing on the planned heading"
                    : "no CELL in front in view: GARDEN POLLEN kept");
            return preloads;
        }
        double h = CellCheck.median(aimHeights);
        boolean up = CellCheck.up(aimHeights);
        HiveVision.Cell manual = HiveVision.startingUpCell(hiveAlliance());
        log.row(now(), "EVENT", String.format("CELL in front %s %s: height %.1f in (median of %d frames, last %.1f), floor"
                + " %.1f in, bearing %.1f, launcher %.0f t/s%s", seen.cell, up ? "UP" : "DOWN", h,
                Math.min(aimHeights.size(), CellCheck.FRAMES), seen.heightIn(), CellCheck.median(aimFloors),
                seen.bearingDeg, launchTps, seen.cell == manual ? ""
                        : " (the manual's start-up CELL is " + manual + ": HIVE turned around?)"));
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
        if (!pedroDrives || !waitPoseOk()) {
            return fallbackAim(endSec, null, imu0);
        }
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
            if (pinpointFault()) {                              // no turn on a heading that may be a bad read
                loopTick();
                continue;
            }
            if (vision.update()) {
                HiveVision.Target t = frontCell();
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
     * One queue volley (QueueFeeder, updated in loopTick) while Pedro holds the aimed pose (or, without Pedro, the drive
     * stopped): the gate opens once the flywheel is ready, the queue runs until EMPTY (both beams clear), JAMMED,
     * VOLLEY_FEED_SEC after the gate opened, or the park start (fallback: FALLBACK_FIRE_END_SEC). The gate closes in
     * the next loopTicks (queue stopped, beam_staged clear) while the robot moves on.
     */
    private void fire(boolean preloads) {
        if (pedroDrives && waitPoseOk()) {
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
        if (!BioBuzzRobot.gateCalibrated()) {
            log.row(now(), "EVENT", "FIRE skipped: gate not calibrated (GATE_OPEN / GATE_CLOSED)"
                    + (preloads ? ", pre-loads kept (no GARDEN)" : ""));
            return;
        }
        int dipsBefore = dipCounter.balls();
        intake(FEED_INTAKE_BOOST);                              // helps ball 4 into the lane while the queue runs
        queueFire = true;
        double start = now();
        double openedAt = Double.NaN;
        String end = "park time";
        while (opModeIsActive() && now() < fireDeadline()) {
            holdTick();
            step = String.format("FIRE %s, %d balls (beam), %d (dips)", queue.state(), queue.volleyBalls(),
                    dipCounter.balls() - dipsBefore);
            loopTick();                                         // the queue steps here
            if (Double.isNaN(openedAt) && (queue.firing() || queue.gateOpen())) {
                openedAt = now();
            }
            if (queue.emptied()) {
                end = "EMPTY";
                break;
            }
            if (queue.state() == QueueFeeder.State.JAMMED) {
                end = "JAMMED";
                break;
            }
            if (Double.isNaN(openedAt) ? now() - start > VOLLEY_READY_WAIT_SEC : now() - openedAt > VOLLEY_FEED_SEC) {
                end = Double.isNaN(openedAt) ? "flywheel never ready" : "feed time up";
                break;
            }
        }
        queueFire = false;                                      // the queue stops; the gate closes when safe
        intake(0);
        if (preloads) {
            // G407: only collect more once the queue is proven empty (the EMPTY rule, not a ball count)
            preloadsGone = queue.emptied();
        }
        log.row(now(), "EVENT", String.format("FIRE done (%s): %d balls (beam), %d (flywheel dips), %d jams, %.1f s%s",
                end, queue.volleyBalls(), dipCounter.balls() - dipsBefore, queue.jams(), now() - start,
                preloads ? (preloadsGone ? ", queue empty" : ", balls may be left (no GARDEN)") : ""));
    }

    /**
     * Intake power asked for, through IntakeGuard (applied now and again in every loopTick): never pull a 5th ball in
     * while the queue is FULL (G407 push-back, not while it fires), and the pair-jam unjam while pulling in.
     */
    private void intake(double power) {
        intakeWanted = power;
        applyIntake(now(), robot.lastStagedBroken, robot.lastFullBroken);
    }

    private void applyIntake(double t, boolean staged, boolean fullBeam) {
        if (queue == null) {
            robot.setIntake(intakeWanted > 0 ? 0 : intakeWanted);
            return;
        }
        double amps = intakeGuard.currentDue(t, intakeWanted) ? robot.intakeCurrentAmps() : Double.NaN;
        robot.setIntake(intakeGuard.update(t, intakeWanted, queue.isFull(), queue.firing(), amps, staged, fullBeam));
        if (intakeGuard.event() != null && log != null) {
            log.row(t, "EVENT", "INTAKE " + intakeGuard.event());
        }
    }

    /**
     * Indexer v2, once per loop (from loopTick): flywheel READY decision (also the two-encoder check), the queue
     * (loading while the intake pulls in, the volley while fire() wants it), the gate, and the flywheel dip cross-check.
     */
    private void queueTick(double t) {
        boolean ready = robot.updateLauncherReady();
        int ticks = robot.indexerTicks();
        boolean staged = robot.stagedBroken();
        boolean fullBeam = robot.fullBroken();
        robot.setIndexerPower(queue.update(t, queueFire, true, intakeWanted > 0.05, staged, fullBeam, ready,
                robot.launcherDipTooDeep(), ticks));
        robot.setGateOpen(queue.gateOpen());
        applyIntake(t, staged, fullBeam);                       // push-back / unjam pulses are timed per loop
        if (queue.event() != null && log != null) {
            log.row(t, "EVENT", "QUEUE " + queue.event(), "ticks", ticks, "launcher", robot.launcherVelocity());
        }
        if (robot.launcherFault() != null && !robot.launcherFault().equals(lastFaultLogged) && log != null) {
            log.row(t, "EVENT", "LAUNCHER STOPPED: " + robot.launcherFault());
        }
        lastFaultLogged = robot.launcherFault();
        if (dipCounter.update(t, queue.firing(), robot.isLauncherOn() ? robot.launcherTarget() : 0,
                robot.launcherVelocity()) && log != null) {
            log.row(t, "EVENT", "BALL (flywheel dip) " + dipCounter.balls(), "launcher", robot.launcherVelocity());
        }
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
        handOffHeadingToImu();
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
        intake(0);
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
        intake(0);
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
            log.row(now(), "EVENT", "POSE LOST " + rawHere() + " (Pinpoint " + pinpointStatus() + "; last trusted "
                    + lastGoodRed + ", RED frame): Pedro stops");
            handOffHeadingToImu();
        }
        poseLostOnce = true;
        pedroDrives = false;
        halt();
    }

    /**
     * Rides out a short Pinpoint fault (poseLost: not READY for less than PINPOINT_LOST_SEC, drive stopped) before a path
     * or a hold starts from the pose. False when the pose is lost (losePose done).
     */
    private boolean waitPoseOk() {
        while (opModeIsActive() && pinpointFault()) {
            pedroUpdate();
            if (poseLost()) {
                losePose();
                return false;
            }
            loopTick();
        }
        if (poseLost()) {
            losePose();
            return false;
        }
        return true;
    }

    /** The Pinpoint's last reading was not READY: a short fault is being ridden out (see poseLost). */
    private boolean pinpointFault() {
        return !Double.isNaN(pinpointBadSince);
    }

    /** Pedro stops (or never started): TeleOp's heading goes on from the last trusted one, turned with the hub IMU. */
    private void handOffHeadingToImu() {
        handoffHeadingRed = lastGoodRed != null ? lastGoodRed.headingDeg : AutoField.start(start).headingDeg;
        handoffImuPrev = robot.headingDeg();
        BioBuzzMatchState.recordAutoHeading(handoffHeadingRed);
    }

    /** Without Pedro: the hub IMU's turn, added up loop by loop (its yaw flips by 180 at +-90 deg), for TeleOp. */
    private void followHeadingOnImu() {
        double imu = robot.headingDeg();
        double step = AutoField.normalizeDeg(imu - handoffImuPrev);
        if (Math.abs(step) > 90) {
            step = AutoField.normalizeDeg(step + 180);          // a yaw flip, not a turn (as PinpointInitCheck)
        }
        handoffImuPrev = imu;
        handoffHeadingRed = AutoField.normalizeDeg(handoffHeadingRed + step);
        BioBuzzMatchState.recordAutoHeading(handoffHeadingRed);
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
     * Without Pedro: the TeleOp camera aim (HiveVision.aimTurnCw on the latest frame of the CELL in front, turning on the
     * spot) until it needs no turn on 3 new frames. No recent frame of it: the drive stops. All turning since the aim
     * began ({@code imu0}, hub IMU, either sign) stays under AutoField.AIM_MAX_YAW_DEG - AIM_COAST_DEG so a robot corner can't
     * reach the wall.
     */
    private HiveVision.Target fallbackAim(double endSec, HiveVision.Target seenBefore, double imu0) {
        HiveVision.Target last = seenBefore;
        int settled = 0;
        while (opModeIsActive() && now() < endSec) {
            boolean fresh = vision.update();
            HiveVision.Target t = frontCell();
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
        if (isStopRequested()) {                                // AUTO is ending: the I2C reads are zeros now
            follower.drivetrain.stop();
            return;
        }
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

    /**
     * INIT: a Pinpoint gyro calibrated while the robot moved (Pedro recalibrates it at INIT) drifts all match and turns
     * every path; TeleOp's check caught that in 2 of 100 INITs (logs to 14 Nov). Its heading is compared with the hub
     * IMU's while the robot stands still (PinpointInitCheck, as in TeleOp) and recalibrated when it drifts.
     */
    private void checkGyroInInit() {
        if (pinpointDev == null) {
            return;
        }
        boolean ok = pinpointDev.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
        if (gyroCheck.update(ok, Math.toDegrees(follower.pose().heading()), robot.headingDeg())) {
            pinpointDev.resetPosAndIMU();                       // START sets the pose anyway
            gyroRecalibrations++;
        }
    }

    /** What AUTO does without Pedro (INIT screen and log). */
    private String fallbackPlan() {
        return shoot && start == Start.CELL_WALL
                ? "FALLBACK: back up 2 in, camera aim, fire the pre-loads, back up (LEAVE); no GARDEN, no PARK"
                : "FALLBACK: back up off the wall (LEAVE only)";
    }

    /**
     * INIT: the CELL in front on the new frame, for the UP / DOWN line (median of the last CellCheck.FRAMES frames). Only
     * at S1: from S2 the camera looks at the HIVE side-on and both its CELLs are about equally near (one is always down).
     */
    private void noteInitCell() {
        HiveVision.Target t = start == Start.CELL_WALL ? frontCell() : null;
        HiveVision.Target other = start == Start.CELL_WALL ? vision.nearestCell(otherHiveAlliance()) : null;
        otherCell = other != null && CellCheck.front(other.horizontalIn())
                && (t == null || other.horizontalIn() < t.horizontalIn()) ? other : null;
        if (t == null || (initCell != null && t.cell != initCell.cell)) {
            initHeights.clear();                                // another CELL: don't mix their heights
        }
        initCell = t;
        if (t == null) {
            return;
        }
        initHeights.add(t.heightIn());
        if (initHeights.size() > CellCheck.FRAMES) {
            initHeights.remove(0);
        }
    }

    /**
     * Our ALLIANCE's CELL in front of the robot at S1 / the shot spot: the nearest one, if within
     * CellCheck.FRONT_MAX_FLOOR_IN, whatever its name (null otherwise). That's the CELL AUTO shoots at; the far CELL of the
     * HIVE (55-69 in) never counts.
     */
    private HiveVision.Target frontCell() {
        HiveVision.Target t = vision.nearestCell(hiveAlliance());
        return t != null && CellCheck.front(t.horizontalIn()) ? t : null;
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

    /**
     * Where we are (field frame). During a short Pinpoint fault (see poseLost) its reading may be all zeros: then the last
     * trusted pose, so nothing (park time, GARDEN fit, wiggle, paths, holds) is planned from a bad read.
     */
    private Pose here() {
        if (pinpointFault() && lastPose != null) {
            return lastPose;
        }
        return rawHere();
    }

    /** The pose as Pedro last read it from the Pinpoint (field frame). */
    private Pose rawHere() {
        com.pedropathing.math.Pose p = follower.pose();
        return new Pose(p.x() - 72, p.y() - 72, Math.toDegrees(p.heading()));
    }

    /** Where we are, in the RED frame the planner uses. */
    private Pose redHere() {
        return here().forAlliance(alliance());
    }

    /**
     * The pose can't be trusted, so Pedro must not drive on it (reviews, 14 Nov):
     * - the Pinpoint not READY for PINPOINT_LOST_SEC. Checked first: a failed I2C read is all zeros (NOT_READY at Pedro
     *   (0, 0), the field corner, a position the SDK takes), and on a bad read the driver keeps the last pose, which Pedro
     *   reads without looking at the status. One bad read recovers on the next loop; until then the drive is stopped
     *   (pedroUpdate() only reads the Pinpoint) and here() is the last trusted pose. 14 Nov 12:25 BLUE: one
     *   I2C_TIMEOUT_SDA_STUCK read at the GARDEN wall was POSE LOST at once (the reach check came first): no PARK;
     * - on a READY reading: NaN, or the robot centre farther out than it can get (a Pinpoint that restarted reads its
     *   origin, the field corner (-72, -72));
     * - between READY readings, a jump bigger than the robot can move in that time (checks start at the START pose).
     * Once lost, always lost. Not while AUTO is being stopped (STOP or the 30 s timer): the SDK then shuts the I2C device
     * down and its last reads are zeros (14 Nov logs: "POSE LOST (-72, -72)" right at the end of runs).
     */
    private boolean poseLost() {
        if (isStopRequested()) {
            follower.drivetrain.stop();                         // the SDK's last reads are zeros: no power from them
            return false;
        }
        if (poseLostOnce || poseJumped || pinpointLost) {
            return true;
        }
        double t = now();
        if (pinpointDev != null && pinpointDev.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            if (Double.isNaN(pinpointBadSince)) {
                pinpointBadSince = t;
                log.row(t, "EVENT", "PINPOINT " + pinpointStatus() + ": drive stopped (reads " + rawHere()
                        + "), last trusted pose " + lastPose);
            }
            if (t - pinpointBadSince >= PINPOINT_LOST_SEC) {
                pinpointLost = true;
                return true;
            }
            follower.drivetrain.stop();                         // never drive on a pose that may be frozen
            return false;
        }
        if (pinpointFault()) {
            log.row(t, "EVENT", String.format("PINPOINT READY again after %.2f s at %s", t - pinpointBadSince, rawHere()));
            pinpointBadSince = Double.NaN;
        }
        Pose p = rawHere();
        double reach = AutoField.WALL_IN - AutoField.HALF_WIDTH_IN + 6;
        if (Double.isNaN(p.x) || Double.isNaN(p.y) || Double.isNaN(p.headingDeg)
                || Math.abs(p.x) > reach || Math.abs(p.y) > reach) {
            return true;
        }
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

    private HiveVision.Alliance otherHiveAlliance() {
        return alliance() == Alliance.RED ? HiveVision.Alliance.BLUE : HiveVision.Alliance.RED;
    }

    private void closeVision() {
        if (vision != null) {
            vision.close();
            vision = null;
        }
    }

    private void loopTick() {
        double t = now();
        queueTick(t);                                           // indexer v2: queue, gate, READY (one decision per loop)
        Pose p = follower == null ? null : rawHere();
        if (pedroDrives && poseSet && !isStopRequested()) {     // at STOP the last reads are zeros: record nothing
            if (poseLost()) {
                losePose();                                     // wherever it shows first: Pedro stops at once
            } else if (!pinpointFault()) {                      // a READY reading that passed the checks
                Pose r = redHere();
                lastGoodRed = r;
                BioBuzzMatchState.recordAutoHeading(r.headingDeg);   // for TeleOp's field-centric start
                if (r.distanceTo(AutoField.start(start)) >= FALLBACK_LEAVE_IN) {
                    leftStart = true;
                }
            }
        } else if (!pedroDrives && !Double.isNaN(handoffHeadingRed) && !isStopRequested()) {
            followHeadingOnImu();                               // fallback / pose lost: the camera aim may turn
        }
        HiveVision.Target target = lastTarget;
        if (log != null) {
            log.row(t, step, p == null ? "" : p.x, p == null ? "" : p.y, p == null ? "" : p.headingDeg,
                    robot.launcherVelocity(), robot.launcherTarget(), robot.launcherReady(), robot.indexerTicks(),
                    robot.battery.getVoltage(), target == null ? "" : target.cell, target == null ? 0 : target.tagCount,
                    target == null ? "" : target.bearingDeg, target == null ? "" : target.heightIn(),
                    target == null ? "" : target.horizontalIn(), queue.state() + (queue.holding() ? " HOLD" : ""),
                    queue.gateOpen(), robot.lastStagedBroken, robot.lastFullBroken, robot.launcher1Velocity(),
                    robot.launcher2Velocity());
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
        telemetry.addData("Queue", "%s  gate %s  staged %s  full %s", queue.state(), queue.gateOpen() ? "OPEN" : "closed",
                robot.lastStagedBroken ? "BALL" : "clear", robot.lastFullBroken ? "BALL" : "clear");
        telemetry.addData("Intake", intakeGuard.status());
        String warning = robot.launcherWarning();
        if (warning != null) {
            telemetry.addData("WARNING", warning);
        }
        telemetry.addData("Log", log == null ? "-" : log.status());
        telemetry.update();
        robot.clearBulkCache();                                 // MANUAL bulk cache: the next loop reads fresh values
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
        if (otherCell != null) {
            telemetry.addLine(String.format("!! %s CELL IN FRONT (%.0f in): this is the %s AUTO - the right one?",
                    otherCell.cell, otherCell.horizontalIn(), alliance()));
        }
        if (!BioBuzzRobot.gateCalibrated()) {
            telemetry.addLine("!! QUEUE GATE NOT CALIBRATED (GATE_OPEN / GATE_CLOSED): AUTO can't fire, so no GARDEN");
        }
        if (settings.queueFirePower < QUEUE_POWER_WARN_MIN || settings.queueFirePower > QUEUE_POWER_WARN_MAX) {
            telemetry.addLine(String.format("!! QUEUE FIRE POWER FROM TUNE: %.2f (default %.2f): set it back in BioBuzz"
                    + " Tune", settings.queueFirePower, QUEUE_FIRE_POWER));
        }
        double cellHeight = CellCheck.median(initHeights);
        boolean cellUp = CellCheck.up(initHeights);
        if (shoot && initCell != null && !cellUp) {
            telemetry.addLine(String.format("!! THE CELL IN FRONT (%s) looks DOWN (%.0f in): AUTO will not shoot (G417)."
                    + " Practice: set the HIVE to the match start", initCell.cell, cellHeight));
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
            t += AutoPlanner.estimateSec(at, AutoPlanner.toShot(start)) + AUTO_AIM_TIMEOUT_SEC / 2 + AUTO_FIRE_SETTLE_SEC
                    + 1.5;                                      // one queue volley ~1-1.5 s (was +4 s for the windmill)
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
        telemetry.addData("Pinpoint", "%s%s%s", pinpointStatus(),
                pinpointResets > 0 ? String.format("  (reset %d x in INIT: keep the robot still)", pinpointResets) : "",
                gyroRecalibrations > 0 ? String.format("  (gyro drifted: recalibrated %d x - keep the robot STILL)",
                        gyroRecalibrations) : "");
        telemetry.addData("Battery", "%.1f V%s", volts, volts < LOW_BATTERY_V ? "  LOW" : "");
        telemetry.addData("Queue (from Tune)", "fire power %.2f, gate %s", settings.queueFirePower,
                BioBuzzRobot.gateCalibrated() ? "calibrated" : "NOT CALIBRATED");
        if (vision == null) {
            telemetry.addData("Camera", "NOT FOUND: %s (fires pre-loads unaimed, no stretch)", visionError);
        } else if (start != Start.CELL_WALL) {
            telemetry.addData("Camera", "OK (the CELL in front is checked at the shot spot)");
        } else if (initCell == null) {
            telemetry.addData("Camera", "no CELL in front in view (it fires straight ahead)");
        } else {
            telemetry.addData("Camera", "CELL in front: %s, height %.0f in (%s, %d tags)", cellUp ? "UP" : "DOWN",
                    cellHeight, initCell.cell, initCell.tagCount);
        }
        telemetry.addLine("Place: " + (start == Start.CELL_WALL
                ? "intake against the " + (alliance() == Alliance.RED ? "AUDIENCE" : "REAR")
                + " wall, centred on our up CELL, square to the wall"
                : "intake against our ALLIANCE wall, centred on the middle line, square to the wall"));
        telemetry.update();
    }
}

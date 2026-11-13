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
import org.firstinspires.ftc.teamcode.biobuzz.HiveCells;
import org.firstinspires.ftc.teamcode.biobuzz.HiveVision;
import org.firstinspires.ftc.teamcode.biobuzz.ShotFeeder;
import org.firstinspires.ftc.teamcode.biobuzz.ShotSpeed;
import org.firstinspires.ftc.teamcode.biobuzz.TeleOpSettings;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

/*
 * BIOBUZZ HIVE AUTO (team, 12 Nov): a copy of BioBuzzPedroAutoBase (BioBuzz AUTO, unchanged) with a second CELL.
 * Plan and places: HivePlanner / HiveAutoField, team card AUTO_HIVE.md.
 *
 *   1. NEAR     fire the 4 pre-loads into our up CELL (TIP 1: 20 points; the CELL starts with 3 NECTAR)
 *   2. GARDEN   sweep our 4 GARDEN POLLEN (only with the windmill empty, G407), short back-and-forth at the corner
 *   3. FAR      drive UNDER our HIVE to its other end and fire them into the CELL that came up (TIP 2?)
 *   3.5 FLOWER  (if the robot is empty and there is time) flower arm + intake at FLOWER 1, next to the far spot; the
 *               POLLEN are fired from the far spot again if there is still time (camera must see the CELL up)
 *   4. PARK     in LOADING ZONE slot A (default here: straight across from the far spot) or B
 *
 * G417 (only launch into an UP CELL): the camera watches both CELLs of our HIVE whenever it sees them (UP >= 42 in,
 * DOWN < 37 in, in between = not sure; 3 fresh frames in a row). The HIVE is a seesaw: a CELL counts as down when it,
 * or its other end coming up, is confirmed. Both tag stickers face the end that is up (CAD), so the near spot sees our
 * CELLs only before TIP 1 and the far spot only after it. NOT TIPPED (the near CELL still up or the far one down 1 s
 * after the last pre-load) -> the GARDEN POLLEN go back to the near CELL instead (camera must confirm it up). The far
 * CELL is fired at when the camera confirms it up (or confirms the TIP from there); when it sees nothing, only with
 * "blind far shot" (Circle/B, OFF by default: in good light nothing in view from the far spot means not tipped) and all
 * 4 pre-loads counted. The pre-loads always go all 4 (coach: "shoot all 4 balls first", as BioBuzz AUTO); every other
 * volley stops at once when its CELL is confirmed going down or, once confirmed up, drops out of view (it tipped).
 *
 * PARK ALWAYS WINS, as in BioBuzz AUTO: every step is cut short at the park start (the INIT "park by" time, earlier
 * when the drive to the slot is long). Blocked for 1 s: before the park the step is dropped; while parking it waits
 * and tries again until the end of AUTO.
 *
 * INIT (gamepad 1): D-pad L/R = start, Square/X = slot A/B, Cross/A = pre-loads, Triangle/Y = GARDEN,
 *                   Circle/B = blind far shot (without the camera), R3 = FLOWER (3.5), D-pad U/D = start delay,
 *                   LB/RB = park-by time.
 * Log: /sdcard/BioBuzzLogs/hiveauto_*.csv
 */
public abstract class BioBuzzHiveAutoBase extends LinearOpMode {

    protected abstract Alliance alliance();

    /** Later than BioBuzz AUTO's 24 s: the far spot is close to the LOADING ZONE (park ~3 s from there). */
    static final int PARK_BY_DEFAULT_SEC = 27;
    static final int PARK_BY_MIN_SEC = 18;
    static final int PARK_BY_MAX_SEC = 28;
    static final double BLOCKED_SEC = 1.0;
    static final double BLOCKED_WAIT_SEC = 0.75;
    static final double GARDEN_INTAKE_POWER = 1.0;
    /** GARDEN corner back-and-forth: up to this long (BioBuzz AUTO: 4 s), shortened so the far CELL keeps its time. */
    static final double GARDEN_DWELL_SEC = 2.0;
    static final double GARDEN_MIN_DWELL_SEC = 1.0;
    static final double GARDEN_WIGGLE_MOVE_SEC = 1.0;
    /** Feed time wanted at the far CELL: 4 balls (9 Nov: 2.3-4.8 s, median ~3.2 s). */
    static final double FAR_FEED_TARGET_SEC = 3.3;
    /**
     * Camera aim at the near CELL (BioBuzz AUTO: AUTO_AIM_TIMEOUT_SEC 3.5 s). It ends as soon as the robot is lined up
     * (9 Nov run 212355, CELL in view: lined up 0.3 s after reaching the spot); this cap only matters when the camera
     * can't see the CELL (the garage at night was too dark: 7 of 8 runs; the competition field is lit). The flywheel
     * needs ~1.4 s anyway.
     */
    static final double NEAR_AIM_SEC = 1.5;
    /** Camera aim at the far CELL; the flywheel spins up on the way. */
    static final double FAR_AIM_SEC = 1.0;
    /**
     * After the pre-loads, while the camera sees one of our CELLs, stay up to this long to see the TIP (the HIVE swings
     * over and lands on its damper). Leaves at once when it sees nothing.
     */
    static final double TIP_WATCH_SEC = 1.2;
    /** A CELL seen "still as at the start" only counts as NOT TIPPED this long after the last pre-load. */
    static final double TIP_SETTLE_SEC = 1.0;
    /** Fire at the far CELL only when at least this much time is left there (aim + settle + a few pockets). */
    static final double FAR_MIN_FIRE_SEC = 2.5;
    /** Plan B (no TIP): back to the near CELL only with this much time for aim + settle + a few pockets. */
    static final double NEAR_AGAIN_MIN_FIRE_SEC = 4.0;
    /**
     * Camera verdict on a CELL (G417). CAD: a CELL's tag sticker is 49.7 in high when UP, 35.7 in when tipped DOWN; the
     * camera read an UP CELL as 45.5-46.3 in (9 Nov), so a DOWN one should read ~32. UP at or above
     * CELL_UP_MIN_HEIGHT_IN, DOWN below CELL_DOWN_MAX_HEIGHT_IN, in between = not sure (decides nothing). A verdict counts
     * only when CELL_FRAMES fresh frames in a row agree, the last one at most CELL_FRESH_SEC old. CALIBRATE: height_in.
     */
    static final double CELL_UP_MIN_HEIGHT_IN = 42.0;
    static final double CELL_DOWN_MAX_HEIGHT_IN = 37.0;
    static final int CELL_FRAMES = 3;
    static final double CELL_FRESH_SEC = 0.5;
    /** While feeding, a CELL confirmed DOWN this recently stops the feed at once. */
    static final double CELL_STOP_FRESH_SEC = 0.3;
    /**
     * While feeding at a CELL the camera confirmed UP: not seen at all for this long = it tipped (both tag stickers turn
     * to face the other end of the field when a HIVE tips); stop. Longer than a launched ball crossing the picture.
     */
    static final double CELL_LOST_STOP_SEC = 0.6;
    static final double FALLBACK_LEAVE_IN = 8.0;
    /** FLOWER: arm down and settled before the intake goes in. */
    static final double FLOWER_ARM_SETTLE_SEC = 0.6;
    /** FLOWER: intake running at the stock stop, up to this long (4 POLLEN in the FLOWER); at least the minimum. */
    static final double FLOWER_DWELL_SEC = 3.0;
    static final double FLOWER_MIN_DWELL_SEC = 1.5;
    /** FLOWER: the creep ends when the robot stops moving this long (it touches the FLOWER). */
    static final double FLOWER_CONTACT_SEC = 0.4;
    static final double FLOWER_ARM_UP_SEC = 0.4;

    enum Tip { UNKNOWN, TIPPED, NOT_TIPPED }

    /** What the camera last said about one CELL. */
    private static final class CellSeen {
        int verdict;                                            // +1 UP, -1 DOWN, 0 not sure
        int run;                                                // fresh frames in a row with this verdict
        double atSec = -1;                                      // last time seen
        HiveVision.Target target;
    }

    private final java.util.Map<HiveVision.Cell, CellSeen> cells = new java.util.EnumMap<>(HiveVision.Cell.class);

    private Start start = Start.CELL_WALL;
    private Slot slot = Slot.A;                                 // A: straight across from the far spot (never passes B)
    private boolean shoot = true, garden = true, blindFar = false, flower = true;
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
    private int preloadBalls = 0;
    private double firstBallSec = -1, lastBallSec = -1;
    private boolean nearVolleyRunning = false;
    private boolean farFired = false;
    private boolean poseSet = false;                            // Pedro runs from the known start pose
    private Tip tip = Tip.UNKNOWN;
    private String tipWhy = "no CELL seen";
    private double tipSec = -1;                                 // last time the camera confirmed TIPPED
    private double lastSeenAnySec = -1;                         // last fresh frame with one of our CELLs in it

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

        log = new CsvLog("hiveauto_" + alliance(), "t", "step", "x_in", "y_in", "heading_deg", "launcher_tps",
                "launcher_target", "launcher_ready", "indexer_ticks", "battery_v", "cell", "tags", "bearing_deg",
                "height_in", "floor_in", "tip");
        log.row("CONFIG", "start=" + start, "slot=" + slot, "shoot=" + shoot, "garden=" + garden,
                "blind_far=" + blindFar, "flower=" + flower, "delay=" + delaySec, "park_by=" + parkBySec,
                "camera=" + (vision != null), "arm=" + BioBuzzRobot.armCalibrated(),
                String.format("aim_trim=%.1f", settings.aimTrimDeg));
        matchStart = clock.seconds();
        // TeleOp field-centric start: the robot's heading relative to its drivers (the RED frame's heading).
        BioBuzzMatchState.recordAutoHeading(AutoField.start(start).headingDeg);
        try {
            String why = pedroProblem();
            if (why != null) {
                log.row(0, "EVENT", "NO PEDRO: " + why + " -> fallback LEAVE only");
                fallbackLeave();
            } else {
                follower.setPose(pedro(AutoField.start(start).forAlliance(alliance())));
                poseSet = true;
                runPlan();
            }
            log.row("RESULT", "t=" + now(), "step=" + step, "tip=" + tip, "pose=" + (follower == null ? "?" : here()));
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
        // 1. NEAR: pre-loads into the CELL that starts up
        if (shoot) {
            robot.setLauncherVelocity(launchTps);
            step = "TO SHOT";
            if (drive(HivePlanner.toShot(start), false)) {
                volleyNear();
            }
        }
        boolean preloadsLeft = shoot && !preloadsGone;          // HIVE tipped before the first ball, or a short volley
        // 2. GARDEN (G407: only with the windmill empty)
        boolean gardenTried = false;                            // any GARDEN attempt: the robot may hold POLLEN now
        if (garden && preloadsGone && HivePlanner.fits(now(), redHere(), HivePlanner.toGarden(), GARDEN_MIN_DWELL_SEC,
                slot, parkBySec)) {
            robot.setLauncherVelocity(0);
            step = "GARDEN";
            gardenTried = true;
            if (drive(HivePlanner.toGarden(), false)) {
                gardenWiggle(HivePlanner.gardenDwellEnd(now(), redHere(), slot, parkBySec, GARDEN_MIN_DWELL_SEC,
                        GARDEN_DWELL_SEC, FAR_AIM_SEC + AUTO_FIRE_SETTLE_SEC + FAR_FEED_TARGET_SEC));
            }
            robot.setIntake(0);
        } else if (garden) {
            log.row(now(), "EVENT", !preloadsGone ? "GARDEN skipped: pre-loads not all fired (G407)" : "GARDEN skipped: no time");
        }
        boolean haveBalls = gardenTried || preloadsLeft;
        boolean empty = shoot && preloadsGone && !gardenTried;  // pre-loads not shot (Cross OFF) are still on board
        boolean flowerPlanned = flower && empty && BioBuzzRobot.armCalibrated();
        // 3. FAR (the HIVE tipped, or we don't know) or plan B: the near CELL again (it did not tip)
        if (tip == Tip.NOT_TIPPED) {
            log.row(now(), "EVENT", "NOT TIPPED (" + tipWhy + "): far CELL is down");
            if (haveBalls && HivePlanner.fits(now(), redHere(), HivePlanner.gardenToNearShot(), NEAR_AGAIN_MIN_FIRE_SEC,
                    slot, parkBySec)) {
                robot.setLauncherVelocity(launchTps);
                step = "BACK TO NEAR SHOT";
                if (drive(HivePlanner.gardenToNearShot(), false)) {
                    volleyCamera(HiveVision.startingUpCell(hiveAlliance()), AUTO_AIM_TIMEOUT_SEC, "NEAR AGAIN");
                }
            }
        } else if ((haveBalls || flowerPlanned) && HivePlanner.fits(now(), redHere(), HivePlanner.underHive(redHere()),
                haveBalls ? FAR_MIN_FIRE_SEC : 0, slot, parkBySec)) {
            if (haveBalls) {
                launchTps = ShotSpeed.standard().tableSpeed(HiveAutoField.FAR_SHOT_PLANNED_FLOOR_IN);
                robot.setLauncherVelocity(launchTps);           // spins up on the way
            }
            step = "UNDER THE HIVE";
            if (drive(HivePlanner.underHive(redHere()), false)) {
                if (haveBalls) {
                    empty = volleyFar(true);
                }
                // 3.5 FLOWER 1, next to the far spot
                if (flower && empty && BioBuzzRobot.armCalibrated() && flowerFits()) {
                    if (flowerCollect() && HivePlanner.fits(now(), redHere(), HivePlanner.flowerToFarShot(),
                            FAR_MIN_FIRE_SEC, slot, parkBySec)) {
                        launchTps = ShotSpeed.standard().tableSpeed(HiveAutoField.FAR_SHOT_PLANNED_FLOOR_IN);
                        robot.setLauncherVelocity(launchTps);
                        step = "FLOWER -> FAR SHOT";
                        if (drive(HivePlanner.flowerToFarShot(), false)) {
                            volleyFar(!farFired);
                        }
                    }
                } else if (flower) {
                    log.row(now(), "EVENT", "FLOWER skipped: " + (!empty ? "robot may hold POLLEN (G407)"
                            : !BioBuzzRobot.armCalibrated() ? "arm not calibrated" : "no time"));
                }
            }
        } else if (haveBalls) {
            log.row(now(), "EVENT", "FAR CELL skipped: no time");
        }
        robot.setLauncherVelocity(0);
        robot.setIndexerPower(0);
        robot.setIntake(0);
        if (robot.armState() == BioBuzzRobot.ArmState.DEPLOYED) {
            robot.stowArm();
        }
        park();
    }

    private boolean flowerFits() {
        double work = FLOWER_ARM_SETTLE_SEC + AutoPlanner.legSec(HiveAutoField.FLOWER_APPROACH, HivePlanner.flowerCreep())
                + FLOWER_MIN_DWELL_SEC + FLOWER_ARM_UP_SEC
                + AutoPlanner.legSec(HiveAutoField.FLOWER_STOP, HivePlanner.flowerBackOut());
        return HivePlanner.fits(now(), redHere(), HivePlanner.toFlower(), work, slot, parkBySec);
    }

    private void park() {
        step = "PARK";
        log.row(now(), "EVENT", String.format("PARK %s from %s (deadline was %.1f s)", slot, redHere(), parkStart()));
        while (opModeIsActive() && now() < AutoPlanner.MATCH_SEC) {
            List<AutoPlanner.Leg> legs = HivePlanner.park(redHere(), slot);
            Pose end = AutoPlanner.end(redHere(), legs);
            if (drive(legs, true)) {
                step = "PARKED " + slot;
                follower.hold(pedro(end.forAlliance(alliance())));
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

    // ---------------- Volleys ----------------

    /**
     * Pre-loads: camera aim (short), fire all 4 (the field starts with this CELL up; only the HIVE confirmed tipped
     * before the first ball keeps them for the far CELL); then watch for the TIP while the camera sees our HIVE.
     */
    private void volleyNear() {
        HiveVision.Cell near = nearCell();
        step = "AIM (pre-loads)";
        aim(Math.min(now() + NEAR_AIM_SEC, parkStart()), near);
        if (tip == Tip.TIPPED) {
            log.row(now(), "EVENT", "HIVE already tipped (" + tipWhy + "): pre-loads kept for the far CELL");
            return;
        }
        if (confirmed(near, CELL_FRESH_SEC) > 0) {
            launchTps = ShotSpeed.standard().tableSpeed(cells.get(near).target.horizontalIn());
            robot.setLauncherVelocity(launchTps);
        } else {
            log.row(now(), "EVENT", "near CELL not confirmed in view: firing on the planned heading");
        }
        nearVolleyRunning = true;
        fire(true, near);
        nearVolleyRunning = false;
        watchForTip();
    }

    /** Stays at the spot (holding) up to TIP_WATCH_SEC after the last pre-load while the camera sees our HIVE. */
    private void watchForTip() {
        if (vision == null || tip == Tip.TIPPED) {
            return;
        }
        double from = lastBallSec >= 0 ? lastBallSec : now();
        double end = Math.min(from + TIP_WATCH_SEC, parkStart());
        double watchStart = now();
        step = "TIP WATCH";
        while (opModeIsActive() && now() < end && tip == Tip.UNKNOWN
                && now() - Math.max(lastSeenAnySec, watchStart) < 0.25) {
            follower.update();
            pollVision();
            loopTick();
        }
        log.row(now(), "EVENT", "TIP after the pre-loads: " + tip + " (" + tipWhy + ")");
    }

    /**
     * Far CELL: camera aim. UP (confirmed, not contradicted by the near CELL): fire (camera speed). DOWN (or the near CELL
     * confirmed up), or seen but not sure: POLLEN kept. Not seen: fire on the planned heading only if {@code allowBlind}
     * and either the camera confirmed the TIP during this aim (e.g. the near CELL seen down from here: its sticker faces
     * this end after a TIP), or "blind far shot" is ON (default OFF: from here, in good light, no tag in view most likely
     * means NOT tipped) with all 4 pre-loads counted and NOT TIPPED never seen. Returns true when the robot is empty after.
     */
    private boolean volleyFar(boolean allowBlind) {
        HiveVision.Cell far = farCell();
        step = "AIM (far CELL)";
        double aimStart = now();
        aim(Math.min(now() + FAR_AIM_SEC, parkStart()), far);
        CellSeen s = cells.get(far);
        int v = cellState(far, CELL_FRESH_SEC);
        if (v != 0) {
            log.row(now(), "EVENT", "FAR CELL " + far + (v > 0 ? " UP" : " DOWN") + (s != null && s.target != null
                    ? String.format(": height %.1f in, floor %.1f in, bearing %.1f", s.target.heightIn(),
                    s.target.horizontalIn(), s.target.bearingDeg) : ": its other end is confirmed UP"));
            if (v < 0) {
                return false;                                   // G417: never launch into a downward CELL
            }
            launchTps = ShotSpeed.standard().tableSpeed(s.target.horizontalIn());
            robot.setLauncherVelocity(launchTps);
        } else if (s != null && s.atSec >= aimStart) {
            log.row(now(), "EVENT", String.format("far CELL seen but not sure (height %.1f in, %d frames): POLLEN kept",
                    s.target.heightIn(), s.run));
            return false;
        } else {
            boolean tippedNow = tip == Tip.TIPPED && tipSec >= aimStart;
            boolean blind = allowBlind && (tippedNow
                    || (blindFar && tip != Tip.NOT_TIPPED && preloadBalls >= AUTO_PRELOADS));
            log.row(now(), "EVENT", "far CELL not in view: " + (blind ? "firing on the planned heading" : "POLLEN kept")
                    + " (tip=" + tip + (tippedNow ? " seen from here" : "") + ", pre-loads counted " + preloadBalls
                    + ", blind far shot=" + blindFar + (allowBlind ? "" : ", 2nd far volley needs the camera") + ")");
            if (!blind) {
                return false;
            }
        }
        farFired = true;
        return fire(false, far);
    }

    /** Plan B at the near CELL: fires only when the camera confirms it UP (and its other end does not say otherwise). */
    private void volleyCamera(HiveVision.Cell cell, double aimSec, String label) {
        step = "AIM (" + label + ")";
        aim(Math.min(now() + aimSec, parkStart()), cell);
        int v = cellState(cell, CELL_FRESH_SEC);
        log.row(now(), "EVENT", label + ": CELL " + cell + (v > 0 ? " UP" : v < 0 ? " DOWN" : " not confirmed")
                + (v > 0 ? "" : ": POLLEN kept"));
        if (v > 0) {
            launchTps = ShotSpeed.standard().tableSpeed(cells.get(cell).target.horizontalIn());
            robot.setLauncherVelocity(launchTps);
            fire(false, cell);
        }
    }

    /** Turns on the spot (Pedro hold) until the camera bearing to {@code cell} is within AIM_TOLERANCE_DEG on 3 frames. */
    private HiveVision.Target aim(double endSec, HiveVision.Cell cell) {
        if (vision == null) {
            return null;
        }
        com.pedropathing.math.Pose spot = follower.pose();
        double targetHeading = spot.heading();
        follower.hold(spot);
        HiveVision.Target last = null;
        int settled = 0;
        while (opModeIsActive() && now() < endSec) {
            follower.update();
            if (pollVision()) {
                HiveVision.Target t = null;
                for (HiveVision.Target c : HiveCells.visible(vision, hiveAlliance())) {
                    if (c.cell == cell && (t == null || c.tagCount > t.tagCount)) {
                        t = c;
                    }
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

    /**
     * The TeleOp pocket-by-pocket feed (ShotFeeder) while Pedro holds the aimed pose, until the park start. Every volley
     * but the pre-loads stops at once when the camera sees its CELL going down (G417; launching at a tipping HIVE may
     * also undo the TIP). Returns true when the windmill is empty (4 balls counted or 3 full turns).
     */
    private boolean fire(boolean preloads, HiveVision.Cell target) {
        follower.hold(follower.pose());
        double settleEnd = Math.min(now() + AUTO_FIRE_SETTLE_SEC, parkStart());
        while (opModeIsActive() && now() < settleEnd) {
            follower.update();
            pollVision();
            step = "FIRE settle";
            loopTick();
        }
        ShotFeeder feeder = ShotFeeder.fromSettings(settings);
        BallCounter counter = BallCounter.forAuto();
        robot.setIntake(FEED_INTAKE_BOOST);
        double feedEnd = now() + AUTO_FEED_TIMEOUT_SEC;
        boolean stoppedForTip = false;
        // a CELL the camera confirmed UP when the feed starts: losing it completely means it tipped (stickers turn away)
        boolean watchLoss = !preloads && cellState(target, CELL_FRESH_SEC) > 0;
        while (opModeIsActive() && now() < Math.min(feedEnd, parkStart()) && feeder.shots() < AUTO_FEED_POCKETS
                && feeder.state() != ShotFeeder.State.JAMMED
                && !(counter.balls() >= AUTO_PRELOADS && feeder.state() != ShotFeeder.State.FEEDING)) {
            follower.update();
            pollVision();
            // G417: the CELL we fire at is going down: stop at once, even mid-pocket. Not for the pre-loads: the coach
            // wants all 4 in the CELL that starts up (as BioBuzz AUTO); its sticker turns away from this spot when it tips.
            if (!preloads && (cellState(target, CELL_STOP_FRESH_SEC) < 0
                    || (watchLoss && now() - cells.get(target).atSec > CELL_LOST_STOP_SEC))) {
                robot.setIndexerPower(0);
                stoppedForTip = true;
                break;
            }
            double t = now();
            int ticks = robot.indexerTicks();
            robot.setIndexerPower(feeder.update(t, ticks, robot.updateLauncherReady(), true));
            double velocity = robot.launcherVelocity();
            if (counter.update(t, feeder.state() == ShotFeeder.State.FEEDING, launchTps, velocity)) {
                log.row(t, "EVENT", "BALL " + counter.balls(), "launcher", velocity);
                if (preloads) {
                    preloadBalls = counter.balls();
                    if (firstBallSec < 0) {
                        firstBallSec = t;
                    }
                    lastBallSec = t;
                }
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
        boolean empty = counter.balls() >= AUTO_PRELOADS || feeder.shots() >= AUTO_FEED_POCKETS;
        if (preloads) {
            // G407: only collect more once the windmill is empty (4 balls counted, or 3 full turns of the windmill)
            preloadsGone = !stoppedForTip && empty;
            preloadBalls = counter.balls();
        }
        log.row(now(), "EVENT", String.format("FIRE done: %d balls, %d pockets, %d jams, %s%s%s", counter.balls(),
                feeder.shots(), feeder.jams(), feeder.state() == ShotFeeder.State.JAMMED ? "JAMMED" : "ok",
                stoppedForTip ? ", STOPPED: " + target + " going down or lost from view" : "",
                preloads ? (preloadsGone ? ", windmill empty" : ", balls may be left (no GARDEN)") : ""));
        return empty && !stoppedForTip;
    }

    // ---------------- TIP watch ----------------

    /**
     * Updates the camera. On a fresh frame: tracks each CELL of our HIVE (UP / DOWN / not sure, frames in a row), and
     * once a CELL is confirmed (CELL_FRAMES in a row): the near CELL DOWN or the far one UP = TIPPED (at any time); the
     * near CELL UP or the far one DOWN, TIP_SETTLE_SEC after the last pre-load = NOT TIPPED. Both tag stickers face the
     * end that is up, so the near spot can only see NOT TIPPED (after TIP 1 both turn away: the TIP watch then sees
     * nothing), the far spot only TIPPED. The TIP state is frozen once the far CELL has been fired at (it may tip back);
     * the CELL tracking goes on (fire() stops on a CELL going down).
     */
    private boolean pollVision() {
        if (vision == null || !vision.update()) {
            return false;
        }
        List<HiveVision.Target> visible = HiveCells.visible(vision, hiveAlliance());
        HiveVision.Cell near = nearCell();
        HiveVision.Target best = null;
        for (HiveVision.Cell c : new HiveVision.Cell[]{near, farCell()}) {
            HiveVision.Target t = null;
            for (HiveVision.Target v : visible) {
                if (v.cell == c && (t == null || v.tagCount > t.tagCount)) {
                    t = v;
                }
            }
            CellSeen s = cells.get(c);
            if (s == null) {
                s = new CellSeen();
                cells.put(c, s);
            }
            if (t == null) {
                s.run = 0;                                      // not in this frame: no longer "in a row"
                continue;
            }
            if (best == null || t.tagCount > best.tagCount) {
                best = t;
            }
            double h = t.heightIn();
            int v = h >= CELL_UP_MIN_HEIGHT_IN ? 1 : h < CELL_DOWN_MAX_HEIGHT_IN ? -1 : 0;
            s.run = v == s.verdict ? s.run + 1 : 1;
            s.verdict = v;
            s.atSec = now();
            s.target = t;
            lastSeenAnySec = s.atSec;
            if (v == 0 || s.run < CELL_FRAMES || farFired) {
                continue;
            }
            boolean startState = (c == near) == (v > 0);       // near up / far down = as at the start
            String what = String.format("%s %s on %d frames (%.0f in) at %.1f s", c, v > 0 ? "UP" : "DOWN", s.run, h,
                    now());
            if (!startState) {
                setTip(Tip.TIPPED, what);
                tipSec = now();
            } else if (preloadsGone && !nearVolleyRunning && lastBallSec >= 0
                    && now() - lastBallSec >= TIP_SETTLE_SEC) {
                setTip(Tip.NOT_TIPPED, what);
            }
        }
        if (best != null) {
            lastTarget = best;                                  // logged (height_in: calibrate the UP/DOWN bands)
        }
        return true;
    }

    /** +1 / -1 when CELL_FRAMES fresh frames in a row saw this CELL UP / DOWN, the last within maxAgeSec; else 0. */
    private int confirmed(HiveVision.Cell cell, double maxAgeSec) {
        CellSeen s = cells.get(cell);
        if (s == null || s.run < CELL_FRAMES || now() - s.atSec > maxAgeSec) {
            return 0;
        }
        return s.verdict;
    }

    /**
     * The HIVE is a seesaw: a CELL is DOWN (-1) when it is confirmed down or its other end is confirmed up; UP (+1) only
     * when it is confirmed up itself and its other end does not say otherwise; else 0.
     */
    private int cellState(HiveVision.Cell cell, double maxAgeSec) {
        int self = confirmed(cell, maxAgeSec), other = confirmed(HiveCells.otherEnd(cell), maxAgeSec);
        if (self < 0 || other > 0) {
            return -1;
        }
        return self > 0 ? 1 : 0;
    }

    private HiveVision.Cell nearCell() {
        return HiveVision.startingUpCell(hiveAlliance());
    }

    private HiveVision.Cell farCell() {
        return HiveCells.otherEnd(nearCell());
    }

    private void setTip(Tip t, String why) {
        if (t != tip) {
            log.row(now(), "EVENT", "TIP " + tip + " -> " + t + ": " + why);
        }
        tip = t;
        tipWhy = why;
    }

    // ---------------- Driving ----------------

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
                pollVision();
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

    /**
     * At the GARDEN corner until {@code endSec}: GARDEN_WIGGLE_IN back along the wall and in again, over and over, with
     * the intake on. A move that can't finish (POLLEN or the wall in the way) ends after GARDEN_WIGGLE_MOVE_SEC.
     */
    private void gardenWiggle(double endSec) {
        log.row(now(), "EVENT", String.format("GARDEN WIGGLE %.1f in until %.1f s", AutoField.GARDEN_WIGGLE_IN, endSec));
        int moves = 0;
        while (opModeIsActive() && now() < endSec) {
            for (AutoPlanner.Leg leg : HivePlanner.gardenWiggle()) {
                follower.follow(path(leg.to.forAlliance(alliance()), leg.speed));
                ElapsedTime move = new ElapsedTime();
                step = leg.label;
                while (opModeIsActive() && now() < endSec && move.seconds() < GARDEN_WIGGLE_MOVE_SEC
                        && (move.seconds() < 0.2 || follower.isBusy())) {
                    follower.update();
                    robot.setIntake(GARDEN_INTAKE_POWER);
                    if (poseLost()) {
                        log.row(now(), "EVENT", "POSE LOST " + here());
                        halt();
                        return;
                    }
                    loopTick();
                }
                moves++;
                if (!opModeIsActive() || now() >= endSec) {
                    break;
                }
            }
        }
        halt();
        log.row(now(), "EVENT", "GARDEN WIGGLE done: " + moves + " moves, at " + here());
    }

    /**
     * Step 3.5, FLOWER 1 (manual G418: POLLEN only out of the bottom; the arm's wedge lifts the bottom POLLEN over the
     * lower ring into the intake, as the flower test kit): line up, arm down while stopped, creep in with the intake on
     * until it touches, keep the intake running, arm up, back out. Returns true when it got to the FLOWER.
     */
    private boolean flowerCollect() {
        step = "TO FLOWER";
        if (!drive(HivePlanner.toFlower(), false)) {
            return false;
        }
        robot.deployArm();
        log.row(now(), "EVENT", "FLOWER: arm down at " + here());
        follower.hold(follower.pose());
        // every phase in the FLOWER ends FLOWER_ARM_UP_SEC before the park start: the arm goes up before backing out
        double settle = Math.min(now() + FLOWER_ARM_SETTLE_SEC, parkStart() - FLOWER_ARM_UP_SEC);
        while (opModeIsActive() && now() < settle) {
            follower.update();
            step = "FLOWER arm down";
            loopTick();
        }
        AutoPlanner.Leg creep = HivePlanner.flowerCreep();
        follower.follow(path(creep.to.forAlliance(alliance()), creep.speed));
        ElapsedTime legTimer = new ElapsedTime(), still = new ElapsedTime();
        Pose mark = here();
        double creepEnd = now() + AutoPlanner.legSec(redHere(), creep) * 2 + 1;
        while (opModeIsActive() && now() < Math.min(creepEnd, parkStart() - FLOWER_ARM_UP_SEC)
                && (legTimer.seconds() < 0.2 || follower.isBusy())) {
            follower.update();
            robot.setIntake(GARDEN_INTAKE_POWER);
            Pose p = here();
            if (p.distanceTo(mark) >= 0.5) {
                mark = p;
                still.reset();
            } else if (legTimer.seconds() > 0.5 && still.seconds() > FLOWER_CONTACT_SEC) {
                break;                                          // touching the FLOWER: the stock stop
            }
            step = "FLOWER IN";
            loopTick();
        }
        follower.hold(follower.pose());
        log.row(now(), "EVENT", "FLOWER: in at " + here());
        // dwell: up to FLOWER_DWELL_SEC, but end in time to fire the POLLEN from the far spot when that still fits
        // (at least FLOWER_MIN_DWELL_SEC; if even that leaves no time to fire, they stay on board for TeleOp)
        double fireBy = HivePlanner.parkStartSec(HiveAutoField.FAR_SHOT, slot, parkBySec) - FAR_MIN_FIRE_SEC
                - FLOWER_ARM_UP_SEC - AutoPlanner.legSec(redHere(), HivePlanner.flowerBackOut())
                - AutoPlanner.estimateSec(HiveAutoField.FLOWER_APPROACH, HivePlanner.flowerToFarShot());
        double dwellEnd = Math.min(Math.min(now() + FLOWER_DWELL_SEC, parkStart() - FLOWER_ARM_UP_SEC),
                Math.max(now() + FLOWER_MIN_DWELL_SEC, fireBy));
        while (opModeIsActive() && now() < dwellEnd) {
            follower.update();
            robot.setIntake(GARDEN_INTAKE_POWER);
            step = "FLOWER intake";
            loopTick();
        }
        robot.stowArm();
        double up = now() + FLOWER_ARM_UP_SEC;
        while (opModeIsActive() && now() < up) {
            follower.update();
            robot.setIntake(GARDEN_INTAKE_POWER);
            step = "FLOWER arm up";
            loopTick();
        }
        step = "FLOWER OUT";
        boolean out = drive(java.util.Collections.singletonList(HivePlanner.flowerBackOut()), false);
        robot.setIntake(0);
        log.row(now(), "EVENT", "FLOWER done at " + here() + (out ? "" : " (cut short)"));
        return out;
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
        return HivePlanner.parkStartSec(redHere(), slot, parkBySec);
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
        if (poseSet && !poseLost()) {
            BioBuzzMatchState.recordAutoHeading(redHere().headingDeg);   // for TeleOp's field-centric start
        }
        HiveVision.Target target = lastTarget;
        if (log != null) {
            log.row(t, step, p == null ? "" : p.x, p == null ? "" : p.y, p == null ? "" : p.headingDeg,
                    robot.launcherVelocity(), robot.launcherTarget(), robot.launcherReady(), robot.indexerTicks(),
                    robot.battery.getVoltage(), target == null ? "" : target.cell, target == null ? 0 : target.tagCount,
                    target == null ? "" : target.bearingDeg, target == null ? "" : target.heightIn(),
                    target == null ? "" : target.horizontalIn(), tip);
        }
        lastTarget = null;
        telemetry.addData("Step", step);
        telemetry.addData("Time", "%.1f s   (park starts by %.1f s)", t, follower == null ? 0 : parkStart());
        telemetry.addData("HIVE", "%s (%s)", tip, tipWhy);
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
            blindFar = !blindFar;
        }
        if (gamepad1.rightStickButtonWasPressed()) {
            flower = !flower;
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
        telemetry.addLine("BIOBUZZ HIVE AUTO " + alliance() + " - near CELL, GARDEN, under the HIVE, far CELL");
        telemetry.addData("Start (D-pad L/R)", start.label);
        telemetry.addData("Park slot (Square/X)", slot.label);
        telemetry.addLine(">> Tell your partner: we park in slot " + slot + ", they take slot " + slot.other()
                + "; we drive UNDER our HIVE and shoot from the REAR end");
        telemetry.addData("Shoot pre-loads (Cross/A)", shoot ? "ON" : "OFF");
        telemetry.addData("GARDEN POLLEN (Triangle/Y)", !garden ? "OFF" : shoot ? "ON" : "ON but needs shooting (G407)");
        telemetry.addData("Blind far shot (Circle/B)", blindFar
                ? "ON: far CELL fired at without the camera after 4 counted pre-loads (G417 risk if the HIVE did not tip)"
                : "OFF: far CELL only when the camera confirms it (normal in a lit field)");
        telemetry.addData("FLOWER 1 if time (R3)", !flower ? "OFF"
                : BioBuzzRobot.armCalibrated() ? "ON (only when the robot is empty)" : "ON but arm NOT CALIBRATED");
        telemetry.addData("Start delay (D-pad U/D)", "%d s", delaySec);
        telemetry.addData("Park by (LB/RB)", "%d s at the latest", parkBySec);

        double t = delaySec;
        Pose at = s;
        if (shoot) {
            t += AutoPlanner.estimateSec(at, HivePlanner.toShot(start)) + NEAR_AIM_SEC + AUTO_FIRE_SETTLE_SEC + 3.5;
            at = AutoField.SHOT;
            telemetry.addData("Plan", "near CELL: 4 pre-loads done ~%.0f s", t);
        }
        boolean gardenOn = shoot && garden
                && HivePlanner.fits(t, at, HivePlanner.toGarden(), GARDEN_MIN_DWELL_SEC, slot, parkBySec);
        if (gardenOn) {
            t += AutoPlanner.estimateSec(at, HivePlanner.toGarden());
            at = AutoPlanner.end(at, HivePlanner.toGarden());
            t = HivePlanner.gardenDwellEnd(t, at, slot, parkBySec, GARDEN_MIN_DWELL_SEC, GARDEN_DWELL_SEC,
                    FAR_AIM_SEC + AUTO_FIRE_SETTLE_SEC + FAR_FEED_TARGET_SEC);
            telemetry.addData("Plan", "GARDEN done ~%.0f s", t);
            List<AutoPlanner.Leg> under = HivePlanner.underHive(at);
            if (HivePlanner.fits(t, at, under, FAR_MIN_FIRE_SEC, slot, parkBySec)) {
                t += AutoPlanner.estimateSec(at, under);
                at = HiveAutoField.FAR_SHOT;
                telemetry.addData("Plan", "far spot ~%.0f s, fire until %.0f s", t,
                        HivePlanner.parkStartSec(at, slot, parkBySec));
            } else {
                telemetry.addData("Plan", "NO TIME for the far CELL (later park-by? GARDEN off?)");
            }
        }
        telemetry.addData("Plan", "PARK %s starts by %.0f s, takes ~%.0f s", slot,
                HivePlanner.parkStartSec(at, slot, parkBySec), AutoPlanner.estimateSec(at, HivePlanner.park(at, slot)));
        String route = HivePlanner.routeProblem(s, HivePlanner.toShot(start), null);
        if (route == null) {
            route = HivePlanner.routeProblem(at, HivePlanner.park(at, slot), slot.other());
        }
        telemetry.addData("Route check", route == null ? "OK" : "PROBLEM: " + route);

        String why = pedroProblem();
        telemetry.addData("Pedro", why == null ? "ready" : "NOT READY: " + why + (pedroError != null ? " (" + pedroError
                + ")" : "") + " -> AUTO will only back off the wall (LEAVE)");
        if (vision == null) {
            telemetry.addData("Camera", "NOT FOUND: %s (pre-loads unaimed; far shot only when blind far shot is ON)",
                    visionError);
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

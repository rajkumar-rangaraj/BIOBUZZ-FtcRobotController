package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoField.*;
import static org.firstinspires.ftc.teamcode.auto.HiveAutoField.*;

import org.firstinspires.ftc.teamcode.auto.AutoField.Pose;
import org.firstinspires.ftc.teamcode.auto.AutoField.Slot;
import org.firstinspires.ftc.teamcode.auto.AutoField.Start;
import org.firstinspires.ftc.teamcode.auto.AutoPlanner.Action;
import org.firstinspires.ftc.teamcode.auto.AutoPlanner.Leg;
import org.firstinspires.ftc.teamcode.fieldtour.FieldMap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Routes for the HIVE AUTO (RED frame), a copy of the AUTO plan with a second CELL (team, 12 Nov):
 *
 *   1. pre-loads into our up CELL (TIP 1)   2. GARDEN: 4 POLLEN   3. under the HIVE to its other end, fire them into
 *   the CELL that came up (TIP 2)   3.5 (if time) POLLEN from FLOWER 1, next to the far spot   4. PARK
 *
 * Shared with AutoPlanner (unchanged): the near shot, the GARDEN sweep and wiggle, the time model and the park route
 * from outside the HIVE frame. New here: the way under the HIVE, the far shot spot, FLOWER 1, and a footprint check that
 * knows the HIVE frame's real legs (AutoPlanner treats the whole frame as a solid box). No FTC SDK dependency.
 */
public final class HivePlanner {
    private HivePlanner() {
    }

    /** From the GARDEN to the front of the HIVE (long, open). */
    public static final double SPEED_TO_HIVE = 0.70;
    /** Under the HIVE (1.8 in from the legs) and on to the far spot in one leg: steady. */
    public static final double SPEED_UNDER = 0.60;
    public static final double SPEED_FAR_SHOT = AutoPlanner.SPEED_SHOT;
    public static final double SPEED_FLOWER = 0.40;
    /** Into the FLOWER with the arm down and the intake on (TeleOp drives at 30 % with the arm out). */
    public static final double SPEED_FLOWER_CREEP = 0.15;
    /** Margin to the HIVE frame (legs, feet) in the footprint check. */
    public static final double FRAME_MARGIN_IN = 1.5;
    /**
     * Near FLOWER 1 the robot first backs straight off to this y before it turns or moves sideways (a turning robot's
     * corners reach 12.7 in from its centre).
     */
    public static final double FLOWER_CLEAR_Y = 52.0;
    /** Robots this far toward the middle (x) from FLOWER 1's side count as "near FLOWER 1". */
    public static final double FLOWER_ZONE_X_MIN = -34.0;
    /**
     * A robot turning on the spot reaches 12.7 in from its centre: this far from the middle (y) it can turn next to the
     * HIVE frame (19.5 + 12.7 + margin).
     */
    public static final double FRAME_TURN_CLEAR_Y = 35.0;

    // ---------------- Routes (RED) ----------------

    public static List<Leg> toShot(Start start) {
        return AutoPlanner.toShot(start);
    }

    public static List<Leg> toGarden() {
        return AutoPlanner.toGarden();
    }

    public static List<Leg> gardenWiggle() {
        return AutoPlanner.gardenWiggle();
    }

    /**
     * Plan B (the HIVE did not tip): back to the near spot, as AutoPlanner's stretch but with the intake OFF (the robot
     * may already hold 4: G407).
     */
    public static List<Leg> gardenToNearShot() {
        return Arrays.asList(new Leg("BACK TURN", GARDEN_TURN, AutoPlanner.SPEED_TRANSIT, Action.DRIVE),
                new Leg("SHOT SPOT", SHOT, AutoPlanner.SPEED_SHOT, Action.DRIVE));
    }

    /**
     * From anywhere on the audience side (GARDEN, near shot spot): to the front of the HIVE turning to face the REAR
     * wall, then straight under it in the lane at exactly 90 deg to the far shot spot (one leg: Pedro slows down at its
     * end; the far aim then turns the robot on the spot to FAR_AIM_HEADING). Intake off: spilled POLLEN/NECTAR may lie
     * under the HIVE and the robot may already hold 4 (G407).
     */
    public static List<Leg> underHive(Pose from) {
        List<Leg> legs = new ArrayList<>();
        Pose p = from;
        if (Math.abs(normalizeDeg(FACE_REAR - p.headingDeg)) > 135) {
            // from the near spot (facing the audience wall): turn away from the middle line first, in two 90 deg steps
            legs.add(new Leg("TURN", GARDEN_TURN, SPEED_TO_HIVE, Action.DRIVE));
            p = GARDEN_TURN;
        }
        if (p.y < -HIVE_CLEAR_Y + 0.5) {
            legs.add(new Leg("HIVE ENTRY", new Pose(HIVE_LANE_X, -HIVE_CLEAR_Y, FACE_REAR), SPEED_TO_HIVE, Action.DRIVE));
        }
        legs.add(new Leg("UNDER THE HIVE", FAR_SHOT, SPEED_UNDER, Action.DRIVE));
        return legs;
    }

    /** Below the far spot, level with FLOWER 1's approach point: sideways moves near FLOWER 1 happen on this row. */
    static final Pose FAR_SHOT_LOW = new Pose(HIVE_LANE_X, FLOWER_APPROACH_Y, FACE_REAR);

    /**
     * {@code headingDeg} limited to {@code loDeg..hiDeg} (an arc shorter than 360 deg), the short way round. Returns
     * {@code headingDeg} itself when it is inside.
     */
    public static double limitHeadingDeg(double headingDeg, double loDeg, double hiDeg) {
        double mid = (loDeg + hiDeg) / 2, half = (hiDeg - loDeg) / 2;
        double off = normalizeDeg(headingDeg - mid);
        return Math.abs(off) <= half ? headingDeg : mid + Math.copySign(half, off);
    }

    // ---------------- The robot as built (HiveAutoField: ROBOT_*, START_OFFSET_IN) at the far spot ----------------

    /** The as-built robot's corners at a Pedro pose (RED frame): really START_OFFSET_IN further toward the REAR wall. */
    public static double[][] asBuiltCorners(Pose p) {
        double h = Math.toRadians(p.headingDeg), c = Math.cos(h), s = Math.sin(h);
        double cx = p.x, cy = p.y + START_OFFSET_IN;
        double[][] local = {{ROBOT_FRONT_IN, ROBOT_HALF_WIDTH_IN}, {ROBOT_FRONT_IN, -ROBOT_HALF_WIDTH_IN},
                {-ROBOT_BACK_IN, -ROBOT_HALF_WIDTH_IN}, {-ROBOT_BACK_IN, ROBOT_HALF_WIDTH_IN}};
        double[][] out = new double[4][];
        for (int i = 0; i < 4; i++) {
            double f = local[i][0], l = local[i][1];
            out[i] = new double[]{cx + f * c - l * s, cy + f * s + l * c};
        }
        return out;
    }

    /** As-built clearances (in) at a Pedro pose (RED frame): {REAR wall, our side's line x 0, FLOWER 1's CAD box}. */
    public static double[] asBuiltClearances(Pose p) {
        double[][] k = asBuiltCorners(p);
        double fx0 = FLOWER_X - FLOWER_HALF_WIDTH_IN, fx1 = FLOWER_X + FLOWER_HALF_WIDTH_IN;
        double fy0 = WALL_IN - FLOWER_DEPTH_IN, fy1 = WALL_IN;
        double wall = Double.MAX_VALUE, middle = Double.MAX_VALUE, flower = Double.MAX_VALUE;
        for (int i = 0; i < 4; i++) {
            double[] a = k[i], b = k[(i + 1) % 4];
            wall = Math.min(wall, WALL_IN - a[1]);
            middle = Math.min(middle, -a[0]);
            for (int j = 0; j <= 80; j++) {                     // along the robot's side, every ~0.2 in
                double x = a[0] + (b[0] - a[0]) * j / 80.0, y = a[1] + (b[1] - a[1]) * j / 80.0;
                double dx = Math.max(Math.max(fx0 - x, x - fx1), 0), dy = Math.max(Math.max(fy0 - y, y - fy1), 0);
                flower = Math.min(flower, Math.hypot(dx, dy));
            }
        }
        return new double[]{wall, middle, flower};
    }

    /**
     * The headings (deg, RED frame) the far aim may turn to where Pedro really stopped ({@code at}; its heading = where
     * the robot points now): FAR_AIM_HEADING +- FAR_AIM_MAX_TURN_DEG, cut so that every heading from at.headingDeg to
     * there, plus FAR_AIM_HEADING_ERR_DEG of Pedro heading error, keeps the as-built robot FAR_AIM_CLEAR_IN off the REAR
     * wall, our side's line and FLOWER 1, or, from one it is already closer to, never closer than now (it may turn away
     * from it). {lo, hi} in at.headingDeg's 360 wrap, or null when there is none or the robot is already within
     * FAR_AIM_MIN_CLEAR_IN of something (it must not turn).
     */
    public static double[] farAimLimits(Pose at) {
        final double step = 0.05, search = 30;
        double[] now = asBuiltClearances(at);
        double[] floor = new double[now.length];
        for (int i = 0; i < now.length; i++) {
            if (now[i] < FAR_AIM_MIN_CLEAR_IN) {
                return null;
            }
            floor[i] = Math.min(FAR_AIM_CLEAR_IN, now[i]);
        }
        double lo = at.headingDeg, hi = at.headingDeg;
        while (lo - step > at.headingDeg - search && clearOf(new Pose(at.x, at.y, lo - step), floor)) {
            lo -= step;
        }
        while (hi + step < at.headingDeg + search && clearOf(new Pose(at.x, at.y, hi + step), floor)) {
            hi += step;
        }
        double centre = at.headingDeg + normalizeDeg(FAR_AIM_HEADING - at.headingDeg);
        double aimLo = Math.max(lo + FAR_AIM_HEADING_ERR_DEG, centre - FAR_AIM_MAX_TURN_DEG);
        double aimHi = Math.min(hi - FAR_AIM_HEADING_ERR_DEG, centre + FAR_AIM_MAX_TURN_DEG);
        return aimLo <= aimHi ? new double[]{aimLo, aimHi} : null;
    }

    private static boolean clearOf(Pose p, double[] floor) {
        double[] c = asBuiltClearances(p);
        for (int i = 0; i < c.length; i++) {
            if (c[i] < floor[i]) {
                return false;
            }
        }
        return true;
    }

    /** From the far spot to FLOWER 1's approach point (intake facing the FLOWER, centred on it): back, then across. */
    public static List<Leg> toFlower() {
        return Arrays.asList(new Leg("BACK OFF", FAR_SHOT_LOW, SPEED_FLOWER, Action.DRIVE),
                new Leg("FLOWER LINE-UP", FLOWER_APPROACH, SPEED_FLOWER, Action.DRIVE));
    }

    /** Arm down, intake on: in to the stock stop. */
    public static Leg flowerCreep() {
        return new Leg("FLOWER IN", FLOWER_STOP, SPEED_FLOWER_CREEP, Action.INTAKE);
    }

    /** Back out of the FLOWER (arm up first). */
    public static Leg flowerBackOut() {
        return new Leg("FLOWER OUT", FLOWER_APPROACH, SPEED_FLOWER, Action.DRIVE);
    }

    /** From FLOWER 1's approach point back to the far spot: across, then forward. */
    public static List<Leg> flowerToFarShot() {
        return Arrays.asList(new Leg("TO LANE", FAR_SHOT_LOW, SPEED_FLOWER, Action.DRIVE),
                new Leg("FAR SHOT SPOT", FAR_SHOT, SPEED_FAR_SHOT, Action.DRIVE));
    }

    /** True when the robot centre is in the HIVE frame's span (it must leave along the lane before turning away). */
    public static boolean underTheHive(Pose p) {
        return Math.abs(p.y) < HIVE_CLEAR_Y - 0.5 && p.x > FRAME_FLOOR_X_MIN - HALF_WIDTH_IN - FRAME_MARGIN_IN;
    }

    /** True when the robot is close to FLOWER 1 (far spot, FLOWER legs): back straight off before moving away. */
    public static boolean nearFlower(Pose p) {
        return p.y > FLOWER_CLEAR_Y + 0.5 && p.x > FLOWER_ZONE_X_MIN;
    }

    /**
     * Park pose for the HIVE AUTO from the REAR side: slot A's place facing the REAR wall (the robot is square, so the
     * footprint is AutoField.slot()'s; no 90 deg turn next to FLOWER 1), at REAR_SLOT_A_Y (the as-built robot's short
     * back faces the LOADING ZONE).
     */
    public static Pose rearSlot(Slot slot) {
        Pose s = AutoField.slot(slot);
        return new Pose(s.x, slot == Slot.A ? REAR_SLOT_A_Y : s.y, FACE_REAR);
    }

    /**
     * Park from anywhere:
     * - under the HIVE: first straight out along the lane to the end that parks sooner;
     * - near FLOWER 1: first straight back to FLOWER_CLEAR_Y;
     * - from the REAR side into slot A: straight across, facing the REAR wall (it never passes slot B);
     * - otherwise AutoPlanner's park (lane beside the HIVE, slow past slot B, partner-aware).
     */
    public static List<Leg> park(Pose from, Slot slot) {
        List<Leg> legs = new ArrayList<>();
        Pose p = from;
        if (underTheHive(p)) {
            Pose rear = new Pose(p.x, HIVE_CLEAR_Y, p.headingDeg), front = new Pose(p.x, -HIVE_CLEAR_Y, p.headingDeg);
            Leg outRear = new Leg("OUT FROM UNDER", rear, AutoPlanner.SPEED_PARK, Action.DRIVE);
            Leg outFront = new Leg("OUT FROM UNDER", front, AutoPlanner.SPEED_PARK, Action.DRIVE);
            double viaRear = AutoPlanner.legSec(p, outRear) + AutoPlanner.estimateSec(rear, parkOutside(rear, slot));
            double viaFront = AutoPlanner.legSec(p, outFront) + AutoPlanner.estimateSec(front, parkOutside(front, slot));
            legs.add(viaRear <= viaFront ? outRear : outFront);
            p = viaRear <= viaFront ? rear : front;
        }
        legs.addAll(parkOutside(p, slot));
        return legs;
    }

    private static List<Leg> parkOutside(Pose from, Slot slot) {
        List<Leg> legs = new ArrayList<>();
        Pose p = from;
        if (Math.abs(p.x - FLOWER_X) < 3.0 && p.y > FLOWER_APPROACH_Y + 0.5) {
            // in the FLOWER (step 3.5 cut short): straight back out first (the arm is raised before any park)
            p = FLOWER_APPROACH;
            legs.add(HivePlanner.flowerBackOut());
        }
        if (nearFlower(p)) {
            p = new Pose(p.x, FLOWER_CLEAR_Y, p.headingDeg);
            legs.add(new Leg("BACK OFF", p, AutoPlanner.SPEED_PARK, Action.DRIVE));
        }
        boolean rearFacing = Math.abs(normalizeDeg(p.headingDeg - FACE_REAR)) < 45;
        if (slot == Slot.A && p.y >= HIVE_CLEAR_Y - 0.5 && (p.x > AutoField.LANE_X + 0.5 || rearFacing)) {
            // REAR side: up to slot A's row, then straight across facing the REAR wall (never beside slot B, and no
            // turn on the spot next to the wall)
            if (p.y < REAR_SLOT_A_Y - 0.5) {
                p = new Pose(p.x, REAR_SLOT_A_Y, p.headingDeg);
                legs.add(new Leg("TO REAR ROW", p, AutoPlanner.SPEED_PARK, Action.DRIVE));
            }
            legs.add(new Leg("PARK A", rearSlot(Slot.A), AutoPlanner.SPEED_PARK, Action.DRIVE));
            return legs;
        }
        if (Math.abs(p.y) < FRAME_TURN_CLEAR_Y - 0.5 && p.x > AutoField.LANE_X + 0.5) {
            // in front of / behind the HIVE frame: straight away from it first (AutoPlanner's park turns on its first leg)
            p = new Pose(p.x, p.y >= 0 ? FRAME_TURN_CLEAR_Y : -FRAME_TURN_CLEAR_Y, p.headingDeg);
            legs.add(new Leg("AWAY FROM HIVE", p, AutoPlanner.SPEED_PARK, Action.DRIVE));
        }
        legs.addAll(AutoPlanner.park(p, slot));
        return legs;
    }

    // ---------------- Time (AutoPlanner's model) ----------------

    /**
     * End of the GARDEN corner back-and-forth: up to {@code maxDwell}, at least {@code minDwell}, but early enough that
     * the far CELL still gets {@code farWorkSec} (aim + settle + feed) before the park start; never after the park start.
     */
    public static double gardenDwellEnd(double nowSec, Pose corner, Slot slot, double userDeadlineSec, double minDwell,
                                        double maxDwell, double farWorkSec) {
        double latest = parkStartSec(FAR_SHOT, slot, userDeadlineSec) - farWorkSec
                - AutoPlanner.estimateSec(corner, underHive(corner));
        double end = Math.min(nowSec + maxDwell, Math.max(nowSec + minDwell, latest));
        return Math.min(end, parkStartSec(corner, slot, userDeadlineSec));
    }
    public static double parkStartSec(Pose from, Slot slot, double userDeadlineSec) {
        return Math.min(userDeadlineSec,
                AutoPlanner.MATCH_SEC - AutoPlanner.PARK_MARGIN_SEC - AutoPlanner.estimateSec(from, park(from, slot)));
    }

    /** True when the legs plus extraSec of work at their end still leave time to park from there. */
    public static boolean fits(double nowSec, Pose from, List<Leg> legs, double extraSec, Slot slot,
                               double userDeadlineSec) {
        double done = nowSec + AutoPlanner.estimateSec(from, legs) + extraSec;
        return done <= parkStartSec(AutoPlanner.end(from, legs), slot, userDeadlineSec);
    }

    // ---------------- Footprint check (HIVE frame legs instead of a solid box) ----------------

    /**
     * Null when the robot at p is clear of the walls, our side's line, the HIVE frame's feet and legs (two-box robot
     * body vs the leaning legs), the FLOWERs (FLOWER 1 skipped when {@code atFlower}: the FLOWER legs touch it on
     * purpose) and the partner's slot.
     */
    public static String footprintProblem(Pose p, Slot partnerSlot, boolean atFlower) {
        double[][] c = AutoPlanner.corners(p);
        for (double[] k : c) {
            if (Math.abs(k[0]) > WALL_IN + AutoPlanner.WALL_TOLERANCE_IN
                    || Math.abs(k[1]) > WALL_IN + AutoPlanner.WALL_TOLERANCE_IN) {
                return "outside the field at " + p;
            }
            if (k[0] > -AutoPlanner.OWN_SIDE_MARGIN_IN) {
                return "on the opponent side at " + p;
            }
        }
        String frame = frameProblem(p);
        if (frame != null) {
            return frame;
        }
        for (FieldMap.Place f : FieldMap.Place.values()) {
            if (!f.label.startsWith("Flower") || (atFlower && f == FieldMap.Place.FLOWER_1)) {
                continue;
            }
            if (hitsFlower(c, f)) {
                return "hits " + f.label + " at " + p;
            }
        }
        if (partnerSlot != null) {
            Pose s = AutoField.slot(partnerSlot);
            if (AutoPlanner.overlapsBox(c, s.x, s.y, HALF_LENGTH_IN + AutoPlanner.PARTNER_MARGIN_IN,
                    HALF_WIDTH_IN + AutoPlanner.PARTNER_MARGIN_IN)) {
                return "hits the partner in slot " + partnerSlot + " at " + p;
            }
        }
        return null;
    }

    /**
     * A FLOWER's CAD footprint at robot heights (rings and pipes: FLOWER_HALF_WIDTH_IN either side along its wall,
     * FLOWER_DEPTH_IN out from it) plus FLOWER_MARGIN_IN. AutoPlanner's 3.7 in circle is bigger than the FLOWER and
     * would block the far shot spot, 1.3 in beside FLOWER 1.
     */
    static boolean hitsFlower(double[][] corners, FieldMap.Place f) {
        double m = AutoPlanner.FLOWER_MARGIN_IN, out = FLOWER_DEPTH_IN / 2;
        if (Math.abs(f.x) > Math.abs(f.y)) {                    // on the RED / BLUE wall
            return AutoPlanner.overlapsBox(corners, Math.signum(f.x) * (WALL_IN - out), f.y, out + m,
                    FLOWER_HALF_WIDTH_IN + m);
        }
        return AutoPlanner.overlapsBox(corners, f.x, Math.signum(f.y) * (WALL_IN - out), FLOWER_HALF_WIDTH_IN + m,
                out + m);                                       // on the audience / REAR wall
    }

    /** The RED half of the HIVE frame (feet, low and high leg boxes, both legs), plus margin, vs the robot's body. */
    static String frameProblem(Pose p) {
        double m = FRAME_MARGIN_IN;
        double[][] low = AutoPlanner.corners(p);
        double[][] high = corners(p, HALF_LENGTH_IN, ROBOT_UPPER_HALF_WIDTH_IN);
        double floorCx = (FRAME_FLOOR_X_MIN + FRAME_FLOOR_X_MAX) / 2, floorHx = (FRAME_FLOOR_X_MAX - FRAME_FLOOR_X_MIN) / 2;
        if (AutoPlanner.overlapsBox(low, floorCx, 0, floorHx + m, FRAME_HALF_Y + m)) {
            return "hits the HIVE frame feet at " + p;
        }
        for (int side = -1; side <= 1; side += 2) {
            double lowCy = side * (LEG_LOW_Y_MIN + FRAME_HALF_Y) / 2, lowHy = (FRAME_HALF_Y - LEG_LOW_Y_MIN) / 2;
            double lowCx = (FRAME_FLOOR_X_MIN + LEG_LOW_X_MAX) / 2, lowHx = (LEG_LOW_X_MAX - FRAME_FLOOR_X_MIN) / 2;
            if (AutoPlanner.overlapsBox(low, lowCx, lowCy, lowHx + m, lowHy + m)) {
                return "hits a HIVE frame leg (low) at " + p;
            }
            double highCy = side * (LEG_HIGH_Y_MIN + LEG_HIGH_Y_MAX) / 2, highHy = (LEG_HIGH_Y_MAX - LEG_HIGH_Y_MIN) / 2;
            double highCx = (FRAME_FLOOR_X_MIN + LEG_HIGH_X_MAX) / 2, highHx = (LEG_HIGH_X_MAX - FRAME_FLOOR_X_MIN) / 2;
            if (AutoPlanner.overlapsBox(high, highCx, highCy, highHx + m, highHy + m)) {
                return "hits a HIVE frame leg (high) at " + p;
            }
        }
        return null;
    }

    /** Robot rectangle corners with the given half sizes (same order as AutoPlanner.corners). */
    static double[][] corners(Pose p, double halfLength, double halfWidth) {
        double h = Math.toRadians(p.headingDeg);
        double fx = Math.cos(h), fy = Math.sin(h), sx = -fy, sy = fx;
        double[][] c = new double[4][];
        int i = 0;
        for (int a = -1; a <= 1; a += 2) {
            for (int b = -1; b <= 1; b += 2) {
                c[i++] = new double[]{p.x + a * halfLength * fx + b * halfWidth * sx,
                        p.y + a * halfLength * fy + b * halfWidth * sy};
            }
        }
        return c;
    }

    /** Checks the robot along every leg (straight line, heading turning the short way, as Pedro's linear()). */
    public static String routeProblem(Pose from, List<Leg> legs, Slot partnerSlot) {
        Pose p = from;
        for (Leg l : legs) {
            boolean atFlower = l.label.startsWith("FLOWER IN") || l.label.startsWith("FLOWER OUT");
            double dh = normalizeDeg(l.to.headingDeg - p.headingDeg);
            int n = (int) Math.max(4, Math.ceil(Math.max(p.distanceTo(l.to), Math.abs(dh) / 2)));
            for (int i = 0; i <= n; i++) {
                double t = (double) i / n;
                Pose q = new Pose(p.x + t * (l.to.x - p.x), p.y + t * (l.to.y - p.y), p.headingDeg + t * dh);
                String problem = footprintProblem(q, partnerSlot, atFlower);
                if (problem != null) {
                    return l.label + ": " + problem;
                }
            }
            p = l.to;
        }
        return null;
    }
}

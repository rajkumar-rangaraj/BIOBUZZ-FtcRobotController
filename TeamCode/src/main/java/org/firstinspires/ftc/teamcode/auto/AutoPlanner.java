package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoField.*;

import org.firstinspires.ftc.teamcode.fieldtour.FieldMap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * The AUTO routes (RED frame, see {@link AutoField}), their time estimates, the park deadline and a footprint check.
 * No FTC SDK dependency: laptop-tested in AutoPlannerTest.
 *
 * Plan: [start] -> SHOT (fire the 4 pre-loads) -> GARDEN sweep (intake the 4 POLLEN) -> [stretch: back to SHOT, fire]
 * -> PARK in slot A or B. Parking always wins: every step is skipped or cut short so the park starts by
 * {@link #parkStartSec}.
 */
public final class AutoPlanner {
    private AutoPlanner() {
    }

    public static final double MATCH_SEC = 30.0;
    /** The park should be finished this long before the end of AUTO. */
    public static final double PARK_MARGIN_SEC = 1.0;
    /** Pedro Foresight forward top speed (pedro.Constants, AutoTune 4 Nov), in/s. */
    public static final double TOP_SPEED_IN_PER_SEC = 61.4;
    /**
     * Average speed over a leg as a fraction of the commanded path speed (speed-up and slow-down included). Fitted to 7
     * BLUE runs on 9 Nov: lane 97.6 in at 0.7 took 3.9 s, sweep 24.5 in at 0.25 took 2.9 s, line-up 0.95 s, park leg 0.9 s.
     */
    public static final double SPEED_EFFICIENCY = 0.6;
    /** Per leg: settle at the end. */
    public static final double LEG_OVERHEAD_SEC = 0.4;
    /** A turn of 90 deg adds half of this on top of the drive time (the robot turns while it drives). */
    public static final double TURN_SEC_PER_90 = 0.5;

    /** Path speeds as a fraction of top speed (Pedro maxPathSpeed). */
    public static final double SPEED_SHOT = 0.30;
    public static final double SPEED_TRANSIT = 0.60;
    /** GARDEN: quick to just before the first POLLEN, then slow through the line so the intake grabs them. */
    public static final double SPEED_SWEEP_APPROACH = 0.40;
    public static final double SPEED_SWEEP = 0.15;
    /** Back and forth at the GARDEN corner (GARDEN_WIGGLE_IN), intake on. */
    public static final double SPEED_WIGGLE = 0.25;
    public static final double SPEED_PARK = 0.70;
    /** Slot A only: speed while passing beside slot B, where the partner may already be parked. */
    public static final double SPEED_PASS = 0.30;
    /** The slow pass starts/ends this far before/after our robot is level with a robot in slot B. */
    public static final double PASS_MARGIN_IN = 3.0;

    /** Margins of the footprint check. */
    public static final double HIVE_MARGIN_IN = 2.0;
    public static final double FLOWER_MARGIN_IN = 1.0;
    public static final double PARTNER_MARGIN_IN = 1.5;
    public static final double OWN_SIDE_MARGIN_IN = 0.5;
    public static final double WALL_TOLERANCE_IN = 0.05;

    /** Lane used from the ALLIANCE-wall start to the shot spot (no partner is parked yet: more room from the HIVE). */
    public static final double TRANSIT_X = -42.0;
    /** Past the lane and this close to the slot's y: drive straight into the slot (no trip back to the lane). */
    public static final double FINAL_APPROACH_HALF_Y_IN = 3.0;
    /** PARK counts once this much of the robot is inside the LOADING ZONE (both directions). */
    public static final double PARKED_MIN_OVERLAP_IN = 2.0;

    /** True when the robot already overlaps our LOADING ZONE by PARKED_MIN_OVERLAP_IN (PARK would score). */
    public static boolean partlyInZone(Pose p) {
        double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (double[] k : corners(p)) {
            minX = Math.min(minX, k[0]);
            minY = Math.min(minY, k[1]);
            maxY = Math.max(maxY, k[1]);
        }
        double overlapY = Math.min(maxY, ZONE_Y_MAX) - Math.max(minY, ZONE_Y_MIN);
        return ZONE_INNER_X - minX >= PARKED_MIN_OVERLAP_IN && overlapY >= PARKED_MIN_OVERLAP_IN;
    }

    public enum Action { DRIVE, INTAKE }

    public static final class Leg {
        public final String label;
        public final Pose to;
        /** Where the robot really stops: {@link #to} itself, or the wall in front of it on a leg into a wall. */
        public final Pose stop;
        public final double speed;
        public final Action action;

        public Leg(String label, Pose to, double speed, Action action) {
            this(label, to, to, speed, action);
        }

        /** A leg into a wall: it aims at {@code to} beyond the wall, the wall stops the robot at {@code stop}. */
        public Leg(String label, Pose to, Pose stop, double speed, Action action) {
            this.label = label;
            this.to = to;
            this.stop = stop;
            this.speed = speed;
            this.action = action;
        }

        /** Staying still at the end of this leg means the robot has reached the wall, not that it is blocked. */
        public boolean intoWall() {
            return stop != to;
        }

        @Override
        public String toString() {
            return label + " -> " + to;
        }
    }

    private static Leg leg(String label, double x, double y, double h, double speed) {
        return new Leg(label, new Pose(x, y, h), speed, Action.DRIVE);
    }

    // ---------------- Routes (RED) ----------------

    public static List<Leg> toShot(Start start) {
        if (start == Start.CELL_WALL) {
            return Arrays.asList(new Leg("SHOT SPOT", SHOT, SPEED_SHOT, Action.DRIVE));
        }
        return Arrays.asList(
                leg("OFF WALL", TRANSIT_X, START_ALLIANCE_WALL.y, FACE_ALLIANCE_WALL, SPEED_TRANSIT),
                leg("LANE", TRANSIT_X, TURN_Y, FACE_ALLIANCE_WALL, SPEED_TRANSIT),
                leg("TURN", CELL_X, TURN_Y, FACE_AUDIENCE, SPEED_TRANSIT),
                new Leg("SHOT SPOT", SHOT, SPEED_SHOT, Action.DRIVE));
    }

    /** From the shot spot: back off and turn, line up on the wall, sweep the 4 POLLEN with the intake on, into the wall. */
    public static List<Leg> toGarden() {
        return Arrays.asList(
                new Leg("GARDEN TURN", GARDEN_TURN, SPEED_TRANSIT, Action.DRIVE),
                leg("GARDEN LINE-UP", GARDEN_SWEEP_START_X, GARDEN_LANE_Y, FACE_ALLIANCE_WALL, SPEED_TRANSIT),
                new Leg("GARDEN APPROACH", new Pose(GARDEN_SWEEP_SLOW_X, GARDEN_LANE_Y, FACE_ALLIANCE_WALL),
                        SPEED_SWEEP_APPROACH, Action.INTAKE),
                new Leg("GARDEN SWEEP", new Pose(GARDEN_SWEEP_END_X, GARDEN_LANE_Y, FACE_ALLIANCE_WALL),
                        gardenCorner(), SPEED_SWEEP, Action.INTAKE));
    }

    /** Intake bumper against the RED wall at the GARDEN corner. */
    public static Pose gardenCorner() {
        return new Pose(GARDEN_CORNER_X, GARDEN_LANE_Y, FACE_ALLIANCE_WALL);
    }

    /**
     * At the GARDEN corner, from where the robot is now ({@code at}, RED frame): back GARDEN_WIGGLE_IN along the wall from
     * there, then push in to the wall again. The op mode repeats it while the intake runs at the corner, so a POLLEN
     * pressed against the wall gets room to roll into the intake. The back-off is measured from where the robot really
     * stopped (the wall or a POLLEN), not from where it thinks the wall is, and never leaves the part of the lane the
     * sweep already crossed.
     */
    public static List<Leg> gardenWiggle(Pose at) {
        double outX = Math.min(Math.max(at.x, GARDEN_SWEEP_END_X) + GARDEN_WIGGLE_IN, GARDEN_SWEEP_SLOW_X);
        return Arrays.asList(
                new Leg("GARDEN WIGGLE OUT", new Pose(outX, GARDEN_LANE_Y, FACE_ALLIANCE_WALL), SPEED_WIGGLE,
                        Action.INTAKE),
                new Leg("GARDEN WIGGLE IN", new Pose(GARDEN_SWEEP_END_X, GARDEN_LANE_Y, FACE_ALLIANCE_WALL),
                        gardenCorner(), SPEED_WIGGLE, Action.INTAKE));
    }

    /** End of the GARDEN sweep back to the shot spot (stretch); the intake keeps pulling the POLLEN in on the way. */
    public static List<Leg> gardenToShot() {
        return Arrays.asList(
                new Leg("BACK TURN", GARDEN_TURN, SPEED_TRANSIT, Action.INTAKE),
                new Leg("SHOT SPOT", SHOT, SPEED_SHOT, Action.DRIVE));
    }

    /** From anywhere on our side: to the lane, along the lane to the slot's level, into the slot. */
    public static List<Leg> park(Pose from, Slot slot) {
        Pose target = AutoField.slot(slot);
        List<Leg> legs = new ArrayList<>();
        if (from.x < LANE_X - 0.5 && Math.abs(from.y - target.y) <= FINAL_APPROACH_HALF_Y_IN) {
            legs.add(new Leg("PARK " + slot.name(), target, SPEED_PARK, Action.DRIVE));   // already on the way in
            return legs;
        }
        double laneY = Math.max(TURN_Y, Math.min(-TURN_Y, from.y));
        Pose laneIn = new Pose(LANE_X, laneY, FACE_ALLIANCE_WALL);
        if (from.distanceTo(laneIn) > 0.5 || Math.abs(normalizeDeg(from.headingDeg - FACE_ALLIANCE_WALL)) > 2) {
            legs.add(new Leg("TO LANE", laneIn, SPEED_PARK, Action.DRIVE));
        }
        if (slot == Slot.A) {
            // Slot A: a partner may already be parked in B. Pass beside B slowly (a block there stops us gently).
            Pose b = AutoField.slot(Slot.B);
            double passFrom = b.y - 2 * HALF_WIDTH_IN - PASS_MARGIN_IN, passTo = b.y + 2 * HALF_WIDTH_IN + PASS_MARGIN_IN;
            double y = laneY;
            if (y < passFrom - 0.5) {
                legs.add(leg("LANE", LANE_X, passFrom, FACE_ALLIANCE_WALL, SPEED_PARK));
                y = passFrom;
            }
            if (y < passTo - 0.5) {
                legs.add(leg("PASS SLOT B", LANE_X, Math.min(passTo, target.y), FACE_ALLIANCE_WALL, SPEED_PASS));
                y = Math.min(passTo, target.y);
            }
            laneY = y;
        }
        if (Math.abs(target.y - laneY) > 0.5) {
            legs.add(leg("LANE", LANE_X, target.y, FACE_ALLIANCE_WALL, SPEED_PARK));
        }
        legs.add(new Leg("PARK " + slot.name(), target, SPEED_PARK, Action.DRIVE));
        return legs;
    }

    // ---------------- Time ----------------

    /** A leg into a wall ends once the robot has stayed still this long against it (the op mode uses the same). */
    public static final double WALL_SETTLE_SEC = 1.0;
    /**
     * Still on a leg into a wall counts as AT THE WALL only this close to where the wall should stop it (pose error ~2 in,
     * a POLLEN wedged in front ~3 in). Still farther out = blocked (e.g. by a robot): the op mode stops there as on any
     * other leg, so no back and forth into whatever is in the way.
     */
    public static final double WALL_CONTACT_BAND_IN = 6.0;

    /** On a leg into a wall, is the robot ({@code at}, RED frame) close enough to the wall to be stopped by it? */
    public static boolean atWall(Pose at, Leg leg) {
        return leg.intoWall() && at.distanceTo(leg.stop) <= WALL_CONTACT_BAND_IN;
    }

    public static double legSec(Pose from, Leg leg) {
        double drive = from.distanceTo(leg.stop) / (leg.speed * TOP_SPEED_IN_PER_SEC * SPEED_EFFICIENCY);
        double turn = Math.abs(normalizeDeg(leg.to.headingDeg - from.headingDeg)) / 90 * TURN_SEC_PER_90;
        return drive + turn / 2 + LEG_OVERHEAD_SEC + (leg.intoWall() ? WALL_SETTLE_SEC : 0);
    }

    public static double estimateSec(Pose from, List<Leg> legs) {
        double t = 0;
        Pose p = from;
        for (Leg l : legs) {
            t += legSec(p, l);
            p = l.stop;
        }
        return t;
    }

    /** Where the robot ends after the legs (the wall's stop, on a leg into a wall). */
    public static Pose end(Pose from, List<Leg> legs) {
        return legs.isEmpty() ? from : legs.get(legs.size() - 1).stop;
    }

    /** Latest match time to start parking from {@code from}: the user deadline, earlier if the drive is long. */
    public static double parkStartSec(Pose from, Slot slot, double userDeadlineSec) {
        return Math.min(userDeadlineSec, MATCH_SEC - PARK_MARGIN_SEC - estimateSec(from, park(from, slot)));
    }

    /**
     * True when, starting at {@code nowSec}, the legs plus {@code extraSec} of work at their end still leave time to
     * park from there.
     */
    public static boolean fits(double nowSec, Pose from, List<Leg> legs, double extraSec, Slot slot,
                               double userDeadlineSec) {
        double done = nowSec + estimateSec(from, legs) + extraSec;
        return done <= parkStartSec(end(from, legs), slot, userDeadlineSec);
    }

    // ---------------- Footprint check ----------------

    /** Robot corners at a pose (RED frame). */
    public static double[][] corners(Pose p) {
        double h = Math.toRadians(p.headingDeg);
        double fx = Math.cos(h), fy = Math.sin(h), sx = -fy, sy = fx;
        double[][] c = new double[4][];
        int i = 0;
        for (int a = -1; a <= 1; a += 2) {
            for (int b = -1; b <= 1; b += 2) {
                c[i++] = new double[]{p.x + a * HALF_LENGTH_IN * fx + b * HALF_WIDTH_IN * sx,
                        p.y + a * HALF_LENGTH_IN * fy + b * HALF_WIDTH_IN * sy};
            }
        }
        return c;
    }

    /** Null when the robot at {@code p} is clear of walls, the HIVE frame, FLOWERs, the BLUE side and the partner. */
    public static String footprintProblem(Pose p, Slot partnerSlot) {
        double[][] c = corners(p);
        for (double[] k : c) {
            if (Math.abs(k[0]) > WALL_IN + WALL_TOLERANCE_IN || Math.abs(k[1]) > WALL_IN + WALL_TOLERANCE_IN) {
                return "outside the field at " + p;
            }
            if (k[0] > -OWN_SIDE_MARGIN_IN) {
                return "on the opponent side at " + p;
            }
        }
        if (overlapsBox(c, 0, 0, HIVE_HALF_X_IN + HIVE_MARGIN_IN, HIVE_HALF_Y_IN + HIVE_MARGIN_IN)) {
            return "hits the HIVE frame at " + p;
        }
        for (FieldMap.Place f : FieldMap.Place.values()) {
            if (f.label.startsWith("Flower") && distanceToRobot(p, f.x, f.y) < FLOWER_RADIUS_IN + FLOWER_MARGIN_IN) {
                return "hits " + f.label + " at " + p;
            }
        }
        if (partnerSlot != null) {
            Pose s = AutoField.slot(partnerSlot);
            if (overlapsBox(c, s.x, s.y, HALF_LENGTH_IN + PARTNER_MARGIN_IN, HALF_WIDTH_IN + PARTNER_MARGIN_IN)) {
                return "hits the partner in slot " + partnerSlot + " at " + p;
            }
        }
        return null;
    }

    /**
     * Checks the robot along every leg (straight line, heading turning the short way, as Pedro's linear()). A leg into a
     * wall is checked up to where the wall stops the robot.
     */
    public static String routeProblem(Pose from, List<Leg> legs, Slot partnerSlot) {
        Pose p = from;
        for (Leg l : legs) {
            Pose to = l.stop;
            double dh = normalizeDeg(to.headingDeg - p.headingDeg);
            int n = (int) Math.max(4, Math.ceil(Math.max(p.distanceTo(to), Math.abs(dh) / 2)));
            for (int i = 0; i <= n; i++) {
                double t = (double) i / n;
                Pose q = new Pose(p.x + t * (to.x - p.x), p.y + t * (to.y - p.y), p.headingDeg + t * dh);
                String problem = footprintProblem(q, partnerSlot);
                if (problem != null) {
                    return l.label + ": " + problem;
                }
            }
            p = to;
        }
        return null;
    }

    /** Separating-axis test: robot corners vs an axis-aligned box (half sizes hx, hy) at (bx, by). */
    static boolean overlapsBox(double[][] c, double bx, double by, double hx, double hy) {
        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (double[] k : c) {
            minX = Math.min(minX, k[0]);
            maxX = Math.max(maxX, k[0]);
            minY = Math.min(minY, k[1]);
            maxY = Math.max(maxY, k[1]);
        }
        if (maxX <= bx - hx || minX >= bx + hx || maxY <= by - hy || minY >= by + hy) {
            return false;
        }
        double[][] box = {{bx - hx, by - hy}, {bx + hx, by - hy}, {bx + hx, by + hy}, {bx - hx, by + hy}};
        for (int e = 0; e < 2; e++) {                      // the robot's two edge directions
            double ax = c[e == 0 ? 2 : 1][0] - c[0][0], ay = c[e == 0 ? 2 : 1][1] - c[0][1];
            double r0 = Double.MAX_VALUE, r1 = -Double.MAX_VALUE, b0 = Double.MAX_VALUE, b1 = -Double.MAX_VALUE;
            for (double[] k : c) {
                double d = k[0] * ax + k[1] * ay;
                r0 = Math.min(r0, d);
                r1 = Math.max(r1, d);
            }
            for (double[] k : box) {
                double d = k[0] * ax + k[1] * ay;
                b0 = Math.min(b0, d);
                b1 = Math.max(b1, d);
            }
            if (r1 <= b0 || b1 <= r0) {
                return false;
            }
        }
        return true;
    }

    /** Distance from a point to the robot rectangle (0 inside). */
    static double distanceToRobot(Pose p, double px, double py) {
        double h = Math.toRadians(p.headingDeg);
        double dx = px - p.x, dy = py - p.y;
        double along = dx * Math.cos(h) + dy * Math.sin(h);
        double side = -dx * Math.sin(h) + dy * Math.cos(h);
        double ox = Math.max(0, Math.abs(along) - HALF_LENGTH_IN);
        double oy = Math.max(0, Math.abs(side) - HALF_WIDTH_IN);
        return Math.hypot(ox, oy);
    }
}

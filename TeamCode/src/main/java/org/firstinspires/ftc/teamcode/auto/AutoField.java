package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.fieldtour.FieldMap;

/**
 * AUTO places, measured from the official field CAD (field-cad-step.step, read 7 Nov 2026). No FTC SDK dependency.
 *
 * Frame = {@link FieldMap}: origin at the field centre, +X toward the BLUE alliance wall, +Y = left as seen from the RED
 * alliance station (toward the rear wall; the audience wall is -Y), heading in degrees CCW from +X. The robot FRONT is
 * the intake side; the launcher and camera are at the back.
 *
 * Everything is written for RED. The field is point-symmetric: BLUE is RED turned 180 deg about the centre
 * ({@link Pose#mirror()}).
 *
 * CAD facts (RED):
 * - LOADING ZONE: on the RED wall, x -70.5..-59.1, y +24.9..+45.6 (23 x 11 in, Section 9).
 * - GARDEN: on the audience wall in the RED corner; 4 POLLEN in a line against the wall, centres y -69.3,
 *   x -69.3 / -66.4 / -63.5 / -60.6.
 * - Upward CELL at the start = RED AUDIENCE CELL, x -23.3..-2.1 (centre -12.7), y -1.6..-20.1; its opening faces the
 *   AUDIENCE wall. The other RED CELL (down at the start) faces the rear wall.
 * - FLOWERs (centres) from {@link FieldMap.Place}, about 7.4 in across.
 */
public final class AutoField {
    private AutoField() {
    }

    public enum Alliance { RED, BLUE }

    /** Legal starts (G304: touching the wall, own side, not in the LOADING ZONE, not touching a FLOWER). */
    public enum Start {
        CELL_WALL("CELL wall: side wall in front of our up CELL (shoots from here)"),
        ALLIANCE_WALL("ALLIANCE wall: between the FLOWER and the LOADING ZONE");

        public final String label;

        Start(String label) {
            this.label = label;
        }

        public Start next() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    /** Two parking places in our LOADING ZONE, so both robots of the ALLIANCE fit. */
    public enum Slot {
        A("A = CORNER end of the LOADING ZONE"),
        B("B = MIDDLE end of the LOADING ZONE (toward the FLOWER)");

        public final String label;

        Slot(String label) {
            this.label = label;
        }

        public Slot other() {
            return this == A ? B : A;
        }
    }

    /** Field pose: inches, heading in degrees CCW from +X. */
    public static final class Pose {
        public final double x, y, headingDeg;

        public Pose(double x, double y, double headingDeg) {
            this.x = x;
            this.y = y;
            this.headingDeg = normalizeDeg(headingDeg);
        }

        /** The same place for the other ALLIANCE (180 deg about the field centre). */
        public Pose mirror() {
            return new Pose(-x, -y, headingDeg + 180);
        }

        public Pose forAlliance(Alliance a) {
            return a == Alliance.RED ? this : mirror();
        }

        public double distanceTo(Pose o) {
            return Math.hypot(o.x - x, o.y - y);
        }

        @Override
        public String toString() {
            return String.format("(%.1f, %.1f, %.0f deg)", x, y, headingDeg);
        }
    }

    // ---------------- Robot ----------------

    /** Robot centre to the intake bumper and to the launcher back. CALIBRATE: measure the robot (18 in max). */
    public static final double HALF_LENGTH_IN = 9.0;
    /** Robot centre to its side. CALIBRATE. */
    public static final double HALF_WIDTH_IN = 9.0;

    // ---------------- Field (CAD, RED) ----------------

    public static final double WALL_IN = FieldMap.FIELD_HALF_IN;
    public static final double ZONE_INNER_X = -59.1;
    public static final double ZONE_Y_MIN = 24.9;
    public static final double ZONE_Y_MAX = 45.6;
    public static final double CELL_X = -12.7;
    public static final double GARDEN_BALL_Y = -69.3;
    public static final double[] GARDEN_BALL_X = {-69.3, -66.4, -63.5, -60.6};
    public static final double FLOWER_RADIUS_IN = 3.7;
    public static final double HIVE_HALF_X_IN = FieldMap.HIVE_FRAME_HALF_X_IN;
    public static final double HIVE_HALF_Y_IN = FieldMap.HIVE_FRAME_HALF_Y_IN;

    // ---------------- Plan (RED) ----------------

    /** Heading with the intake toward the audience wall (launcher + camera toward the up CELL). */
    public static final double FACE_AUDIENCE = -90;
    /** Heading with the intake toward the RED alliance wall. */
    public static final double FACE_ALLIANCE_WALL = 180;

    /** Intake bumper against the audience wall, straight in front of the up CELL (the shot spot is 2 in out). */
    public static final Pose START_CELL_WALL = new Pose(CELL_X, -WALL_IN + HALF_LENGTH_IN, FACE_AUDIENCE);
    /**
     * Intake bumper against the RED wall, centred on the wall's middle line: 10.7 in from the FLOWER (y -23.4) and 2.9 in
     * from a robot parked in slot B.
     */
    public static final Pose START_ALLIANCE_WALL = new Pose(-WALL_IN + HALF_LENGTH_IN, 0.0, FACE_ALLIANCE_WALL);

    /** Straight-on shot (3 Nov shot review): intake 2 in off the audience wall, launcher facing the CELL opening. */
    public static final double SHOT_WALL_GAP_IN = 2.0;
    public static final Pose SHOT = new Pose(CELL_X, -WALL_IN + HALF_LENGTH_IN + SHOT_WALL_GAP_IN, FACE_AUDIENCE);
    /**
     * The camera aim at the shot spot never turns more than this from the SHOT heading. Our CELL is straight ahead there
     * (logged bearings within +/-1.1 deg), and 2 in off the wall a robot corner reaches the wall at ~15 deg, at ~11 deg
     * when the robot is 0.5 in closer (AutoPlannerTest checks it).
     */
    public static final double AIM_MAX_YAW_DEG = 8.0;

    /**
     * Lane beside the HIVE frame used to drive between the audience side and the LOADING ZONE: the middle of the gap
     * between the HIVE frame (x -24.75) and a partner parked in slot B (x -50.5), 3.9 in from each.
     */
    public static final double LANE_X = (-HIVE_HALF_X_IN + (-WALL_IN + 2 * HALF_LENGTH_IN + 2.0)) / 2;
    /** Where the robot turns on the audience side (far enough from the wall for its corners). */
    public static final double TURN_Y = -48.0;

    /** Park: intake 2 in off the RED wall (LEAVE still counts), PARK_IN_ZONE_IN of the robot inside the zone. */
    public static final double PARK_WALL_GAP_IN = 2.0;
    public static final double PARK_IN_ZONE_IN = 5.0;
    public static final double PARK_X = -WALL_IN + HALF_LENGTH_IN + PARK_WALL_GAP_IN;
    public static final double SLOT_A_Y = ZONE_Y_MAX + HALF_WIDTH_IN - PARK_IN_ZONE_IN;
    public static final double SLOT_B_Y = ZONE_Y_MIN - HALF_WIDTH_IN + PARK_IN_ZONE_IN;

    /**
     * GARDEN sweep: intake toward the RED wall, robot side GARDEN_SIDE_GAP_IN off the audience wall. CALIBRATE on the
     * field: the intake must reach POLLEN whose centres are 1.2 in from the wall.
     */
    public static final double GARDEN_SIDE_GAP_IN = 1.5;
    public static final double GARDEN_LANE_Y = -WALL_IN + HALF_WIDTH_IN + GARDEN_SIDE_GAP_IN;
    public static final double GARDEN_SWEEP_START_X = -36.0;
    /** The slow part starts with the intake bumper 5 in before the first POLLEN (x -60.6). */
    public static final double GARDEN_SWEEP_SLOW_X = GARDEN_BALL_X[GARDEN_BALL_X.length - 1] + HALF_LENGTH_IN + 5.0;
    /** Intake bumper touching the RED wall at the GARDEN corner (the corner POLLEN is against that wall). */
    public static final double GARDEN_CORNER_X = -WALL_IN + HALF_LENGTH_IN;
    /**
     * The sweep and every move in at the corner aim this far PAST the wall, so the intake bumper really reaches it (team,
     * 13 Nov: it stopped ~2 in short of the wall when it aimed 1 in short: the robot is often 1-2 in off where it
     * thinks it is, e.g. from the start placement). The wall stops the robot: the sweep counts as done once the robot
     * stays still (AutoPlanner.WALL_SETTLE_SEC), a move in ends after BioBuzzPedroAutoBase.GARDEN_WIGGLE_MOVE_SEC.
     */
    public static final double GARDEN_PUSH_IN = 2.0;
    /** Where the sweep and the moves in aim: GARDEN_PUSH_IN past the wall (the robot stops at GARDEN_CORNER_X). */
    public static final double GARDEN_SWEEP_END_X = GARDEN_CORNER_X - GARDEN_PUSH_IN;
    /**
     * At the GARDEN corner (team, 12 Nov: the intake takes the POLLEN when the robot comes 1-2 in back and forth): the
     * robot backs off this far along the wall from where the wall (or a POLLEN) stopped it, then pushes in again, over
     * and over while the intake runs. CALIBRATE: 1-2 in.
     */
    public static final double GARDEN_WIGGLE_IN = 2.0;
    /** Back-off point between the shot spot and the GARDEN, where the robot turns. */
    public static final Pose GARDEN_TURN = new Pose(-28.0, -52.0, FACE_ALLIANCE_WALL);

    public static Pose start(Start s) {
        return s == Start.CELL_WALL ? START_CELL_WALL : START_ALLIANCE_WALL;
    }

    public static Pose slot(Slot s) {
        return new Pose(PARK_X, s == Slot.A ? SLOT_A_Y : SLOT_B_Y, FACE_ALLIANCE_WALL);
    }

    public static double normalizeDeg(double d) {
        d %= 360;
        if (d > 180) {
            d -= 360;
        } else if (d <= -180) {
            d += 360;
        }
        return d;
    }
}

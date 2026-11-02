package org.firstinspires.ftc.teamcode.fieldtour;

/**
 * The BIOBUZZ field in inches, measured from the official field CAD (field_cad/x/field-cad-step.step).
 *
 * Frame: origin = field centre; +X toward the BLUE alliance wall; +Y = LEFT as seen standing at the RED alliance
 * station; heading counter-clockwise from +X (0 = facing the BLUE wall). No FTC SDK dependency (laptop-tested).
 */
public final class FieldMap {
    private FieldMap() {
    }

    /** Inside faces of the perimeter walls (+-1791 mm); the 6 x 6 soft tiles fill this square. */
    public static final double FIELD_HALF_IN = 70.5;
    public static final int TILES = 6;
    public static final double TILE_IN = 2 * FIELD_HALF_IN / TILES;

    /** Robot footprint half-width (18 in max) and the gap kept to walls / obstacles. */
    public static final double ROBOT_HALF_IN = 9.0;
    public static final double CLEARANCE_IN = 3.0;

    /** HIVE support A-frame on the floor ("am-5854: Frame", x -628..629 mm, z -495..495 mm). Both HIVEs hang above it. */
    public static final double HIVE_FRAME_HALF_X_IN = 24.75;
    public static final double HIVE_FRAME_HALF_Y_IN = 19.5;

    /** Robot centre must stay inside this square (walls minus robot half-width minus clearance). */
    public static final double DRIVE_LIMIT_IN = FIELD_HALF_IN - ROBOT_HALF_IN - CLEARANCE_IN;

    /**
     * Robot centre must stay outside this rectangle: the A-frame grown by the robot half-width + clearance. The tour
     * never turns (it keeps the start heading, which is square to the field), so the footprint stays axis-aligned.
     */
    public static final double KEEP_OUT_HALF_X_IN = HIVE_FRAME_HALF_X_IN + ROBOT_HALF_IN + CLEARANCE_IN;
    public static final double KEEP_OUT_HALF_Y_IN = HIVE_FRAME_HALF_Y_IN + ROBOT_HALF_IN + CLEARANCE_IN;

    /** A start tile may be right next to a wall (robot placed by hand); only the laps keep CLEARANCE_IN. */
    public static final double START_LIMIT_IN = FIELD_HALF_IN - ROBOT_HALF_IN;

    /** Named places (centres, inches) for the screen. Flowers are listed but only avoided when on the field. */
    public enum Place {
        BLUE_HIVE("Blue HIVE", 12.8, -2.6),
        RED_HIVE("Red HIVE", -12.7, 2.6),
        FLOWER_1("Flower 1", -23.4, 68.1),
        FLOWER_2("Flower 2", 23.4, -68.1),
        FLOWER_3("Flower 3", -68.1, -23.4),
        FLOWER_4("Flower 4", 68.1, 23.4);

        public final String label;
        public final double x, y;

        Place(String label, double x, double y) {
            this.label = label;
            this.x = x;
            this.y = y;
        }
    }

    /** Which way the robot FRONT (intake side) faces at the start. */
    public enum Facing {
        BLUE_WALL("BLUE wall", 0), LEFT_SIDE("left side", 90), RED_WALL("RED wall", 180), RIGHT_SIDE("right side", 270);

        public final String label;
        public final double headingDeg;

        Facing(String label, double headingDeg) {
            this.label = label;
            this.headingDeg = headingDeg;
        }

        public Facing next() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    /** Centre of tile (column 1..6 from the RED wall, row 1..6 from the right side as seen from the RED station). */
    public static double tileCentreX(int column) {
        return -FIELD_HALF_IN + TILE_IN * (column - 0.5);
    }

    public static double tileCentreY(int row) {
        return -FIELD_HALF_IN + TILE_IN * (row - 0.5);
    }

    public static boolean insideDriveLimit(double x, double y) {
        return Math.abs(x) <= DRIVE_LIMIT_IN + 1e-9 && Math.abs(y) <= DRIVE_LIMIT_IN + 1e-9;
    }

    public static boolean insideKeepOut(double x, double y) {
        return Math.abs(x) < KEEP_OUT_HALF_X_IN && Math.abs(y) < KEEP_OUT_HALF_Y_IN;
    }

    /** Null when the robot can start (and tour) from here, otherwise why not. */
    public static String startProblem(double x, double y) {
        if (insideKeepOut(x, y)) {
            return "that tile is under the HIVE frame";
        }
        if (Math.abs(x) > START_LIMIT_IN || Math.abs(y) > START_LIMIT_IN) {
            return "the robot would be outside the field";
        }
        return null;
    }

    /** True when the straight segment a->b passes through the HIVE keep-out (Liang-Barsky clip, tiny tolerance). */
    public static boolean segmentHitsKeepOut(double ax, double ay, double bx, double by) {
        double hx = KEEP_OUT_HALF_X_IN - 1e-6, hy = KEEP_OUT_HALF_Y_IN - 1e-6;
        double dx = bx - ax, dy = by - ay;
        double t0 = 0, t1 = 1;
        double[] p = {-dx, dx, -dy, dy};
        double[] q = {ax + hx, hx - ax, ay + hy, hy - ay};
        for (int i = 0; i < 4; i++) {
            if (p[i] == 0) {
                if (q[i] < 0) {
                    return false;
                }
            } else {
                double r = q[i] / p[i];
                if (p[i] < 0) {
                    t0 = Math.max(t0, r);
                } else {
                    t1 = Math.min(t1, r);
                }
                if (t0 > t1) {
                    return false;
                }
            }
        }
        return t1 - t0 > 1e-9;
    }

    /** Nearest named place to (x, y), for the "where am I" screen line. */
    public static Place nearest(double x, double y) {
        Place best = null;
        double bestD = Double.MAX_VALUE;
        for (Place p : Place.values()) {
            double d = Math.hypot(p.x - x, p.y - y);
            if (d < bestD) {
                bestD = d;
                best = p;
            }
        }
        return best;
    }
}

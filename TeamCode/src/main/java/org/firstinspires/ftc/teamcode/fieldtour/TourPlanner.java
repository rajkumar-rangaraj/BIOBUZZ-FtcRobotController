package org.firstinspires.ftc.teamcode.fieldtour;

import java.util.ArrayList;
import java.util.List;

/**
 * Roborock-style tour: from HOME to the outer lap (near the walls), round it once, in to the inner lap (round the
 * HIVE frame), round it once, then back out and home. Straight segments only; every segment stays inside the drive
 * limit and outside the HIVE keep-out. Points are field inches (see {@link FieldMap}). No FTC SDK dependency.
 */
public final class TourPlanner {
    private TourPlanner() {
    }

    public static final class Point {
        public final double x, y;
        public final String label;

        Point(double x, double y, String label) {
            this.x = x;
            this.y = y;
            this.label = label;
        }
    }

    /** Outer lap: as close to the walls as allowed. */
    public static final double OUTER_HALF = FieldMap.DRIVE_LIMIT_IN;
    /** Inner lap: just outside the HIVE keep-out. */
    public static final double INNER_HALF_X = FieldMap.KEEP_OUT_HALF_X_IN + 2;
    public static final double INNER_HALF_Y = FieldMap.KEEP_OUT_HALF_Y_IN + 2;

    /** Waypoints after HOME, ending at HOME. */
    public static List<Point> plan(double homeX, double homeY) {
        if (FieldMap.startProblem(homeX, homeY) != null) {
            throw new IllegalArgumentException(FieldMap.startProblem(homeX, homeY));
        }
        List<Point> out = new ArrayList<>();
        double[] e = nearestOnRect(homeX, homeY, OUTER_HALF, OUTER_HALF);
        add(out, e[0], e[1], "outer lap");
        lap(out, e, OUTER_HALF, OUTER_HALF, "outer");
        double[] in = nearestOnRect(e[0], e[1], INNER_HALF_X, INNER_HALF_Y);
        add(out, in[0], in[1], "inner lap");
        lap(out, in, INNER_HALF_X, INNER_HALF_Y, "inner");
        add(out, e[0], e[1], "back out");
        out.add(new Point(homeX, homeY, "HOME"));             // always the last point, even if very close
        return out;
    }

    /** Corners (counter-clockwise) then back to the start point p, which lies on the rectangle edge. */
    private static void lap(List<Point> out, double[] p, double hx, double hy, String name) {
        double[][] c = {{-hx, -hy}, {hx, -hy}, {hx, hy}, {-hx, hy}};
        int edge = edgeOf(p, hx, hy);
        for (int i = 1; i <= 4; i++) {
            double[] k = c[(edge + i) % 4];
            add(out, k[0], k[1], name + " corner " + i);
        }
        add(out, p[0], p[1], name + " lap done");
    }

    /** Edge index k: the edge from corner k to corner k+1 (counter-clockwise) that p lies on. */
    static int edgeOf(double[] p, double hx, double hy) {
        double eps = 1e-6;
        if (Math.abs(p[1] + hy) < eps && p[0] < hx - eps) {
            return 0;                                           // bottom, going +x
        }
        if (Math.abs(p[0] - hx) < eps && p[1] < hy - eps) {
            return 1;                                           // right, going +y
        }
        if (Math.abs(p[1] - hy) < eps && p[0] > -hx + eps) {
            return 2;                                           // top, going -x
        }
        return 3;                                               // left, going -y
    }

    /** Nearest point on the boundary of the centred rectangle (half sizes hx, hy) to (x, y). */
    static double[] nearestOnRect(double x, double y, double hx, double hy) {
        double cx = Math.max(-hx, Math.min(hx, x));
        double cy = Math.max(-hy, Math.min(hy, y));
        boolean inside = Math.abs(x) < hx && Math.abs(y) < hy;
        if (!inside) {
            return new double[]{cx, cy};
        }
        double dl = x + hx, dr = hx - x, db = y + hy, dt = hy - y;     // inside: push to the closest edge
        double m = Math.min(Math.min(dl, dr), Math.min(db, dt));
        if (m == dl) {
            return new double[]{-hx, y};
        }
        if (m == dr) {
            return new double[]{hx, y};
        }
        if (m == db) {
            return new double[]{x, -hy};
        }
        return new double[]{x, hy};
    }

    private static void add(List<Point> out, double x, double y, String label) {
        Point last = out.isEmpty() ? null : out.get(out.size() - 1);
        if (last != null && Math.hypot(last.x - x, last.y - y) < 0.5) {
            return;                                             // skip zero-length segments
        }
        out.add(new Point(x, y, label));
    }

    /** Total driving distance HOME -> ... -> HOME, inches. */
    public static double length(double homeX, double homeY, List<Point> pts) {
        double d = 0, px = homeX, py = homeY;
        for (Point p : pts) {
            d += Math.hypot(p.x - px, p.y - py);
            px = p.x;
            py = p.y;
        }
        return d;
    }
}

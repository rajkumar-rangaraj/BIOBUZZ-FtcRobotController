package org.firstinspires.ftc.teamcode.biobuzz.pp;

/**
 * The robot's pose in the ALLIANCE frame (see {@link PpConstants}), from the Pinpoint at loop rate.
 *
 * The Pinpoint is never written: its raw reading (x forward / y left of wherever it was last reset, heading CCW) is
 * turned into the field frame by an offset (rotation + shift) kept here. Starting the pose, a camera fix or a new
 * "intake away from me" heading only change that offset, so they can't race the Pinpoint's own I2C updates.
 *
 * Raw readings of the last 64 loops are kept (about 1.6 s at 25 ms a loop, more at today's ~50 ms), so a camera frame
 * taken a few loops ago is compared with where the robot was when the frame was taken ({@link #fieldAt}). A raw jump
 * the robot can't make (a Pinpoint restart reads its origin) drops the pose until it is set again and the next fix.
 * A pose set before the first good reading (START during a bad I2C read or the gyro calibration) waits and is applied
 * on the first good one.
 * No FTC SDK dependency: unit-tested (FieldPoseTest).
 */
public class FieldPose {

    private static final int HISTORY = 64;
    /** The jump check allows the robot's top speed / turn rate over the real gap, up to this long (bad reads, stalls). */
    static final double JUMP_DT_CAP_SEC = 1.0;

    private final double maxSpeedIps;
    private final double jumpMarginIn;
    private final double maxTurnDps;
    private final double turnMarginDeg;

    private double thetaDeg = Double.NaN;       // field heading = raw heading + theta
    private double dx, dy;                      // field xy = R(theta) raw xy + (dx, dy)
    private boolean positionKnown = false;

    private final double[] ht = new double[HISTORY], hx = new double[HISTORY], hy = new double[HISTORY],
            hh = new double[HISTORY];
    private int count = 0, head = -1;           // head = newest sample
    private boolean rawOk = false;
    private String event = null;
    private double pendX = Double.NaN, pendY = Double.NaN, pendH = Double.NaN;   // set before the first good reading
    private double pendT = Double.NaN;          // ... at this time (the last update's)
    private double lastUpdateT = Double.NaN;
    /** A start waiting longer than this for a good reading is dropped: the robot may have moved meanwhile. */
    static final double PENDING_MAX_SEC = 1.0;

    public FieldPose(double maxSpeedIps, double jumpMarginIn, double maxTurnDps, double turnMarginDeg) {
        this.maxSpeedIps = maxSpeedIps;
        this.jumpMarginIn = jumpMarginIn;
        this.maxTurnDps = maxTurnDps;
        this.turnMarginDeg = turnMarginDeg;
    }

    public static FieldPose standard() {
        return new FieldPose(PpConstants.POSE_MAX_SPEED_IPS, PpConstants.POSE_JUMP_MARGIN_IN,
                org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.HEADING_MAX_RATE_DPS,
                org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.HEADING_GLITCH_MARGIN_DEG);
    }

    /**
     * Call once per loop with the fresh Pinpoint reading.
     *
     * @param t  seconds (the same clock as camera frame times)
     * @param ok the Pinpoint reported READY (a bad read is ignored; the last good reading stays)
     */
    public void update(double t, double rawX, double rawY, double rawHeadingDeg, boolean ok) {
        event = null;
        lastUpdateT = t;
        rawOk = ok && !Double.isNaN(rawX) && !Double.isNaN(rawY) && !Double.isNaN(rawHeadingDeg);
        if (!rawOk) {
            return;
        }
        if (count > 0) {
            double dt = Math.max(0, Math.min(t - ht[head], JUMP_DT_CAP_SEC));
            double moved = Math.hypot(rawX - hx[head], rawY - hy[head]);
            double turned = Math.abs(norm(rawHeadingDeg - hh[head]));
            // A restart near its origin may keep x / y and reset only the heading: check both
            if (moved > maxSpeedIps * dt + jumpMarginIn || turned > maxTurnDps * dt + turnMarginDeg) {
                if (headingKnown()) {
                    event = String.format("POSE_JUMP raw moved %.1f in / turned %.0f deg in %.3f s: pose dropped"
                            + " until the heading is set again and the next fix", moved, turned, dt);
                }
                positionKnown = false;
                thetaDeg = Double.NaN;              // a restarted Pinpoint's heading starts over too
                count = 0;                          // old samples are in the old raw frame
            }
        }
        head = (head + 1) % HISTORY;
        ht[head] = t;
        hx[head] = rawX;
        hy[head] = rawY;
        hh[head] = rawHeadingDeg;
        count = Math.min(count + 1, HISTORY);
        if (!Double.isNaN(pendH)) {                 // a start set while there was no reading
            double px = pendX, py = pendY, ph = pendH, pt = pendT;
            boolean expired = !Double.isNaN(pt) && t - pt > PENDING_MAX_SEC;
            pendX = pendY = pendH = pendT = Double.NaN;
            if (expired) {
                event = String.format("START_EXPIRED no good Pinpoint reading for %.1f s after the start: pose"
                        + " dropped, the camera places the robot", t - pt);
            } else if (Double.isNaN(px)) {
                setHeading(ph);
            } else {
                setPose(px, py, ph);
            }
        }
    }

    /** A pose or heading set before the first good reading is waiting for it. */
    public boolean pending() {
        return !Double.isNaN(pendH);
    }

    /** "Pose events" from the last update (POSE_JUMP), null if none. */
    public String event() {
        return event;
    }

    /** The Pinpoint gave a good reading this loop. */
    public boolean rawOk() {
        return rawOk;
    }

    public boolean headingKnown() {
        return !Double.isNaN(thetaDeg) && count > 0;
    }

    public boolean positionKnown() {
        return positionKnown && headingKnown();
    }

    /** Sets the field heading now; a known (or waiting) position stays where it is. */
    public void setHeading(double fieldHeadingDeg) {
        if (count == 0) {
            if (Double.isNaN(pendH)) {
                pendT = lastUpdateT;                // the clock starts when the start was set
            }
            pendH = fieldHeadingDeg;                // applied on the first good reading, with a waiting x / y if any
            return;
        }
        double[] keep = positionKnown() ? new double[]{x(), y()} : null;
        thetaDeg = fieldHeadingDeg - hh[head];
        if (keep != null) {
            placeAt(keep[0], keep[1]);
        }
    }

    /** Sets the whole pose now (a known start, AUTO's hand-off); before the first good reading it waits for it. */
    public void setPose(double x, double y, double headingDeg) {
        if (count == 0) {
            pendX = x;
            pendY = y;
            pendH = headingDeg;
            pendT = lastUpdateT;
            return;
        }
        thetaDeg = headingDeg - hh[head];
        placeAt(x, y);
    }

    /** Sets the position now, keeping the heading (a camera fix when the position was unknown). */
    public void setPosition(double x, double y) {
        if (headingKnown()) {
            placeAt(x, y);
        }
    }

    /** Moves the whole pose (now and history) by (ddx, ddy): a camera correction. */
    public void shift(double ddx, double ddy) {
        dx += ddx;
        dy += ddy;
    }

    /**
     * Measured minus posed position at time t (a camera fix taken then), null when the heading isn't known or t is older
     * than the history. Works while the position is still unknown (then it is the shift that would place it).
     */
    public double[] residualAt(double t, double measX, double measY) {
        double[] p = fieldAt(t);
        return p == null ? null : new double[]{measX - p[0], measY - p[1]};
    }

    /** Places the pose so it was at (measX, measY) at time t. Returns false when t is out of the history. */
    public boolean snapAt(double t, double measX, double measY) {
        double[] r = residualAt(t, measX, measY);
        if (r == null) {
            return false;
        }
        shift(r[0], r[1]);
        positionKnown = true;
        return true;
    }

    private void placeAt(double x, double y) {
        double c = Math.cos(Math.toRadians(thetaDeg)), s = Math.sin(Math.toRadians(thetaDeg));
        dx = x - (c * hx[head] - s * hy[head]);
        dy = y - (s * hx[head] + c * hy[head]);
        positionKnown = true;
    }

    public double x() {
        return toField(hx[head], hy[head])[0];
    }

    public double y() {
        return toField(hx[head], hy[head])[1];
    }

    /** Field heading, CCW deg in (-180, 180]. */
    public double headingDeg() {
        return norm(hh[head] + thetaDeg);
    }

    /** Time of the newest reading. */
    public double time() {
        return count == 0 ? Double.NaN : ht[head];
    }

    private double[] toField(double rx, double ry) {
        double c = Math.cos(Math.toRadians(thetaDeg)), s = Math.sin(Math.toRadians(thetaDeg));
        return new double[]{c * rx - s * ry + dx, s * rx + c * ry + dy};
    }

    /**
     * The field pose {x, y, heading} at an earlier time (linear between readings), null when that is older than the
     * history or nothing is known. A time after the newest reading gives the newest pose.
     */
    public double[] fieldAt(double t) {
        if (!headingKnown()) {
            return null;
        }
        double[] raw = rawAt(t);
        if (raw == null) {
            return null;
        }
        double[] xy = toField(raw[0], raw[1]);
        return new double[]{xy[0], xy[1], norm(raw[2] + thetaDeg)};
    }

    private double[] rawAt(double t) {
        if (count == 0) {
            return null;
        }
        if (t >= ht[head]) {
            return new double[]{hx[head], hy[head], hh[head]};
        }
        int newer = head;
        for (int k = 1; k < count; k++) {
            int i = (head - k + HISTORY) % HISTORY;
            if (ht[i] <= t) {
                double span = ht[newer] - ht[i];
                double f = span <= 0 ? 1 : (t - ht[i]) / span;
                double dh = norm(hh[newer] - hh[i]);
                return new double[]{hx[i] + f * (hx[newer] - hx[i]), hy[i] + f * (hy[newer] - hy[i]), hh[i] + f * dh};
            }
            newer = i;
        }
        return null;
    }

    /** Turn rate (CCW deg/s) over about the last 0.1 s, 0 if unknown. */
    public double turnRateDps() {
        int i = older(0.1);
        return i < 0 ? 0 : norm(hh[head] - hh[i]) / (ht[head] - ht[i]);
    }

    /** Speed (in/s) over about the last 0.1 s, 0 if unknown. */
    public double speedIps() {
        int i = older(0.1);
        return i < 0 ? 0 : Math.hypot(hx[head] - hx[i], hy[head] - hy[i]) / (ht[head] - ht[i]);
    }

    /** Index of the newest sample at least {@code span} s older than the newest one (or the oldest), -1 if none. */
    private int older(double span) {
        if (count < 2) {
            return -1;
        }
        int pick = -1;
        for (int k = 1; k < count; k++) {
            int i = (head - k + HISTORY) % HISTORY;
            pick = i;
            if (ht[head] - ht[i] >= span) {
                break;
            }
        }
        return pick >= 0 && ht[head] > ht[pick] ? pick : -1;
    }

    public static double norm(double deg) {
        double a = deg % 360;
        if (a > 180) {
            a -= 360;
        } else if (a <= -180) {
            a += 360;
        }
        return a;
    }
}

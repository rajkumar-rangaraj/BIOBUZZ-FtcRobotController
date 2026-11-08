package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.Objects;

/**
 * Picks the flywheel's base speed each loop (TeleOp adds the driver's D-pad trim on top):
 * <ul>
 *   <li>AUTO: from the camera's floor distance to our CELL, by a distance table taken from the drivers' shots. It moves
 *   only when the table speed differs from the current one by more than the hysteresis (camera jitter can't make it
 *   hunt) and is kept for keepSec after the CELL drops out of view. It follows ONE CELL: when two of our CELLs are in
 *   view the camera's best CELL can flip between them every few frames, so another CELL is taken only after the
 *   followed one has been out of view for switchSec.</li>
 *   <li>PRESET: NEAR / FAR when there is no camera distance (or the auto speed is switched off in INIT).</li>
 *   <li>MANUAL: the driver picked NEAR / FAR with the D-pad; the camera is ignored until AUTO is asked for again.</li>
 * </ul>
 * While FIRE is held the base speed is latched: a new target resets the ready gate (LauncherGate) mid-burst. A driver
 * choice (setManual) releases the latch. No FTC SDK dependency, so it is unit-tested on the laptop.
 */
public class ShotSpeed {

    public enum Source { AUTO, AUTO_KEPT, PRESET, MANUAL }

    private final double[] distIn;
    private final double[] speed;
    private final double keepSec;
    private final double hysteresis;
    private final double switchSec;

    private boolean autoEnabled = true;
    private boolean manual = false;
    private double current = Double.NaN;                        // auto speed (rounded to 5 t/s), NaN = none
    private double lastSeen = -1e9;                             // any of our CELLs
    private boolean locked = false;                             // following a CELL
    private Object lockedCell = null;
    private double lockedSeen = -1e9;
    private double lastDistance = Double.NaN;
    private double latched = Double.NaN;                        // base speed held while FIRE is held
    private Source latchedSource = Source.PRESET;
    private Source source = Source.PRESET;

    /**
     * @param distIn     table distances (in), strictly increasing
     * @param speed      flywheel speeds (t/s) at those distances; clamped to the end values outside the table
     * @param keepSec    the auto speed stays this long after the CELL was last seen
     * @param hysteresis the auto speed changes only when the table speed is more than this away from it (t/s)
     * @param switchSec  another CELL is followed only after the followed one has been out of view this long
     */
    public ShotSpeed(double[] distIn, double[] speed, double keepSec, double hysteresis, double switchSec) {
        if (distIn.length < 2 || distIn.length != speed.length) {
            throw new IllegalArgumentException("need at least 2 table points, distances and speeds alike");
        }
        for (int i = 1; i < distIn.length; i++) {
            if (!(distIn[i] > distIn[i - 1])) {
                throw new IllegalArgumentException("table distances must increase");
            }
        }
        this.distIn = distIn.clone();
        this.speed = speed.clone();
        this.keepSec = keepSec;
        this.hysteresis = hysteresis;
        this.switchSec = switchSec;
    }

    public static ShotSpeed standard() {
        return new ShotSpeed(BioBuzzConstants.AUTO_SPEED_DIST_IN, BioBuzzConstants.AUTO_SPEED_TPS,
                BioBuzzConstants.AUTO_SPEED_KEEP_SEC, BioBuzzConstants.AUTO_SPEED_HYSTERESIS,
                BioBuzzConstants.AUTO_SPEED_SWITCH_SEC);
    }

    /** Table speed at a floor distance (linear between points, the end values outside the table). */
    public double tableSpeed(double d) {
        if (d <= distIn[0]) {
            return speed[0];
        }
        int n = distIn.length;
        if (d >= distIn[n - 1]) {
            return speed[n - 1];
        }
        int i = 1;
        while (d > distIn[i]) {
            i++;
        }
        double f = (d - distIn[i - 1]) / (distIn[i] - distIn[i - 1]);
        return speed[i - 1] + f * (speed[i] - speed[i - 1]);
    }

    /** INIT switch: off = NEAR / FAR presets only (as before the auto speed). */
    public void setAutoEnabled(boolean on) {
        autoEnabled = on;
        latched = Double.NaN;
    }

    public boolean autoEnabled() {
        return autoEnabled;
    }

    /** The driver picked a preset (true) or asked for the camera speed again (false). Releases a FIRE latch. */
    public void setManual(boolean m) {
        manual = m;
        latched = Double.NaN;
    }

    public boolean manual() {
        return manual;
    }

    /**
     * @param t           seconds (any monotonic clock)
     * @param cell        which CELL the distance is to (any key, e.g. HiveVision.Cell); null when none is in view
     * @param distanceIn  camera floor distance to that CELL, NaN when none is in view
     * @param fire        FIRE held: the base speed doesn't change (see the class comment)
     * @param presetSpeed NEAR / FAR (or the SHOT SPOT speed) for PRESET and MANUAL
     * @return base flywheel speed (t/s), before the driver's trim
     */
    public double update(double t, Object cell, double distanceIn, boolean fire, double presetSpeed) {
        boolean seen = !Double.isNaN(distanceIn) && distanceIn > 0;
        if (seen) {
            lastSeen = t;
            if (!locked || Objects.equals(cell, lockedCell) || t - lockedSeen > switchSec) {
                locked = true;
                lockedCell = cell;
                lockedSeen = t;
                lastDistance = distanceIn;
                double c = 5 * Math.round(tableSpeed(distanceIn) / 5);
                if (Double.isNaN(current) || Math.abs(c - current) > hysteresis) {
                    current = c;
                }
            }
        } else if (t - lastSeen > keepSec) {
            current = Double.NaN;
            locked = false;
            lockedCell = null;
        }
        double base;
        Source src;
        if (autoEnabled && !manual && !Double.isNaN(current)) {
            base = current;
            src = t - lockedSeen <= switchSec ? Source.AUTO : Source.AUTO_KEPT;
        } else {
            base = presetSpeed;
            src = manual ? Source.MANUAL : Source.PRESET;
        }
        if (fire) {
            if (Double.isNaN(latched)) {
                latched = base;
                latchedSource = src;
            }
            source = latchedSource;
            return latched;
        }
        latched = Double.NaN;
        source = src;
        return base;
    }

    public Source source() {
        return source;
    }

    /** Last camera floor distance (in) to the followed CELL, NaN if never. */
    public double lastDistanceIn() {
        return lastDistance;
    }

    /** The CELL the auto speed follows (the key given to update), null if none. */
    public Object cell() {
        return lockedCell;
    }
}

package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * Decides when the flywheel may fire the next ball.
 *
 * The measured speed must stay within {@code +/- tolerance} of the target for {@code holdSec}. The hub reports the
 * speed in steps of about 20 ticks/s, so one lucky reading is not enough. The shot model (tools/shot_model.py) gives
 * about +/-4.7 % of speed for a ball to score with the printed hood, so the band must sit well inside that.
 *
 * If the speed doesn't settle in the band for {@code fallbackSec} (for example it droops on a weak battery, or it hunts
 * across the band edge), a fallback fires anyway while the speed is between {@code target - fallbackMargin} and
 * {@code target + tolerance}. So a ball is never held back for long while the speed is close, and never fired long
 * on an overspeed.
 * No FTC SDK dependency: unit-tested on the laptop.
 */
public class LauncherGate {

    private final double tolerance;
    private final double holdSec;
    private final double fallbackMargin;
    private final double fallbackSec;

    private double lastTarget = Double.NaN;
    private double inBandSince = Double.NaN;
    private double settledOrStart = Double.NaN;          // last time it was steady, or when the target was set
    private boolean fallback = false;

    public LauncherGate(double tolerance, double holdSec, double fallbackMargin, double fallbackSec) {
        this.tolerance = tolerance;
        this.holdSec = holdSec;
        this.fallbackMargin = fallbackMargin;
        this.fallbackSec = fallbackSec;
    }

    /** Robot settings from BioBuzzConstants. */
    public static LauncherGate standard() {
        return new LauncherGate(BioBuzzConstants.LAUNCHER_READY_TOLERANCE, BioBuzzConstants.LAUNCHER_READY_HOLD_SEC,
                BioBuzzConstants.LAUNCHER_FALLBACK_MARGIN, BioBuzzConstants.LAUNCHER_READY_FALLBACK_SEC);
    }

    /**
     * Safe to call several times in one loop.
     *
     * @param t        seconds on any monotonic clock
     * @param target   commanded flywheel speed, ticks/s ({@code <= 0} = launcher off)
     * @param velocity measured flywheel speed, ticks/s
     * @return true when a ball may be fed now
     */
    public boolean update(double t, double target, double velocity) {
        if (!(target > 0)) {
            reset();
            return false;
        }
        if (target != lastTarget) {                          // new preset or trim: the speed has to settle again
            lastTarget = target;
            inBandSince = Double.NaN;
            settledOrStart = t;
        }
        if (Math.abs(velocity - target) <= tolerance) {
            if (Double.isNaN(inBandSince)) {
                inBandSince = t;
            }
        } else {
            inBandSince = Double.NaN;
        }
        boolean steady = !Double.isNaN(inBandSince) && t - inBandSince >= holdSec;
        if (steady) {
            settledOrStart = t;
        }
        // Not settled for fallbackSec (drooping, or hunting across the band edge) but above the old margin and not
        // overspeed: fire anyway. Timed from the last settled moment, so a normal shot dip never triggers it.
        fallback = !steady && velocity >= target - fallbackMargin && velocity <= target + tolerance
                && t - settledOrStart >= fallbackSec;
        return steady || fallback;
    }

    /** True when the last {@link #update} allowed firing only through the fallback rule (log it: the speed is off). */
    public boolean usedFallback() {
        return fallback;
    }

    public void reset() {
        lastTarget = Double.NaN;
        inBandSince = Double.NaN;
        settledOrStart = Double.NaN;
        fallback = false;
    }
}

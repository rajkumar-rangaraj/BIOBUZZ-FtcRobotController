package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * Counts balls from flywheel slowdowns (AUTO's "all preloads gone" check, Tune's balls per pocket).
 * <p>
 * With the stiff flywheel loop (P 100) a dip of AUTO_SHOT_DIP_TPS is only two encoder steps (readings come in
 * 20 t/s steps), so a slowdown counts as a ball only when:
 * <ul>
 *   <li>the speed falls more than {@code dipTps} below the speed the flywheel really holds (a running average outside
 *       dips, never above the target): a flywheel that settles a little low doesn't make its normal readings dips;</li>
 *   <li>every reading stays below that line for at least {@code minDipSec}: one odd hub sample (~50 ms, maybe read
 *       2-3 times) is never a ball, a real ball keeps the speed down for 100 ms or more. Optionally (TeleOp, Tune:
 *       twoSamples) two low readings in a row that differ and are at least TWO_SAMPLES_MIN_SEC apart count too: they
 *       are two hub samples, not one read twice. With 55-60 ms loops (TeleOp while firing, 10 Nov logs) a real dip
 *       gives only two low readings ~55 ms apart: 6 of 34 shots weren't counted, and their LB + D-pad labels lost;</li>
 *   <li>it starts while the windmill feeds a pocket or within {@code windowSec} after it, at most {@code maxPerPocket}
 *       per pocket: spin-up, speed changes and anything between pockets never count.</li>
 * </ul>
 * A missed ball only costs AUTO an extra pocket; a phantom ball could leave a preload behind.
 * No FTC SDK dependency: unit-tested.
 */
public class BallCounter {

    private static final double BASELINE_ALPHA = 0.2;          // per loop: ~0.1 s at 20 ms loops
    private static final double MAX_DIP_SEC = 1.0;             // longer than this is not a ball's dip: re-learn
    static final double TWO_SAMPLES_MIN_SEC = 0.045;           // two different low readings this far apart: a ball

    private final double dipTps;
    private final double windowSec;
    private final double minDipSec;
    private final int maxPerPocket;
    private final boolean twoSamples;

    private int balls = 0;
    private int ignored = 0;
    private int thisPocket = 0;
    private boolean wasFeeding = false;
    private double armedUntil = Double.NEGATIVE_INFINITY;
    private double lastTarget = 0;
    private double baseline = Double.NaN;
    private double dipSince = Double.NaN;                      // below the line, not yet recovered, since (NaN = no dip)
    private double runSince = Double.NaN;                      // every reading below the line since
    private double runFirstTps;                                // the first reading of that run
    private boolean runDistinct = false;                       // a later reading of the run differs from it
    private boolean runArmed = false;
    private boolean decided = false;                           // this dip was already counted or ignored

    public BallCounter(double dipTps, double windowSec, double minDipSec, int maxPerPocket) {
        this(dipTps, windowSec, minDipSec, maxPerPocket, false);
    }

    /** @param twoSamples two different low readings TWO_SAMPLES_MIN_SEC apart are a ball too (slow loops) */
    public BallCounter(double dipTps, double windowSec, double minDipSec, int maxPerPocket, boolean twoSamples) {
        this.dipTps = dipTps;
        this.windowSec = windowSec;
        this.minDipSec = minDipSec;
        this.maxPerPocket = maxPerPocket;
        this.twoSamples = twoSamples;
    }

    /** AUTO: one ball per pocket at most (an extra pocket is cheaper than leaving a preload behind). */
    public static BallCounter forAuto() {
        return new BallCounter(BioBuzzConstants.AUTO_SHOT_DIP_TPS, BioBuzzConstants.AUTO_DIP_WINDOW_SEC,
                BioBuzzConstants.AUTO_DIP_MIN_SEC, 1);
    }

    /**
     * TeleOp, Tune: up to 2 per pocket, so a pocket that throws two separate balls can show more than 100 %; two
     * different low readings count (TeleOp's loop is ~55-60 ms while firing).
     */
    public static BallCounter forTuning() {
        return new BallCounter(BioBuzzConstants.AUTO_SHOT_DIP_TPS, BioBuzzConstants.AUTO_DIP_WINDOW_SEC,
                BioBuzzConstants.AUTO_DIP_MIN_SEC, 2, true);
    }

    /**
     * Call once per loop.
     *
     * @param t           seconds, monotonic
     * @param feeding     the windmill is turning a pocket (ShotFeeder FEEDING) after this loop's update
     * @param targetTps   flywheel target (0 = off)
     * @param velocityTps flywheel speed reading
     * @return true when this loop counted a ball
     */
    public boolean update(double t, boolean feeding, double targetTps, double velocityTps) {
        if (targetTps != lastTarget) {                          // off/on or a new speed: re-learn; nothing low now is a ball
            lastTarget = targetTps;
            armedUntil = Double.NEGATIVE_INFINITY;
            baseline = Double.NaN;
            dipSince = Double.NaN;
        }
        if (feeding && !wasFeeding) {
            thisPocket = 0;                                     // a new pocket may count its own ball
        }
        wasFeeding = feeding;
        if (feeding) {
            armedUntil = t + windowSec;
        }
        if (targetTps <= 0) {
            return false;
        }
        if (Double.isNaN(baseline)) {
            baseline = velocityTps;
        }
        double reference = Math.min(targetTps, baseline);
        boolean low = velocityTps < reference - dipTps;
        if (Double.isNaN(dipSince)) {
            if (!low) {
                baseline += BASELINE_ALPHA * (velocityTps - baseline);   // the speed this flywheel really holds
                return false;
            }
            dipSince = t;
            runSince = Double.NaN;
            decided = false;
        } else if (velocityTps > reference - dipTps / 2 || t - dipSince > MAX_DIP_SEC) {
            dipSince = Double.NaN;                              // recovered (or not a ball's dip at all)
            return false;
        }
        if (!low) {
            runSince = Double.NaN;                              // a reading above the line: the 60 ms start over
            return false;
        }
        if (Double.isNaN(runSince)) {
            runSince = t;
            runFirstTps = velocityTps;
            runDistinct = false;
            runArmed = t <= armedUntil;
        } else if (velocityTps != runFirstTps) {
            runDistinct = true;
        }
        boolean longEnough = t - runSince >= minDipSec - 1e-9
                || twoSamples && runDistinct && t - runSince >= TWO_SAMPLES_MIN_SEC - 1e-9;
        if (!decided && longEnough) {
            decided = true;
            if (runArmed && thisPocket < maxPerPocket) {
                thisPocket++;
                balls++;
                return true;
            }
            ignored++;
        }
        return false;
    }

    public int balls() {
        return balls;
    }

    /** Real-looking dips that didn't count (outside a fed pocket's window, or one too many for it). Logged by AUTO. */
    public int ignoredDips() {
        return ignored;
    }

    public void reset() {
        balls = 0;
        ignored = 0;
        thisPocket = 0;
        wasFeeding = false;
        armedUntil = Double.NEGATIVE_INFINITY;
        baseline = Double.NaN;
        dipSince = Double.NaN;
        runSince = Double.NaN;
    }
}

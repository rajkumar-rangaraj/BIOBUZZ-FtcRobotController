package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * Counts balls from flywheel slowdowns (AUTO's "all preloads gone" check, Tune's balls per pocket).
 * <p>
 * A slowdown counts only while the windmill feeds a pocket or within {@code windowSec} after it, and at most
 * {@code maxPerPocket} times per pocket. With the stiff flywheel loop (P 100) a dip of AUTO_SHOT_DIP_TPS is only two
 * encoder steps, so a slowdown with no ball fed (reading noise, spin-up, a speed change) must not count.
 * No FTC SDK dependency: unit-tested.
 */
public class BallCounter {

    private final double dipTps;
    private final double windowSec;
    private final int maxPerPocket;

    private int balls = 0;
    private int ignored = 0;
    private int thisPocket = 0;
    private boolean dipping = false;
    private boolean wasFeeding = false;
    private double armedUntil = Double.NEGATIVE_INFINITY;

    public BallCounter(double dipTps, double windowSec, int maxPerPocket) {
        this.dipTps = dipTps;
        this.windowSec = windowSec;
        this.maxPerPocket = maxPerPocket;
    }

    /** AUTO: one ball per pocket at most (an extra pocket is cheaper than leaving a preload behind). */
    public static BallCounter forAuto() {
        return new BallCounter(BioBuzzConstants.AUTO_SHOT_DIP_TPS, BioBuzzConstants.AUTO_DIP_WINDOW_SEC, 1);
    }

    /** Tune: up to 2 per pocket, so a pocket that throws two balls shows up as more than 100 %. */
    public static BallCounter forTuning() {
        return new BallCounter(BioBuzzConstants.AUTO_SHOT_DIP_TPS, BioBuzzConstants.AUTO_DIP_WINDOW_SEC, 2);
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
        if (feeding && !wasFeeding) {
            thisPocket = 0;                                 // a new pocket may count its own ball
        }
        wasFeeding = feeding;
        if (feeding) {
            armedUntil = t + windowSec;
        }
        if (targetTps <= 0) {
            dipping = false;
            return false;
        }
        if (!dipping && velocityTps < targetTps - dipTps) {
            dipping = true;
            if (t <= armedUntil && thisPocket < maxPerPocket) {
                thisPocket++;
                balls++;
                return true;
            }
            ignored++;
        } else if (dipping && velocityTps > targetTps - dipTps / 2) {
            dipping = false;
        }
        return false;
    }

    public int balls() {
        return balls;
    }

    /** Slowdowns that didn't count (no pocket fed, or already counted for this pocket). Logged by AUTO. */
    public int ignoredDips() {
        return ignored;
    }

    public void reset() {
        balls = 0;
        ignored = 0;
        thisPocket = 0;
        dipping = false;
        wasFeeding = false;
        armedUntil = Double.NEGATIVE_INFINITY;
    }
}

package org.firstinspires.ftc.teamcode.biobuzz.pp;

/**
 * Lets the next volley ball go only when the flywheel has really settled: no more than SETTLE_HIGH over the target
 * (an overshoot while it recovers from the last ball's dip sends the next ball LONG), no more than SETTLE_LOW under,
 * and not rising. After SETTLE_FALLBACK_SEC near the target a bit slower is fine too (tired battery). This is on top
 * of LauncherGate's READY, never instead of it. No FTC SDK dependency: unit-tested (FlywheelSettleTest).
 */
public class FlywheelSettle {

    private final double high, low, maxRise, fallbackLow, fallbackSec;
    private double prevVel = Double.NaN;
    private double nearSince = Double.NaN;

    public FlywheelSettle(double high, double low, double maxRise, double fallbackLow, double fallbackSec) {
        this.high = high;
        this.low = low;
        this.maxRise = maxRise;
        this.fallbackLow = fallbackLow;
        this.fallbackSec = fallbackSec;
    }

    public static FlywheelSettle standard() {
        return new FlywheelSettle(PpConstants.SETTLE_HIGH_TPS, PpConstants.SETTLE_LOW_TPS, PpConstants.SETTLE_MAX_RISE_TPS,
                PpConstants.SETTLE_FALLBACK_LOW_TPS, PpConstants.SETTLE_FALLBACK_SEC);
    }

    /** Call once per loop with a fresh velocity reading. */
    public boolean update(double t, double targetTps, double velocityTps) {
        double rise = Double.isNaN(prevVel) ? 0 : velocityTps - prevVel;
        prevVel = velocityTps;
        double err = velocityTps - targetTps;
        boolean nearBand = err <= high && err >= -fallbackLow;
        if (!nearBand) {
            nearSince = Double.NaN;
        } else if (Double.isNaN(nearSince)) {
            nearSince = t;
        }
        if (rise > maxRise || err > high) {
            return false;
        }
        return err >= -low || (nearBand && t - nearSince >= fallbackSec);
    }

    public void reset() {
        prevVel = Double.NaN;
        nearSince = Double.NaN;
    }
}

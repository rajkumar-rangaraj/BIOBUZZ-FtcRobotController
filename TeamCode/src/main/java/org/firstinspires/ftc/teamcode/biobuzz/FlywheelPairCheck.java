package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * Indexer v2 flywheel: two motors ("launcher" and "launcher2") belted 1:1 to one shaft, each in velocity mode on its
 * own encoder. Both encoders see the same shaft, so they must agree. If one encoder is unplugged (or counts the wrong
 * way) its motor's velocity loop sees "too slow" and drives at full power against the other motor, so the pair must be
 * stopped. A fault is:
 * <ul>
 *   <li>one encoder reads ~0 ({@code deadTps}) while the other is above {@code minTps} (spinning), or</li>
 *   <li>the two readings differ by more than {@code maxDiffTps},</li>
 * </ul>
 * either one lasting {@code faultSec} (the hub reports speed in ~20 t/s steps and the two readings are a few ms apart,
 * so a single odd pair is never a fault). Spin-up is fine: both encoders accelerate together.
 * No FTC SDK dependency: unit-tested on the laptop.
 */
public class FlywheelPairCheck {

    private final double deadTps;
    private final double minTps;
    private final double maxDiffTps;
    private final double faultSec;

    private double badSince = Double.NaN;

    public FlywheelPairCheck(double deadTps, double minTps, double maxDiffTps, double faultSec) {
        this.deadTps = deadTps;
        this.minTps = minTps;
        this.maxDiffTps = maxDiffTps;
        this.faultSec = faultSec;
    }

    public static FlywheelPairCheck standard() {
        return new FlywheelPairCheck(BioBuzzConstants.LAUNCHER_PAIR_DEAD_TPS, BioBuzzConstants.LAUNCHER_PAIR_MIN_TPS,
                BioBuzzConstants.LAUNCHER_PAIR_MAX_DIFF_TPS, BioBuzzConstants.LAUNCHER_PAIR_FAULT_SEC);
    }

    /**
     * Call once per loop while the flywheel is on.
     *
     * @return null while fine, else what is wrong (once it has lasted faultSec)
     */
    public String update(double t, double v1, double v2) {
        String why = problem(v1, v2);
        if (why == null) {
            badSince = Double.NaN;
            return null;
        }
        if (Double.isNaN(badSince)) {
            badSince = t;
        }
        return t - badSince >= faultSec - 1e-9 ? why : null;
    }

    private String problem(double v1, double v2) {
        if (Math.abs(v1) < deadTps && Math.abs(v2) > minTps) {
            return String.format(java.util.Locale.US, "launcher reads ~0 (%.0f t/s) while launcher2 reads %.0f: check"
                    + " the launcher encoder cable", v1, v2);
        }
        if (Math.abs(v2) < deadTps && Math.abs(v1) > minTps) {
            return String.format(java.util.Locale.US, "launcher2 reads ~0 (%.0f t/s) while launcher reads %.0f: check"
                    + " the launcher2 encoder cable", v2, v1);
        }
        if (Math.abs(v1 - v2) > maxDiffTps) {
            boolean opposite = v1 * v2 < 0;
            return String.format(java.util.Locale.US, "launcher %.0f vs launcher2 %.0f t/s%s", v1, v2, opposite
                    ? ": opposite signs - check LAUNCHER2_DIRECTION / motor polarity"
                    : ": encoders disagree - check both encoder cables and the belt");
        }
        return null;
    }

    public void reset() {
        badSince = Double.NaN;
    }
}

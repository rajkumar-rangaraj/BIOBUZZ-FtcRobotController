package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * Mecanum math with no FTC SDK dependency, so it can be unit-tested on a laptop.
 *
 * Wheels must be in the standard "X" layout: seen from ABOVE, the roller lines form an X.
 * forward + = intake side, strafeRight + = robot right, turnCw + = clockwise seen from above.
 */
public final class MecanumMath {
    private MecanumMath() {
    }

    public static final int LF = 0;
    public static final int RF = 1;
    public static final int LB = 2;
    public static final int RB = 3;

    /** Wheel powers {LF, RF, LB, RB}; scaled down together if any would exceed 1, then multiplied by scale. */
    public static double[] wheelPowers(double forward, double strafeRight, double turnCw, double scale) {
        double[] p = {
                forward + strafeRight + turnCw,
                forward - strafeRight - turnCw,
                forward - strafeRight + turnCw,
                forward + strafeRight - turnCw
        };
        double max = 1.0;
        for (double v : p) {
            max = Math.max(max, Math.abs(v));
        }
        for (int i = 0; i < 4; i++) {
            p[i] = p[i] / max * scale;
        }
        return p;
    }

    /** Robot travel implied by the wheel values {LF, RF, LB, RB} (ticks, inches or speeds). */
    public static double forward(double[] w) {
        return (w[LF] + w[RF] + w[LB] + w[RB]) / 4.0;
    }

    public static double strafeRight(double[] w) {
        return (w[LF] - w[RF] - w[LB] + w[RB]) / 4.0;
    }

    public static double turnCw(double[] w) {
        return (w[LF] - w[RF] + w[LB] - w[RB]) / 4.0;
    }

    /** Wraps any angle to (-180, 180]. */
    public static double normalizeDegrees(double deg) {
        double a = deg % 360.0;
        if (a <= -180) {
            a += 360;
        } else if (a > 180) {
            a -= 360;
        }
        return a;
    }
}

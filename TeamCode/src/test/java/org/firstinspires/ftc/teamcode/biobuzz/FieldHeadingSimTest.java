package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.Random;

/**
 * FieldHeading against a simulated robot: random driving (<= 300 deg/s, <= 2500 deg/s^2), TeleOp loop times as in our
 * diag logs (20-80 ms, rarely up to ~0.2 s), an IMU whose yaw flips at +-90 (as on our robot) or doesn't, and one
 * sensor fault per run. The heading must stay within a few degrees of the truth outside the fault window (fault +
 * settling), and at the end.
 */
public class FieldHeadingSimTest {

    enum Fault {
        NONE, PIN_DROPOUT, PIN_RESTART, PIN_GARBAGE, PIN_BURST, IMU_GARBAGE, IMU_BURST, IMU_RESTART, IMU_FROZEN,
        IMU_ONLY, IMU_ONLY_GARBAGE
    }

    private static final double RUN_SEC = 25;
    private static final double SETTLE_SEC = 0.6;

    /** @return {worst error outside the fault window, worst inside, error at the end} */
    static double[] run(long seed, Fault fault, boolean flips, boolean longStalls, StringBuilder trace) {
        Random r = new Random(seed);
        FieldHeading h = FieldHeading.standard(0);
        double truth = r.nextDouble() * 360 - 180, truth0 = truth;   // the drivers' "away" = the start heading
        double imuZero = truth + r.nextDouble() * 60 - 30;           // the IMU zeroed a little apart
        double rate = 0, cmd = 0, cmdUntil = 0.5;                    // still at START
        int bad = fault == Fault.PIN_BURST || fault == Fault.IMU_BURST ? 2 + r.nextInt(3) : 1;
        double tf = 4 + r.nextDouble() * 12;
        double tfEnd = tf + SETTLE_SEC + (bad - 1) * 0.08;
        if (fault == Fault.PIN_DROPOUT) {
            tfEnd += 0.2 + r.nextDouble() * 1.8;
        }
        double truthAtFault = Double.NaN, frozen = Double.NaN;
        double outside = 0, inside = 0, err = 0, t = 0;
        while (t < RUN_SEC) {
            double dt = 0.02 + r.nextDouble() * 0.06;
            if (r.nextDouble() < 0.001) {
                dt = 0.1 + r.nextDouble() * 0.12;
            } else if (longStalls && r.nextDouble() < 0.005) {
                dt = 0.3 + r.nextDouble() * 0.2;
            }
            for (double s = 0; s < dt - 1e-9; s += 0.01) {           // the robot moves (10 ms steps)
                double step = Math.min(0.01, dt - s);
                if (t + s >= cmdUntil) {
                    cmd = r.nextDouble() < 0.3 ? 0 : r.nextDouble() * 600 - 300;
                    cmdUntil = t + s + 0.3 + r.nextDouble() * 1.2;
                }
                rate += Math.max(-2500 * step, Math.min(2500 * step, cmd - rate));
                truth += rate * step;
            }
            t += dt;
            truth = MecanumMath.normalizeDegrees(truth);
            double pin = MecanumMath.normalizeDegrees(truth - truth0 + r.nextGaussian() * 0.05);   // zeroed at INIT
            double imu = MecanumMath.normalizeDegrees(truth - imuZero + r.nextGaussian() * 0.2);
            boolean inFault = t >= tf;
            if (inFault && Double.isNaN(truthAtFault)) {
                truthAtFault = truth;
            }
            switch (fault) {
                case PIN_DROPOUT:
                    if (inFault && t < tfEnd - SETTLE_SEC) {
                        pin = Double.NaN;
                    }
                    break;
                case PIN_RESTART:
                    if (inFault) {
                        pin = MecanumMath.normalizeDegrees(truth - truthAtFault + r.nextGaussian() * 0.05);
                    }
                    break;
                case PIN_GARBAGE:
                case PIN_BURST:
                    if (inFault && bad-- > 0) {
                        pin = r.nextDouble() * 360 - 180;
                    }
                    break;
                case IMU_GARBAGE:
                case IMU_BURST:
                case IMU_ONLY_GARBAGE:
                    if (inFault && bad-- > 0) {
                        imu = r.nextDouble() * 360 - 180;
                    }
                    break;
                case IMU_RESTART:
                    if (inFault) {
                        imu = MecanumMath.normalizeDegrees(truth - truthAtFault + r.nextGaussian() * 0.2);
                    }
                    break;
                default:
                    break;
            }
            if (fault == Fault.IMU_ONLY || fault == Fault.IMU_ONLY_GARBAGE) {
                pin = Double.NaN;
            }
            if (flips && Math.abs(imu) > 90) {
                imu = MecanumMath.normalizeDegrees(imu + 180);
            }
            if (fault == Fault.IMU_FROZEN && inFault) {
                if (Double.isNaN(frozen)) {
                    frozen = imu;
                }
                imu = frozen;
            }
            double out = h.update(t, pin, imu);
            if (trace != null) {
                trace.append(String.format("t=%6.3f dt=%3.0f truth=%7.1f pin=%7.1f imu=%7.1f out=%7.1f %s%n", t, dt * 1000,
                        MecanumMath.normalizeDegrees(truth - truth0), pin, imu, out, h.event() == null ? "" : h.event()));
            }
            if (Double.isNaN(out)) {
                continue;
            }
            err = Math.abs(MecanumMath.normalizeDegrees(out - (truth - truth0)));
            if (t >= tf && t < tfEnd) {
                inside = Math.max(inside, err);
            } else {
                outside = Math.max(outside, err);
            }
        }
        return new double[] {outside, inside, err};
    }

    private static String check(boolean longStalls, Fault... faults) {
        StringBuilder failures = new StringBuilder();        for (Fault fault : faults) {
            double worstOut = 0, worstIn = 0, worstEnd = 0;
            for (long seed = 1; seed <= 200; seed++) {
                for (boolean flips : new boolean[] {false, true}) {
                    double[] e = run(seed * 7919 + fault.ordinal() * 104729L + (longStalls ? 1 : 0), fault, flips,
                            longStalls, null);
                    worstOut = Math.max(worstOut, e[0]);
                    worstIn = Math.max(worstIn, e[1]);
                    worstEnd = Math.max(worstEnd, e[2]);
                }
            }
            // a Pinpoint restart small enough to pass as a turn stays (documented); alone, the IMU waits a loop or two
            // a bad reading that is a possible move can win for a loop: brief only. A Pinpoint restart small enough to
            // pass as a turn stays (documented); the IMU alone can keep a little of a bad reading that looked possible
            double after = fault == Fault.PIN_RESTART ? 16 : fault == Fault.IMU_ONLY_GARBAGE ? 25 : 12;
            double inLimit = fault == Fault.IMU_ONLY_GARBAGE ? 90 : fault == Fault.PIN_GARBAGE   // alone: held 0.3 s
                    || fault == Fault.PIN_BURST || fault == Fault.IMU_GARBAGE || fault == Fault.IMU_BURST ? 35 : 16;
            String line = String.format("%-16s outside %5.1f  inside %5.1f  end %5.1f deg%s%n", fault, worstOut, worstIn,
                    worstEnd, longStalls ? " (with 0.3-0.5 s stalls)" : "");
            System.out.print(line);
            if (worstOut > after || (!longStalls && worstIn > inLimit) || worstEnd > after) {   // stalls: nothing left
                failures.append(line);
            }
        }
        return failures.toString();
    }

    @Test
    public void theHeadingFollowsTheRobotThroughSensorFaults() {
        String failures = check(false, Fault.values());
        assertTrue(failures, failures.isEmpty());
    }

    @Test
    public void withBothSensorsLongLoopStallsDoNotLeaveAnError() {
        // 0.3-0.5 s stalls never happened in our logs (max 0.21 s). One sensor alone can't tell a long-stall turn from
        // a restart (a documented limit): faults that leave one sensor for a while are not run here.
        String failures = check(true, Fault.NONE, Fault.IMU_GARBAGE, Fault.IMU_BURST);
        assertTrue(failures, failures.isEmpty());
    }

    /**
     * A short burst of bad Pinpoint readings (1-5 loops: random, or a ramp away from the truth), the robot still or
     * turning steadily, the IMU flipping or not and, while still, repeating its reading exactly never, half the time or
     * always (ours repeats ~15 % at rest). Review 10 Nov (Opus 5.5): a few such bursts in 10,000 left a lasting error
     * of up to 180 deg.
     *
     * @return the worst error from 1 s after the burst on
     */
    static double burstRun(long seed, StringBuilder trace) {
        Random r = new Random(seed);
        FieldHeading h = FieldHeading.standard(0);
        double truth = r.nextDouble() * 360 - 180, truth0 = truth;
        double imuZero = truth + r.nextDouble() * 60 - 30;
        boolean flips = r.nextBoolean();
        double rate = r.nextBoolean() ? 0 : r.nextDouble() * 600 - 300;
        double repeat = rate != 0 ? 0 : r.nextInt(3) * 0.5;
        boolean steady = r.nextBoolean();                              // 23 ms loops, else 20-80 ms
        int k = 1 + r.nextInt(5);
        boolean ramp = r.nextBoolean();
        double rampStep = (3 + r.nextDouble() * 12) * (r.nextBoolean() ? 1 : -1);
        double tf = 1 + r.nextDouble(), t = 0, imuRaw = Double.NaN, worst = 0;
        int bad = 0;
        while (t < tf + 2.5) {
            double dt = steady ? 0.023 : 0.02 + r.nextDouble() * 0.06;
            t += dt;
            truth += Math.signum(rate) * Math.min(Math.abs(rate), Math.max(0, t - 0.3) * 2500) * dt;   // still at START
            double pin = MecanumMath.normalizeDegrees(truth - truth0 + r.nextGaussian() * 0.05);
            if (Double.isNaN(imuRaw) || r.nextDouble() >= repeat) {
                imuRaw = MecanumMath.normalizeDegrees(truth - imuZero + r.nextGaussian() * (rate == 0 ? 0.02 : 0.2));
            }
            double imu = flips && Math.abs(imuRaw) > 90 ? MecanumMath.normalizeDegrees(imuRaw + 180) : imuRaw;
            if (t >= tf && bad < k) {
                bad++;
                pin = ramp ? MecanumMath.normalizeDegrees(pin + rampStep * bad) : r.nextDouble() * 360 - 180;
            }
            double out = h.update(t, pin, imu);
            double err = Math.abs(MecanumMath.normalizeDegrees(out - (truth - truth0)));
            if (trace != null) {
                trace.append(String.format("t=%6.3f truth=%7.1f pin=%7.1f imu=%7.1f out=%7.1f err=%5.1f %s%n", t,
                        MecanumMath.normalizeDegrees(truth - truth0), pin, imu, out, err,
                        h.event() == null ? "" : h.event()));
            }
            if (t >= tf + 1) {
                worst = Math.max(worst, err);
            }
        }
        return worst;
    }

    @Test
    public void shortBadPinpointBurstsLeaveNoLastingError() {
        int runs = 20000, off = 0;
        double worst = 0;
        long worstSeed = 0;
        for (long seed = 1; seed <= runs; seed++) {
            double e = burstRun(seed, null);
            if (e > 2) {
                off++;
            }
            if (e > worst) {
                worst = e;
                worstSeed = seed;
            }
        }
        String line = String.format("PIN bursts: %d of %d runs off by more than 2 deg 1 s later, worst %.1f deg (seed %d)%n",
                off, runs, worst, worstSeed);
        System.out.print(line);
        assertTrue(line, off == 0);
    }
}

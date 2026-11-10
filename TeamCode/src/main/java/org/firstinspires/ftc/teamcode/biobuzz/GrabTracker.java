package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * The moment the flywheel takes a ball, for the diagnostics log (TeleOp). BallCounter decides THAT a ball went (AUTO uses
 * it too); this measures HOW, from the flywheel speed and, when TeleOp reads it, the launcher current:
 * <ul>
 *   <li>the grab: the first reading of the dip (to within one loop; the gap to the reading before is the uncertainty);</li>
 *   <li>the real speed just before the grab (median of the readings in PRE_SEC before it; the last of them when the
 *       flywheel was still climbing back from the ball before), vs the target;</li>
 *   <li>the dip: lowest speed, depth, time below the line, and the share of the flywheel's energy the ball took,
 *       1 - (min / pre)^2: a weak grab (the windmill pushed the ball in badly) takes less;</li>
 *   <li>the recovery: from the grab until the speed is back within readyTol of the target;</li>
 *   <li>the windmill: ticks left to its pocket boundary at the grab (negative = the ball came after the pocket ended);</li>
 *   <li>the launcher current just before and at its peak in the dip;</li>
 *   <li>drops: a second fall inside the same dip (after a partial recovery) = two balls close together; the ball counter
 *       sees one long dip then.</li>
 * </ul>
 * A record closes when the speed is back in the ready band, after MAX_SEC, when the target changes, or when the next
 * ball comes first (then it is marked overlapped: two balls close together). No FTC SDK dependency: unit-tested.
 */
public class GrabTracker {

    static final double PRE_SEC = 0.20;                 // "just before the grab": readings in this window
    static final double LOOK_BACK_SEC = 0.8;             // how far back the grab is searched for
    static final double MAX_SEC = 1.5;                   // a dip longer than this closes without a recovery
    private static final int BUFFER = 64;

    /** One grab. Fields filled while the dip lasts; the record is complete when poll() returns it. */
    public static class Grab {
        public final int ball;
        public final double tStart;
        public final double startGapSec;
        public final double preTps;
        public final double targetTps;
        public final double ticksToBoundary;
        public final boolean feedingAtStart;
        public final double sincePocketEndSec;
        public final double baseAmps;
        public double minTps;
        public double tMin;
        public double belowSec = 0;
        public double recoverySec = Double.NaN;
        public double peakAmps = Double.NaN;
        /** Separate drops in this dip: 2 or more = balls close together (one long dip for the ball counter). */
        public int drops = 1;
        public boolean overlapped = false;
        public boolean targetChanged = false;
        double dropMin, riseMax, riseMaxT, lastPeakT = Double.NaN;

        Grab(int ball, double tStart, double startGapSec, double preTps, double targetTps, double ticksToBoundary,
             boolean feedingAtStart, double sincePocketEndSec, double baseAmps, double startVel) {
            this.ball = ball;
            this.tStart = tStart;
            this.startGapSec = startGapSec;
            this.preTps = preTps;
            this.targetTps = targetTps;
            this.ticksToBoundary = ticksToBoundary;
            this.feedingAtStart = feedingAtStart;
            this.sincePocketEndSec = sincePocketEndSec;
            this.baseAmps = baseAmps;
            this.minTps = startVel;
            this.tMin = tStart;
            this.dropMin = startVel;
            this.riseMax = startVel;
            this.riseMaxT = tStart;
        }

        /** One more reading of this dip: lowest speed, and a new drop after a partial recovery (another ball). */
        void track(double t, double vel, double dipTps) {
            if (vel < minTps) {
                minTps = vel;
                tMin = t;
            }
            if (riseMax - dropMin > dipTps && vel < riseMax - dipTps) {
                drops++;                                 // it had come back up, and now falls again
                lastPeakT = riseMaxT;                    // the next ball's record starts from this peak
                dropMin = vel;
                riseMax = vel;
                riseMaxT = t;
            } else if (vel < dropMin) {
                dropMin = vel;
                riseMax = vel;
                riseMaxT = t;
            } else if (vel > riseMax) {
                riseMax = vel;
                riseMaxT = t;
            }
        }

        public double depthTps() {
            return preTps - minTps;
        }

        /** Share of the flywheel's kinetic energy the ball took (the motor adds some during the dip: a lower bound). */
        public double energyFraction() {
            return preTps > 0 ? 1 - (minTps / preTps) * (minTps / preTps) : Double.NaN;
        }

        /** Real speed just before the grab minus the target. */
        public double launchErrorTps() {
            return preTps - targetTps;
        }
    }

    private static class Sample {
        final double t, vel, target, ticks, boundary, amps;
        final boolean feeding;

        Sample(double t, double vel, double target, double ticks, double boundary, boolean feeding, double amps) {
            this.t = t;
            this.vel = vel;
            this.target = target;
            this.ticks = ticks;
            this.boundary = boundary;
            this.feeding = feeding;
            this.amps = amps;
        }
    }

    private final double dipTps;
    private final double readyTol;
    private final ArrayDeque<Sample> buf = new ArrayDeque<>();
    private final List<Grab> closed = new ArrayList<>();
    private Grab open = null;
    private double lastFeedingT = Double.NaN;
    private double lastT = Double.NaN;

    /**
     * @param dipTps   a reading this far below the speed before it is part of a dip (BallCounter's line)
     * @param readyTol the dip is over when the speed is back within this of the target (the ready band)
     */
    public GrabTracker(double dipTps, double readyTol) {
        this.dipTps = dipTps;
        this.readyTol = readyTol;
    }

    public static GrabTracker standard() {
        return new GrabTracker(BioBuzzConstants.AUTO_SHOT_DIP_TPS, BioBuzzConstants.LAUNCHER_READY_TOLERANCE);
    }

    /**
     * Once per loop, before ballCounted().
     *
     * @param target   flywheel target (0 = off)
     * @param ticks    windmill encoder
     * @param boundary the windmill's pocket boundary (ShotFeeder.targetTicks())
     * @param feeding  a pocket is turning
     * @param amps     launcher current, NaN when not read this loop
     */
    public void sample(double t, double vel, double target, double ticks, double boundary, boolean feeding, double amps) {
        double dt = Double.isNaN(lastT) ? 0 : t - lastT;
        lastT = t;
        buf.addLast(new Sample(t, vel, target, ticks, boundary, feeding, amps));
        while (buf.size() > BUFFER || (!buf.isEmpty() && t - buf.peekFirst().t > 2 * LOOK_BACK_SEC)) {
            buf.removeFirst();
        }
        if (feeding) {
            lastFeedingT = t;
        }
        if (open == null) {
            return;
        }
        Grab g = open;
        g.track(t, vel, dipTps);
        if (vel < g.preTps - dipTps) {
            g.belowSec += dt;
        }
        if (!Double.isNaN(amps) && !(amps <= g.peakAmps)) {
            g.peakAmps = amps;
        }
        if (target != g.targetTps) {
            g.targetChanged = true;
            close();
        } else if (t > g.tMin && vel >= target - readyTol) {
            g.recoverySec = t - g.tStart;
            close();
        } else if (t - g.tStart > MAX_SEC) {
            close();
        }
    }

    /** BallCounter counted a ball in this loop (after sample()): opens its record. */
    public void ballCounted(double t, int ball) {
        double after = Double.NEGATIVE_INFINITY;         // the new dip is searched for from here on (inclusive)
        if (open != null) {
            open.overlapped = true;                      // the next ball came before this dip was over
            after = Double.isNaN(open.lastPeakT) ? open.tMin : open.lastPeakT;   // from the peak before its drop
            close();
        }
        Sample[] s = buf.toArray(new Sample[0]);
        int n = s.length;
        if (n == 0) {
            return;
        }
        double hi = Double.NEGATIVE_INFINITY;            // the speed the dip fell from
        for (Sample x : s) {
            if (x.t >= after && t - x.t <= LOOK_BACK_SEC) {
                hi = Math.max(hi, x.vel);
            }
        }
        int start = n - 1;                               // the first reading of the run of low readings ending now
        while (start > 0 && s[start - 1].t >= after && t - s[start - 1].t <= LOOK_BACK_SEC
                && s[start - 1].vel < hi - dipTps) {
            start--;
        }
        Sample first = s[start];
        List<Double> pre = new ArrayList<>();
        List<Double> preAmps = new ArrayList<>();
        for (int i = 0; i < start; i++) {
            if (first.t - s[i].t <= PRE_SEC && s[i].t >= after) {
                pre.add(s[i].vel);
                if (!Double.isNaN(s[i].amps)) {
                    preAmps.add(s[i].amps);
                }
            }
        }
        // speed just before the grab: the median of the window (encoder steps of ~20 t/s), or the last reading when
        // the flywheel was still climbing back from the ball before (the median would be too low then)
        double preTps;
        if (pre.isEmpty()) {
            preTps = start > 0 ? s[start - 1].vel : hi;
        } else {
            double lo = Double.POSITIVE_INFINITY, top = Double.NEGATIVE_INFINITY;
            for (double v : pre) {
                lo = Math.min(lo, v);
                top = Math.max(top, v);
            }
            preTps = top - lo > dipTps ? pre.get(pre.size() - 1) : median(pre);
        }
        double gap = start > 0 ? first.t - s[start - 1].t : Double.NaN;
        double sincePocketEnd = first.feeding || Double.isNaN(lastFeedingT) ? Double.NaN : first.t - lastFeedingBefore(s, start);
        Grab g = new Grab(ball, first.t, gap, preTps, first.target, first.boundary - first.ticks, first.feeding,
                sincePocketEnd, preAmps.isEmpty() ? Double.NaN : median(preAmps), first.vel);
        for (int i = start; i < n; i++) {                // the dip so far
            g.track(s[i].t, s[i].vel, dipTps);
            if (i > start && s[i].vel < preTps - dipTps) {
                g.belowSec += s[i].t - s[i - 1].t;
            }
            if (!Double.isNaN(s[i].amps) && !(s[i].amps <= g.peakAmps)) {
                g.peakAmps = s[i].amps;
            }
        }
        open = g;
    }

    /** A finished record (each one once), or null. */
    public Grab poll() {
        return closed.isEmpty() ? null : closed.remove(0);
    }

    /** A dip is being measured (TeleOp reads the launcher current while this or a pocket is on). */
    public boolean active() {
        return open != null;
    }

    private void close() {
        closed.add(open);
        open = null;
    }

    private static double lastFeedingBefore(Sample[] s, int idx) {
        for (int i = idx; i >= 0; i--) {
            if (s[i].feeding) {
                return s[i].t;
            }
        }
        return Double.NaN;
    }

    private static double median(List<Double> v) {
        double[] a = new double[v.size()];
        for (int i = 0; i < a.length; i++) {
            a[i] = v.get(i);
        }
        Arrays.sort(a);
        return a.length % 2 == 1 ? a[a.length / 2] : (a[a.length / 2 - 1] + a[a.length / 2]) / 2;
    }
}

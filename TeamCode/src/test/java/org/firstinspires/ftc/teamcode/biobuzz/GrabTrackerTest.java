package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class GrabTrackerTest {
    private static final double DT = 0.05;

    /** Flywheel at 1450, balls take 330 t/s at the given times, recovery 800 t/s per s; windmill boundary at 96. */
    private static class Run {
        final GrabTracker g = new GrabTracker(35, 30);
        final BallCounter counter = new BallCounter(35, 0.6, 0.06, 2);
        double t = 0, vel = 1450, target = 1450, ticks = 0;
        double[] ballsAt = {};
        boolean[] done = new boolean[8];
        int balls = 0;
        boolean feeding = false;
        double amps = 2.0;

        void step() {
            for (int i = 0; i < ballsAt.length; i++) {
                if (!done[i] && t >= ballsAt[i]) {
                    done[i] = true;
                    vel -= 330;
                    amps = 9.0;
                }
            }
            g.sample(t, vel, target, ticks, 96, feeding, amps);
            if (counter.update(t, feeding, target, vel)) {
                g.ballCounted(t, ++balls);
            }
            vel = Math.min(target, vel + 800 * DT);
            amps = Math.max(2.0, amps - 2.0);
            t += DT;
        }

        void until(double end) {
            while (t < end) {
                step();
            }
        }
    }

    @Test
    public void aCleanGrabIsMeasured() {
        Run r = new Run();
        r.ballsAt = new double[]{1.0};
        r.feeding = true;
        r.ticks = 70;                                                  // 26 ticks before the pocket boundary
        r.until(3);
        GrabTracker.Grab g = r.g.poll();
        assertNotNull(g);
        assertNull("one ball, one record", r.g.poll());
        assertEquals(1, g.ball);
        assertEquals("the grab is the first low reading", 1.0, g.tStart, 1e-9);
        assertEquals(1450, g.preTps, 1e-9);
        assertEquals(1120, g.minTps, 1e-9);
        assertEquals(330, g.depthTps(), 1e-9);
        assertEquals(1 - (1120.0 / 1450) * (1120.0 / 1450), g.energyFraction(), 1e-9);
        assertEquals(0, g.launchErrorTps(), 1e-9);
        assertEquals("back within 30 of the target after (330 - 30) / 800 s, to the next reading", 0.40, g.recoverySec, DT + 1e-9);
        assertEquals(26, g.ticksToBoundary, 1e-9);
        assertTrue(g.feedingAtStart);
        assertEquals(2.0, g.baseAmps, 1e-9);
        assertEquals(9.0, g.peakAmps, 1e-9);
        assertFalse(g.overlapped);
        assertFalse(r.g.active());
    }

    @Test
    public void twoBallsInOneDipShowAsTwoDrops() {
        Run r = new Run();
        r.ballsAt = new double[]{1.0, 1.3};                            // the second comes before the first recovered
        r.feeding = true;
        r.until(4);
        assertEquals("the ball counter sees one long dip", 1, r.balls);
        GrabTracker.Grab a = r.g.poll();
        assertNotNull(a);
        assertNull(r.g.poll());
        assertEquals("two drops: two balls close together", 2, a.drops);
        assertEquals(1030, a.minTps, 1e-9);
        assertEquals(1.3, a.tMin, 1e-9);
        assertEquals("back in the band (1420) at 1.8 s", 0.80, a.recoverySec, DT + 1e-9);
    }

    @Test
    public void twoSeparateBallsGiveTwoRecords() {
        Run r = new Run();
        r.ballsAt = new double[]{1.0, 1.7};
        r.feeding = true;
        r.until(4);
        GrabTracker.Grab a = r.g.poll(), b = r.g.poll();
        assertNotNull(a);
        assertNotNull(b);
        assertEquals(1, a.drops);
        assertEquals(1, b.drops);
        assertFalse(a.overlapped);
        assertEquals(1.7, b.tStart, 1e-9);
        assertEquals("the flywheel was back for the second ball", 1450, b.preTps, 1e-9);
    }

    @Test
    public void aBallCountedBeforeTheLastDipEndedMarksItOverlapped() {
        GrabTracker g = new GrabTracker(35, 30);
        double t = 0;
        for (; t < 0.5 - 1e-9; t += DT) {
            g.sample(t, 1450, 1450, 0, 96, true, Double.NaN);
        }
        g.sample(0.50, 1100, 1450, 0, 96, true, Double.NaN);
        g.sample(0.55, 1150, 1450, 0, 96, true, Double.NaN);
        g.ballCounted(0.55, 1);
        g.sample(0.60, 1200, 1450, 0, 96, true, Double.NaN);
        g.sample(0.65, 1250, 1450, 0, 96, true, Double.NaN);
        g.sample(0.70, 950, 1450, 0, 96, true, Double.NaN);
        g.ballCounted(0.70, 2);
        GrabTracker.Grab first = g.poll();
        assertNotNull(first);
        assertTrue(first.overlapped);
        assertEquals(1, first.ball);
        assertTrue(g.active());
        for (t = 0.75; t < 2; t += DT) {
            g.sample(t, Math.min(1450, 950 + 800 * (t - 0.70)), 1450, 0, 96, true, Double.NaN);
        }
        GrabTracker.Grab second = g.poll();
        assertNotNull(second);
        assertEquals(2, second.ball);
        assertEquals(0.70, second.tStart, 1e-9);
        assertEquals("the speed before the second ball: what it had climbed back to", 1250, second.preTps, 1e-9);
    }

    @Test
    public void aDipThatNeverRecoversClosesAfterMaxSec() {
        Run r = new Run();
        r.ballsAt = new double[]{1.0};
        r.feeding = true;                                              // the ball counter only counts around a pocket
        r.until(1.2);
        r.target = 1450;
        r.vel = 1200;                                                  // stuck low: weak battery
        GrabTracker.Grab g = null;
        while (r.t < 4 && (g = r.g.poll()) == null) {
            r.g.sample(r.t, 1200, 1450, 0, 96, false, Double.NaN);
            r.t += DT;
        }
        assertNotNull(g);
        assertTrue(Double.isNaN(g.recoverySec));
        assertTrue(r.t - g.tStart > GrabTracker.MAX_SEC);
    }

    @Test
    public void aTargetChangeClosesTheRecord() {
        Run r = new Run();
        r.ballsAt = new double[]{1.0};
        r.feeding = true;
        r.until(1.15);
        assertTrue(r.g.active());
        r.target = 1570;                                               // the driver picked FAR mid-dip
        r.step();
        GrabTracker.Grab g = r.g.poll();
        assertNotNull(g);
        assertTrue(g.targetChanged);
        assertTrue(Double.isNaN(g.recoverySec));
    }

    @Test
    public void aBallAfterThePocketEndedShowsHowLate() {
        Run r = new Run();
        r.feeding = true;
        r.ticks = 90;
        r.until(0.5);
        r.feeding = false;                                             // the pocket ended at ~0.45 s
        r.ticks = 97;
        r.ballsAt = new double[]{0.8};
        r.until(2.5);
        GrabTracker.Grab g = r.g.poll();
        assertNotNull(g);
        assertFalse(g.feedingAtStart);
        assertEquals("the ball came ~0.35 s after the last feeding reading", 0.35, g.sincePocketEndSec, DT + 1e-9);
        assertEquals("past the boundary", -1, g.ticksToBoundary, 1e-9);
    }
}

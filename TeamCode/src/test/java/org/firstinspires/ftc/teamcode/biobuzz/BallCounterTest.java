package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;

import java.util.function.DoubleUnaryOperator;

import org.junit.Test;

/** Ball counting from flywheel dips: only balls the windmill actually fed, never reading noise. */
public class BallCounterTest {

    private static final double DT = 0.02;                       // 20 ms loops

    private static BallCounter auto() {
        return new BallCounter(35, 0.6, 0.06, 1);
    }

    /** Steps the counter from t0 to t1 (exclusive); returns the balls counted in that stretch. */
    private static int run(BallCounter c, double t0, double t1, boolean feeding, double target, DoubleUnaryOperator vel) {
        int counted = 0;
        for (double t = t0; t < t1 - 1e-9; t += DT) {
            if (c.update(t, feeding, target, vel.applyAsDouble(t))) {
                counted++;
            }
        }
        return counted;
    }

    /** A real ball: the flywheel drops by depth for ~100 ms, then recovers within ~0.15 s. */
    private static DoubleUnaryOperator ballDip(double start, double target, double depth) {
        return t -> {
            double s = t - start;
            if (s < 0) {
                return target;
            }
            if (s < 0.1) {
                return target - depth;
            }
            return target - depth * Math.max(0, 1 - (s - 0.1) / 0.15);
        };
    }

    @Test
    public void aBallAfterAPocketCountsOnce() {
        BallCounter c = auto();
        assertEquals(0, run(c, 0.0, 0.3, false, 1240, t -> 1240));      // idle: learns the speed
        assertEquals(0, run(c, 0.3, 0.46, true, 1240, t -> 1240));      // pocket turning
        assertEquals(1, run(c, 0.46, 1.0, false, 1240, ballDip(0.5, 1240, 60)));
        assertEquals(1, c.balls());
    }

    @Test
    public void noiseInsideAnArmedWindowDoesNotCount() {
        BallCounter c = auto();                                         // GPT review: 1200/1240 readings at FAR
        run(c, 0.0, 0.3, false, 1240, t -> 1240);
        run(c, 0.3, 0.46, true, 1240, t -> 1240);                       // an empty pocket
        assertEquals(0, run(c, 0.46, 1.1, false, 1240,
                t -> Math.round(t / DT) % 2 == 0 ? 1200 : 1240));       // every other reading low
        run(c, 1.1, 1.26, true, 1240, t -> 1240);                       // another empty pocket
        assertEquals(0, run(c, 1.26, 1.9, false, 1240,
                t -> Math.round(t / DT) % 4 < 2 ? 1200 : 1240));        // one hub sample read twice
        assertEquals(0, c.balls());
    }

    @Test
    public void aFlywheelThatSettlesLowIsNotDipping() {
        BallCounter c = auto();                                         // mean ~1213 at target 1240 (F a bit low)
        DoubleUnaryOperator low = t -> Math.round(t / DT) % 3 == 0 ? 1200 : 1220;
        run(c, 0.0, 0.4, false, 1240, low);
        run(c, 0.4, 0.56, true, 1240, low);
        assertEquals(0, run(c, 0.56, 1.2, false, 1240, low));
        run(c, 1.2, 1.36, true, 1240, low);
        assertEquals(1, run(c, 1.36, 1.9, false, 1240, ballDip(1.4, 1213, 60)));   // a real ball still counts
    }

    @Test
    public void spinUpAndSpeedChangesAreNotBalls() {
        BallCounter c = auto();
        assertEquals(0, run(c, 0.0, 1.0, false, 1150, t -> Math.min(1150, 1400 * t)));   // spin-up from rest
        run(c, 1.0, 1.16, true, 1150, t -> 1150);                       // a pocket: armed until ~1.75
        assertEquals(0, run(c, 1.16, 1.6, false, 1240, t -> Math.min(1240, 1150 + 300 * (t - 1.16))));  // NEAR -> FAR
        assertEquals(0, c.balls());
    }

    @Test
    public void launcherOffAndOnInsideTheWindowCountsNothingMore() {
        BallCounter c = new BallCounter(35, 0.6, 0.06, 2);              // GPT review: Tune OFF/ON during recovery
        run(c, 0.0, 0.3, false, 1150, t -> 1150);
        run(c, 0.3, 0.46, true, 1150, t -> 1150);
        assertEquals(1, run(c, 0.46, 0.8, false, 1150, ballDip(0.5, 1150, 60)));
        run(c, 0.8, 0.9, false, 0, t -> 900);                          // OFF: coasting down
        assertEquals(0, run(c, 0.9, 1.4, false, 1150, t -> Math.min(1150, 900 + 600 * (t - 0.9))));   // ON again
        assertEquals(1, c.balls());
    }

    @Test
    public void onlyOneBallPerPocketInAuto() {
        BallCounter c = auto();
        run(c, 0.0, 0.3, false, 1150, t -> 1150);
        run(c, 0.3, 0.46, true, 1150, t -> 1150);
        assertEquals(1, run(c, 0.46, 0.76, false, 1150, ballDip(0.48, 1150, 60)));
        assertEquals(0, run(c, 0.76, 1.0, false, 1150, ballDip(0.78, 1150, 60)));   // second dip, same pocket
        run(c, 1.0, 1.16, true, 1150, t -> 1150);                       // next pocket
        assertEquals(1, run(c, 1.16, 1.6, false, 1150, ballDip(1.2, 1150, 60)));
        assertEquals(2, c.balls());
        assertEquals(1, c.ignoredDips());
    }

    @Test
    public void lateOrShortDipsAreNotBalls() {
        BallCounter c = auto();
        run(c, 0.0, 0.3, false, 1150, t -> 1150);
        run(c, 0.3, 0.46, true, 1150, t -> 1150);                       // armed until ~1.04
        assertEquals(0, run(c, 0.46, 0.7, false, 1150, t -> t >= 0.5 && t < 0.54 ? 1100 : 1150));  // 40 ms: noise
        assertEquals(0, run(c, 0.7, 1.6, false, 1150, ballDip(1.1, 1150, 60)));      // starts after the window
        assertEquals(0, c.balls());
        assertEquals(1, c.ignoredDips());
    }

    @Test
    public void tuningCountsTwoSeparateBallsFromOnePocket() {
        BallCounter c = new BallCounter(35, 0.6, 0.06, 2);
        run(c, 0.0, 0.3, false, 1150, t -> 1150);
        run(c, 0.3, 0.46, true, 1150, t -> 1150);
        assertEquals(1, run(c, 0.46, 0.76, false, 1150, ballDip(0.48, 1150, 60)));
        assertEquals(1, run(c, 0.76, 1.0, false, 1150, ballDip(0.78, 1150, 60)));
        assertEquals(2, c.balls());
        c.reset();
        assertEquals(0, c.balls());
    }

    /** Readings dt apart from t0; returns the balls counted. */
    private static int feed(BallCounter c, double t0, double dt, boolean feeding, double target, double... vels) {
        int counted = 0;
        double t = t0;
        for (double v : vels) {
            if (c.update(t, feeding, target, v)) {
                counted++;
            }
            t += dt;
        }
        return counted;
    }

    private static void learnAndFeedAPocket(BallCounter c, double dt) {
        feed(c, 0.0, dt, false, 1450, 1460, 1440, 1460, 1460, 1440, 1460);
        feed(c, 0.33, dt, true, 1450, 1460, 1460);
    }

    @Test
    public void slowLoopsCountABallSeenInTwoDifferentLowReadings() {
        // 10 Nov TeleOp logs: 55-60 ms loops while firing; a ball's dip (1460, 1180, 1260, 1440) gave two low readings
        // ~55 ms apart, under the 60 ms rule: 6 of 34 shots weren't counted and their LB + D-pad labels were lost
        BallCounter teleOp = new BallCounter(35, 0.6, 0.06, 2, true);
        BallCounter auto = auto();
        learnAndFeedAPocket(teleOp, 0.055);
        learnAndFeedAPocket(auto, 0.055);
        assertEquals(1, feed(teleOp, 0.44, 0.055, false, 1450, 1460, 1180, 1260, 1440, 1520, 1480));
        assertEquals("AUTO keeps its rule", 0, feed(auto, 0.44, 0.055, false, 1450, 1460, 1180, 1260, 1440, 1520, 1480));
    }

    @Test
    public void oneHubSampleReadTwiceIsStillNotABall() {
        BallCounter c = new BallCounter(35, 0.6, 0.06, 2, true);
        learnAndFeedAPocket(c, 0.05);
        assertEquals(0, feed(c, 0.44, 0.05, false, 1450, 1460, 1180, 1180, 1460, 1460));    // the same value twice
        BallCounter fast = new BallCounter(35, 0.6, 0.06, 2, true);
        learnAndFeedAPocket(fast, 0.02);
        assertEquals(0, feed(fast, 0.44, 0.02, false, 1450, 1460, 1400, 1390, 1460, 1460));   // 20 ms apart: noise
    }

    @Test
    public void noiseInsideAnArmedWindowDoesNotCountWithTwoSamplesEither() {
        BallCounter c = new BallCounter(35, 0.6, 0.06, 2, true);
        run(c, 0.0, 0.3, false, 1240, t -> 1240);
        run(c, 0.3, 0.46, true, 1240, t -> 1240);
        assertEquals(0, run(c, 0.46, 1.1, false, 1240,
                t -> Math.round(t / DT) % 2 == 0 ? 1200 : 1240));       // every other reading low
        run(c, 1.1, 1.26, true, 1240, t -> 1240);
        assertEquals(0, run(c, 1.26, 1.9, false, 1240,
                t -> Math.round(t / DT) % 4 < 2 ? 1200 : 1240));        // one hub sample read twice
        assertEquals(0, c.balls());
    }
}

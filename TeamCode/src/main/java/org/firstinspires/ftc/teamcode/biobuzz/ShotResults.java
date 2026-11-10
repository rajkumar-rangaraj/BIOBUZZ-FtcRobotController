package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.ArrayDeque;

/**
 * Shot results for the diagnostics log (TeleOp). Every ball the flywheel launched (seen as a speed dip, BallCounter)
 * is kept for windowSec. A driver or scout says what it did with LB + D-pad: IN, SHORT (too slow), LONG (too fast) or
 * SIDE (left / right / bounced out: aim, not speed). A result goes to the OLDEST ball of the window that has none
 * yet, so a volley is labelled in the order the balls flew; balls older than the window stay unlabelled.
 * tools/shot_results.py turns the labelled balls into speed-table corrections. No FTC SDK dependency: unit-tested.
 */
public class ShotResults {

    public enum Result { IN, SHORT, LONG, SIDE }

    /** One launched ball, with what the speed table and the driver's trim asked for when it left. */
    public static class Ball {
        public final int number;
        public final double t;
        /** Camera floor distance to the CELL when it left (NaN = no CELL in view). */
        public final double floorIn;
        public final double targetTps;
        public final double trimTps;
        /** The AUTO table's speed at floorIn (NaN without a distance). */
        public final double tableTps;
        private Result result;

        Ball(int number, double t, double floorIn, double targetTps, double trimTps, double tableTps) {
            this.number = number;
            this.t = t;
            this.floorIn = floorIn;
            this.targetTps = targetTps;
            this.trimTps = trimTps;
            this.tableTps = tableTps;
        }

        public Result result() {
            return result;
        }
    }

    private final double windowSec;
    private final ArrayDeque<Ball> recent = new ArrayDeque<>();
    private final int[] counts = new int[Result.values().length];
    private int balls = 0;
    private int labelled = 0;

    public ShotResults(double windowSec) {
        this.windowSec = windowSec;
    }

    public Ball launched(double t, double floorIn, double targetTps, double trimTps, double tableTps) {
        Ball b = new Ball(++balls, t, floorIn, targetTps, trimTps, tableTps);
        recent.addLast(b);
        forgetOld(t);
        return b;
    }

    /** Gives this result to the oldest ball of the window without one; null when there is none. */
    public Ball label(double t, Result r) {
        forgetOld(t);
        for (Ball b : recent) {
            if (b.result == null) {
                b.result = r;
                labelled++;
                counts[r.ordinal()]++;
                return b;
            }
        }
        return null;
    }

    /** Balls of the window still waiting for a result. */
    public int waiting(double t) {
        forgetOld(t);
        int n = 0;
        for (Ball b : recent) {
            if (b.result == null) {
                n++;
            }
        }
        return n;
    }

    public int balls() {
        return balls;
    }

    public int labelled() {
        return labelled;
    }

    public int count(Result r) {
        return counts[r.ordinal()];
    }

    private void forgetOld(double t) {
        while (!recent.isEmpty() && t - recent.peekFirst().t > windowSec) {
            recent.removeFirst();
        }
    }
}

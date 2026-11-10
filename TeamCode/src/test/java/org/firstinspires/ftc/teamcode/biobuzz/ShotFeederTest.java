package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** ShotFeeder against a simulated windmill and flywheel. */
public class ShotFeederTest {
    private static final double DT = 0.01;
    private static final double POCKET = 96.0;

    /** Windmill: ticks/s at full power; flywheel: drops on each ball, recovers at a fixed rate. */
    private static class Sim {
        double t = 0, ticks = 0, vel = 1250, target = 1250;
        double ticksPerSecAtFull = 1500;
        boolean jammed = false;
        double jamAt = Double.NaN;                                   // forward motion blocked beyond this many ticks
        double stickyFrom = Double.NaN, stickyTo = Double.NaN;       // a tight spot: below 0.3 power the windmill stops
        int ballsOut = 0;
        double lastBallBoundary = 0;
        final ShotFeeder f = new ShotFeeder(POCKET, 0.6, 0.35, 0.35, 1.0, 0.25, 3);
        final LauncherGate gate = LauncherGate.standard();         // the robot's real fire rule

        Sim() {
            gate.update(-1.0, target, vel);                          // flywheel already settled at speed
        }

        boolean ready() {
            return gate.update(t, target, vel);
        }

        void step(boolean wantFire) {
            double p = f.update(t, (int) Math.round(ticks), ready(), wantFire);
            boolean sticky = ticks >= stickyFrom && ticks < stickyTo && p > 0 && p < 0.3;
            if (!(jammed && p > 0) && !sticky) {
                ticks += p * ticksPerSecAtFull * DT;
                if (p > 0 && ticks > jamAt) {
                    ticks = jamAt;
                }
            }
            if (ticks - lastBallBoundary >= POCKET) {               // a ball leaves at each pocket boundary
                lastBallBoundary += POCKET;
                ballsOut++;
                vel -= 300;                                          // shot dip
            }
            vel = Math.min(target, vel + 800 * DT);                  // recovery ~0.4 s for a 300 dip
            t += DT;
        }
    }

    @Test
    public void slowLoopDoesNotOvershootThePocket() {
        // Robot log 2026-10-30: ~1500 ticks/s at 0.6 power and 13-54 ms loops gave 1.06-1.76 pockets per shot.
        ShotFeeder f = new ShotFeeder(POCKET, 0.6, 0.35, 0.35, 1.0, 0.25, 3, POCKET, 0.15);
        double t = 0, ticks = 0, vel = 0;                       // vel = windmill ticks/s (motor lag + coast)
        java.util.Random rnd = new java.util.Random(7);
        int lastShots = 0;
        double lastStop = 0, worst = 0;
        for (int i = 0; i < 4000 && f.shots() < 12; i++) {
            double loop = 0.013 + rnd.nextDouble() * 0.041;       // 13..54 ms, as logged
            double p = f.update(t, (int) Math.round(ticks), true, true);
            for (double s = 0; s < loop; s += 0.001) {            // 1 ms physics inside one loop
                vel += (p * 2500 - vel) * (0.001 / 0.010);        // 0.6 power -> 1500 ticks/s; 10 ms (log: coast 2-10 ticks)
                ticks += vel * 0.001;
                t += 0.001;
            }
            if (f.shots() > lastShots) {
                lastShots = f.shots();
                for (int k = 0; k < 200; k++) {                    // coast to rest (power is 0 now)
                    vel += (0 - vel) * (0.001 / 0.010);
                    ticks += vel * 0.001;
                }
                double travel = ticks - lastStop;
                worst = Math.max(worst, Math.abs(ticks - lastShots * POCKET));
                lastStop = ticks;
                assertTrue("shot " + lastShots + " travel " + travel, travel > 0.8 * POCKET && travel < 1.2 * POCKET);
            }
        }
        assertTrue("12 shots, got " + f.shots(), f.shots() >= 12);
        assertTrue("error after 12 shots stays under 30 ticks (never adds up), was " + worst, worst < 30);
    }

    @Test
    public void firesOneBallPerPocketAndWaitsForTheFlywheel() {
        Sim s = new Sim();
        double minSpeedAtShot = 1e9;
        int lastBalls = 0;
        for (int i = 0; i < 400; i++) {                              // hold FIRE for 3 s, then let the last pocket finish
            double velBefore = s.vel;
            s.step(i < 300);
            if (s.ballsOut > lastBalls) {
                minSpeedAtShot = Math.min(minSpeedAtShot, velBefore);
                lastBalls = s.ballsOut;
            }
        }
        assertEquals(ShotFeeder.State.IDLE, s.f.state());
        assertTrue("several shots in 3 s, got " + s.f.shots(), s.f.shots() >= 4);
        assertEquals("one ball per counted shot", s.f.shots(), s.ballsOut);
        assertTrue("every ball left inside the ready band, worst " + minSpeedAtShot,
                minSpeedAtShot >= s.target - BioBuzzConstants.LAUNCHER_READY_TOLERANCE);
    }

    @Test
    public void releasingFireFinishesThePocketThenStops() {
        Sim s = new Sim();
        for (int i = 0; i < 3; i++) {
            s.step(true);                                            // starts a pocket
        }
        for (int i = 0; i < 200; i++) {
            s.step(false);
        }
        assertEquals(1, s.f.shots());
        assertEquals(ShotFeeder.State.IDLE, s.f.state());
        double overshoot = s.ticks - POCKET;                          // stops within one loop's movement of the pocket
        assertTrue("stopped at " + s.ticks, overshoot >= 0 && overshoot <= 0.6 * 1500 * DT + 1e-9);
    }

    @Test
    public void waitsWhileTheFlywheelIsSlow() {
        Sim s = new Sim();
        s.vel = 500;                                                  // still spinning up
        for (int i = 0; i < 20; i++) {
            s.step(true);
        }
        assertEquals(0, s.f.shots());
        assertEquals(0.0, s.ticks, 1e-9);
    }

    @Test
    public void jamBacksOffRetriesThenGivesUpUntilReleased() {
        Sim s = new Sim();
        s.jammed = true;
        String lastEvent = null;
        for (int i = 0; i < 600; i++) {
            s.step(true);
            if (s.f.event() != null) {
                lastEvent = s.f.event();
            }
        }
        assertEquals(ShotFeeder.State.JAMMED, s.f.state());
        assertEquals(3, s.f.jams());
        assertNotNull(lastEvent);
        assertTrue(lastEvent.startsWith("JAMMED"));
        assertTrue("backed off", s.ticks < 0);
        s.step(false);                                                // release FIRE: reset
        assertEquals(ShotFeeder.State.IDLE, s.f.state());
        assertEquals(0, s.f.jams());
    }

    @Test
    public void jamBacksOffHalfAPocketAtMostThenRetriesTheSamePocket() {
        Sim s = new Sim();
        s.jamAt = 50;                                                 // a ball wedges the windmill 50 ticks into the pocket
        double lowest = 1e9;
        while (s.f.jams() == 0 && s.t < 3) {
            s.step(true);
        }
        s.jamAt = Double.NaN;                                         // the back-off frees it
        while (s.f.shots() == 0 && s.t < 6) {
            s.step(true);
            lowest = Math.min(lowest, s.ticks);
        }
        assertEquals(1, s.f.jams());
        assertTrue("backed off at most a quarter pocket (+ one step), lowest " + lowest,
                lowest >= 50 - ShotFeeder.UNJAM_MAX_POCKETS * POCKET - 0.35 * 1500 * DT - 1e-9);
        assertEquals("the retry goes to the same pocket boundary", POCKET, s.f.targetTicks(), 1e-9);
        assertTrue("and reaches it, at " + s.ticks, s.ticks >= POCKET - 0.5);
    }

    @Test
    public void longDriverUnjamDoesNotCatchUpTheMissedPockets() {
        // Robot logs 31 Oct - 1 Nov: the target kept counting on while the windmill went backwards, so the next pocket
        // was a 2-7 pocket push. Now it goes to the first boundary ahead.
        Sim s = new Sim();
        while (s.f.shots() < 2 && s.t < 5) {
            s.step(true);
        }
        for (int i = 0; i < 100; i++) {
            s.step(false);                                            // FIRE released: the pocket finishes, then idle
        }
        assertEquals(ShotFeeder.State.IDLE, s.f.state());
        s.f.reset();                                                  // driver holds B: the windmill turns back 3.5 pockets
        s.ticks -= 3.5 * POCKET;
        double start = s.ticks;
        while (s.f.shots() < 3 && s.t < 10) {
            s.step(true);
        }
        double push = s.ticks - start;
        assertTrue("push " + push + " ticks", push > 0 && push <= POCKET + 0.6 * 1500 * DT);
        assertEquals("it stops on a pocket boundary of the same grid", 0.0, Math.IEEEremainder(s.f.targetTicks(), POCKET), 1e-9);
    }

    @Test
    public void stuckWindmillGetsMorePowerInsteadOfJamming() {
        Sim s = new Sim();
        s.stickyFrom = 60;                                            // the ramp is below 0.3 power here
        s.stickyTo = 85;
        boolean sawStuck = false;
        while (s.f.shots() == 0 && s.t < 3) {
            s.step(true);
            sawStuck |= s.f.event() != null && s.f.event().startsWith("STUCK");
        }
        assertTrue(sawStuck);
        assertEquals(0, s.f.jams());
        assertEquals(1, s.f.shots());
        assertTrue("pocket finished in " + s.t + " s", s.t < 1.0);
    }

    /**
     * Closer to the robot (logs 31 Oct - 1 Nov): 96.125-tick pockets, 60-76 ms loops while feeding, the windmill motor
     * reaches 2200 x power - 30 ticks/s with a 15 ms lag and brakes in ~12 ms at zero power. Flywheel always ready.
     */
    private static class RealSim {
        static final double P = BioBuzzConstants.INDEXER_TICKS_PER_POCKET;
        final ShotFeeder f = new ShotFeeder(P, 0.40, 0.35, 0.35, 1.0, 0.25, 3, P, 0.15);
        final java.util.Random rnd = new java.util.Random(11);
        double t = 0, ticks = 0, w = 0;
        double jamAt = Double.NaN;                                    // forward motion stops here
        boolean reverseBlocked = false;
        double stickyFrom = Double.NaN, stickyTo = Double.NaN, stickyPower = 0.25;
        boolean driverUnjam = false;                                  // TeleOp: B held = feeder.reset() + reverse 0.35
        double lowest = 1e9;

        void loop(boolean fire) {
            double p;
            if (driverUnjam) {
                f.reset();
                p = -0.35;
            } else {
                p = f.update(t, (int) Math.round(ticks), true, fire);
            }
            double dt = 0.060 + rnd.nextDouble() * 0.016;
            for (int ms = 0; ms < Math.round(dt * 1000); ms++) {
                double wss = p == 0 ? 0 : Math.signum(p) * Math.max(0, 2200 * Math.abs(p) - 30);
                boolean sticky = ticks >= stickyFrom && ticks < stickyTo && p > 0 && p < stickyPower;
                w += ((sticky ? 0 : wss) - w) * 0.001 / (p == 0 ? 0.012 : 0.015);
                if (sticky && w > 0) {
                    w = 0;
                }
                double next = ticks + w * 0.001;
                if (w > 0 && next > jamAt) {
                    next = jamAt;
                    w = 0;
                }
                if (w < 0 && reverseBlocked) {
                    next = ticks;
                    w = 0;
                }
                ticks = next;
                lowest = Math.min(lowest, ticks);
            }
            t += dt;
        }

        void settle() {
            for (int i = 0; i < 5; i++) {
                loop(false);
            }
        }
    }

    @Test
    public void realisticNormalShotsStopNearEachBoundary() {
        RealSim s = new RealSim();
        while (s.f.shots() < 8 && s.t < 10) {
            s.loop(true);
        }
        s.settle();
        double err = s.ticks - 8 * RealSim.P;
        assertTrue("8 pockets end " + err + " ticks from the 8th boundary", err > -2 && err < 0.35 * RealSim.P);
        assertEquals(0, s.f.jams());
    }

    @Test
    public void realisticStuckNearTheEndBreaksFreeWithoutFlyingPast() {
        RealSim s = new RealSim();
        s.stickyFrom = RealSim.P - 45;                                // logs: stalls with 26-48 ticks left at 0.15-0.2 power
        s.stickyTo = RealSim.P - 20;
        String stuck = null;
        while (s.f.shots() == 0 && s.t < 3) {
            s.loop(true);
            if (s.f.event() != null && s.f.event().startsWith("STUCK")) {
                stuck = s.f.event();
            }
        }
        s.settle();
        assertNotNull(stuck);
        assertEquals(0, s.f.jams());
        assertEquals(1, s.f.shots());
        double over = s.ticks - RealSim.P;
        assertTrue("stopped " + over + " ticks past the boundary", over > -2 && over < 0.35 * RealSim.P);
    }

    @Test
    public void realisticJamAtThePocketStartRetriesTheSamePocket() {
        RealSim s = new RealSim();
        s.jamAt = 5;                                                  // wedged right at the start of the first pocket
        while (s.f.jams() == 0 && s.t < 3) {
            s.loop(true);
        }
        s.jamAt = Double.NaN;                                         // the back-off frees it
        while (s.f.shots() == 0 && s.t < 6) {
            s.loop(true);
        }
        s.settle();
        assertTrue("back-off " + (5 - s.lowest) + " ticks", 5 - s.lowest < 0.65 * RealSim.P);
        assertEquals("no extra pocket counted for the way back", 1, s.f.shots());
        assertEquals(RealSim.P, s.f.targetTicks(), 1e-9);
        assertTrue("at " + s.ticks, s.ticks > RealSim.P - 2 && s.ticks < 1.35 * RealSim.P);
    }

    @Test
    public void realisticBlockedBackOffRetriesTheSameBoundaryEvenWhenClose() {
        RealSim s = new RealSim();
        s.jamAt = 80;                                                 // stuck 16 ticks before the boundary ...
        s.reverseBlocked = true;                                      // ... and the back-off can't move it either
        while (s.f.jams() == 0 && s.t < 3) {
            s.loop(true);
        }
        while (s.f.state() == ShotFeeder.State.UNJAM) {
            s.loop(true);
        }
        s.jamAt = Double.NaN;
        s.reverseBlocked = false;
        while (s.f.shots() == 0 && s.t < 6) {
            s.loop(true);
        }
        s.settle();
        assertEquals("not a pocket further (that could feed the next ball too)", RealSim.P, s.f.targetTicks(), 1e-9);
        assertTrue("at " + s.ticks, s.ticks > RealSim.P - 2 && s.ticks < 1.35 * RealSim.P);
    }

    @Test
    public void realisticStuckRightAtTheBoundaryIsAJamThatRetriesTheSameBoundary() {
        RealSim s = new RealSim();
        s.jamAt = RealSim.P - 6;                                      // held 6 ticks short: its ball may still be there
        boolean sawStuck = false;
        while (s.f.jams() == 0 && s.t < 3) {
            s.loop(true);
            sawStuck |= s.f.event() != null && s.f.event().startsWith("STUCK");
        }
        assertTrue("more power was tried first", sawStuck);
        assertEquals("not counted as a shot", 0, s.f.shots());
        s.jamAt = Double.NaN;                                         // the back-off frees it
        while (s.f.shots() < 2 && s.t < 8) {                         // keep firing: the retry, then the next pocket
            s.loop(true);
        }
        s.settle();
        assertEquals(1, s.f.jams());
        assertEquals(2, s.f.shots());
        assertEquals("the retry finished pocket 1, then pocket 2 - none skipped", 2 * RealSim.P, s.f.targetTicks(), 1e-9);
    }

    @Test
    public void realisticShortIdleDriverUnjamGoesBackToTheNextUnfiredBoundary() {
        RealSim s = new RealSim();
        while (s.f.shots() < 2 && s.t < 5) {
            s.loop(true);
        }
        s.settle();
        s.f.reset();                                                  // driver unjam: the windmill (and its balls) 0.8 pocket back
        s.ticks -= 0.8 * RealSim.P;
        double start = s.ticks;
        while (s.f.shots() < 3 && s.t < 8) {
            s.loop(true);
        }
        s.settle();
        assertEquals("the next unfired pocket, no empty pushes on the way", 3 * RealSim.P, s.f.targetTicks(), 1e-9);
        assertTrue("one push of " + (s.ticks - start) / RealSim.P + " pockets",
                s.ticks - start > 1.6 * RealSim.P && s.ticks - start < 2.2 * RealSim.P);
    }

    @Test
    public void realisticLongIdleDriverUnjamStepsForwardOnePocketPerPush() {
        RealSim s = new RealSim();
        while (s.f.shots() < 2 && s.t < 5) {
            s.loop(true);
        }
        s.settle();
        s.f.reset();                                                  // 1.5 pockets back: a pocket may have passed the intake
        s.ticks -= 1.5 * RealSim.P;
        double start = s.ticks;
        while (s.f.shots() < 3 && s.t < 8) {
            s.loop(true);
        }
        s.settle();
        assertEquals("the first boundary ahead", RealSim.P, s.f.targetTicks(), 1e-9);
        assertTrue("a push of " + (s.ticks - start) / RealSim.P + " pockets", s.ticks - start < 0.9 * RealSim.P);
    }

    @Test
    public void realisticGivingUpThenFiringAgainRetriesTheSameBoundary() {
        RealSim s = new RealSim();
        s.jamAt = 30;
        while (s.f.state() != ShotFeeder.State.JAMMED && s.t < 8) {
            s.loop(true);
        }
        assertEquals(ShotFeeder.State.JAMMED, s.f.state());
        assertEquals(RealSim.P, s.f.targetTicks(), 1e-9);
        s.loop(false);                                                // FIRE released: reset
        assertEquals(ShotFeeder.State.IDLE, s.f.state());
        s.jamAt = Double.NaN;                                         // the driver cleared it
        while (s.f.shots() == 0 && s.t < 12) {
            s.loop(true);
        }
        s.settle();
        assertEquals(1, s.f.shots());
        assertEquals(RealSim.P, s.f.targetTicks(), 1e-9);
        assertTrue("at " + s.ticks, s.ticks > RealSim.P - 2 && s.ticks < 1.35 * RealSim.P);
    }

    @Test
    public void realisticDriverUnjamNearTheEndFinishesTheSamePocket() {
        RealSim s = new RealSim();
        while (!(s.f.feeding() && s.f.targetTicks() - s.ticks > 0 && s.f.targetTicks() - s.ticks < 30) && s.t < 5) {
            s.loop(true);
        }
        assertTrue("caught a pocket near its end", s.f.feeding());
        double boundary = s.f.targetTicks();
        int shotsBefore = s.f.shots();
        s.driverUnjam = true;                                         // B pressed just before the boundary
        s.loop(false);
        s.loop(false);
        s.driverUnjam = false;
        while (s.f.shots() == shotsBefore && s.t < 8) {
            s.loop(true);
        }
        s.settle();
        assertEquals(shotsBefore + 1, s.f.shots());
        assertEquals("the same boundary, not one further", boundary, s.f.targetTicks(), 1e-9);
        assertTrue("at " + s.ticks, s.ticks > boundary - 2 && s.ticks < boundary + 0.35 * RealSim.P);
    }

    /**
     * Windmill and flywheel as in Sim, but only some pockets hold a ball, and a ball can reach the flywheel some time after
     * its pocket boundary (logs: most dip the flywheel while the pocket turns, a few up to 0.3 s after it stopped).
     * at[k]: how far through pocket k its ball reaches the flywheel (1 = at the boundary; 7 Nov logs: 44 % of balls dip
     * the flywheel during their pocket, at about 2/3 of it), then late[k] seconds more.
     */
    private static class PocketSim {
        static final double PAUSE = 0.25, EMPTY_WAIT = 0.10;
        double t = 0, ticks = 0, vel = 1250, target = 1250;
        boolean aimOk = true;
        final boolean[] ball;
        final double[] late;
        final double[] at;
        final ShotFeeder f;
        final LauncherGate gate = LauncherGate.standard();
        final java.util.List<Double> ballTimes = new java.util.ArrayList<>();
        final java.util.List<Double> speedAtBall = new java.util.ArrayList<>();
        final java.util.List<String> events = new java.util.ArrayList<>();
        final java.util.List<Double> due = new java.util.ArrayList<>();
        final java.util.List<Double> pushStarts = new java.util.ArrayList<>();
        final java.util.List<Double> pocketEnds = new java.util.ArrayList<>();
        int nextBall = 0;

        PocketSim(boolean skip, boolean[] ball, double[] late) {
            this(skip, ball, late, null);
        }

        PocketSim(boolean skip, boolean[] ball, double[] late, double[] at) {
            this(new ShotFeeder(POCKET, 0.6, 0.35, PAUSE, 1.0, 0.25, 3).skipEmptyPockets(skip, EMPTY_WAIT), ball, late, at);
        }

        PocketSim(ShotFeeder f, boolean[] ball, double[] late) {
            this(f, ball, late, null);
        }

        PocketSim(ShotFeeder f, boolean[] ball, double[] late, double[] at) {
            this.ball = ball;
            this.late = late;
            this.at = at != null ? at : filled(ball.length, 1.0);
            this.f = f;
            gate.update(-1.0, target, vel);
        }

        void step(boolean fire) {
            boolean ready = gate.update(t, target, vel);
            boolean wasFeeding = f.feeding();
            int shotsBefore = f.shots();
            double p = f.update(t, (int) Math.round(ticks), ready, aimOk, fire);
            if (f.event() != null) {
                events.add(String.format("%.2f %s", t, f.event()));
            }
            if (f.shots() > shotsBefore) {
                pocketEnds.add(t);
            }
            if (!wasFeeding && f.feeding()) {
                pushStarts.add(t);
            }
            ticks += Math.max(0, p) * 1500 * DT;
            while (nextBall < ball.length && ticks >= (nextBall + at[nextBall]) * POCKET) {
                if (ball[nextBall]) {
                    due.add(t + late[nextBall]);
                }
                nextBall++;
            }
            for (java.util.Iterator<Double> it = due.iterator(); it.hasNext(); ) {
                if (t >= it.next()) {
                    it.remove();
                    ballTimes.add(t);
                    speedAtBall.add(vel);
                    vel -= 300;                                       // one wheel: 300-350 t/s per ball (7 Nov logs)
                }
            }
            vel = Math.min(target, vel + 800 * DT);                  // recovers ~0.4 s after a ball
            t += DT;
        }

        double runUntilBalls(int n, double maxT) {
            while (ballTimes.size() < n && t < maxT) {
                step(true);
            }
            return t;
        }

        boolean sawEvent(String start) {
            for (String e : events) {
                if (e.substring(e.indexOf(' ') + 1).startsWith(start)) {
                    return true;
                }
            }
            return false;
        }
    }

    private static boolean[] pockets(String s) {                     // "B.B." = ball, empty, ball, empty
        boolean[] b = new boolean[s.length()];
        for (int i = 0; i < b.length; i++) {
            b[i] = s.charAt(i) == 'B';
        }
        return b;
    }

    @Test
    public void emptyPocketsGoWithoutThePause() {
        boolean[] b = pockets("B.B..B");
        double[] late = new double[b.length];
        PocketSim plain = new PocketSim(false, b, late);
        PocketSim skip = new PocketSim(true, b, late);
        double tPlain = plain.runUntilBalls(3, 10);
        double tSkip = skip.runUntilBalls(3, 10);
        assertEquals(3, skip.ballTimes.size());
        assertEquals("3 empty pockets before the last ball: no pause after each", 3, skip.f.noPausePockets());
        assertEquals(0, plain.f.noPausePockets());
        assertTrue("3 balls: " + tSkip + " s with the skip vs " + tPlain + " s", tSkip < tPlain - 3 * 0.12);
        for (double v : skip.speedAtBall) {
            assertTrue("every ball goes into a flywheel at speed, was " + v,
                    v >= skip.target - BioBuzzConstants.LAUNCHER_READY_TOLERANCE);
        }
    }

    @Test
    public void aBallInEveryPocketKeepsTheFullPause() {
        boolean[] b = pockets("BBBB");
        PocketSim s = new PocketSim(true, b, new double[b.length]);
        s.runUntilBalls(4, 10);
        assertEquals(4, s.ballTimes.size());
        assertEquals("every pocket dipped the flywheel: never skipped", 0, s.f.noPausePockets());
        for (int i = 1; i < s.ballTimes.size(); i++) {
            assertTrue("ball " + i + " at " + s.speedAtBall.get(i), s.speedAtBall.get(i) >= s.target - 30);
        }
    }

    @Test
    public void aLateBallStopsTheNextPocketUntilTheFlywheelIsBack() {
        boolean[] b = pockets("BB");
        double[] late = {0.20, 0.0};                                // ball 1 reaches the flywheel 0.2 s after its pocket stops
        PocketSim s = new PocketSim(true, b, late);
        s.runUntilBalls(2, 10);
        assertEquals(2, s.ballTimes.size());
        assertTrue("pocket 2 started early (pocket 1 looked empty): " + s.events, s.sawEvent("NO PAUSE"));
        assertTrue("then held when ball 1 dipped the flywheel: " + s.events, s.sawEvent("HOLD"));
        assertTrue("ball 2 went into a flywheel at speed, was " + s.speedAtBall.get(1),
                s.speedAtBall.get(1) >= s.target - BioBuzzConstants.LAUNCHER_READY_TOLERANCE);
        assertTrue("and not before the pause after ball 1",
                s.ballTimes.get(1) - s.ballTimes.get(0) >= PocketSim.PAUSE - 1e-9);
        assertEquals(0, s.f.jams());
    }

    @Test
    public void aShorterPlainPauseWouldLetTheNextBallIntoALateDip() {
        // why the skip waits for the flywheel instead of just using a short pause: a 0.1 s pause, same late ball
        boolean[] b = pockets("BB");
        double[] late = {0.20, 0.0};
        PocketSim s = new PocketSim(new ShotFeeder(POCKET, 0.6, 0.35, 0.10, 1.0, 0.25, 3), b, late);
        s.runUntilBalls(2, 10);
        assertEquals(2, s.ballTimes.size());
        assertTrue("a plain 0.1 s pause lets ball 2 into the dip: " + s.speedAtBall.get(1),
                s.speedAtBall.get(1) < s.target - BioBuzzConstants.LAUNCHER_READY_TOLERANCE);
    }

    @Test
    public void anAimHoldIsNotTakenForABall() {
        boolean[] b = pockets("..B");
        PocketSim s = new PocketSim(true, b, new double[b.length]);
        while (s.f.shots() < 1 && s.t < 5) {
            s.step(true);
        }
        double done = s.t;                                           // pocket 1 (empty) just finished
        s.aimOk = false;                                             // the aim gate holds the next pocket briefly
        while (s.t < done + 0.08) {
            s.step(true);
        }
        assertEquals("nothing starts while the aim gate holds", 1, s.f.shots());
        assertEquals(ShotFeeder.State.WAIT_READY, s.f.state());
        s.aimOk = true;
        while (!s.f.feeding() && s.t < done + 1) {
            s.step(true);
        }
        double wait = s.t - done;
        assertTrue("the next pocket starts after the empty wait, not the full pause: " + wait,
                wait >= PocketSim.EMPTY_WAIT - 1e-9 && wait < PocketSim.PAUSE);
        assertTrue(s.events.toString(), s.sawEvent("NO PAUSE"));
    }

    @Test
    public void aLongHoldIsNotAJam() {
        boolean[] b = pockets("BB");
        double[] late = {0.20, 0.0};
        PocketSim s = new PocketSim(true, b, late);
        while (!s.f.holding() && s.t < 5) {
            s.step(true);
        }
        assertTrue("held", s.f.holding());
        s.vel = s.target - 1200;                                      // a slow flywheel: about 1.5 s to recover
        s.runUntilBalls(2, 10);
        assertEquals(2, s.ballTimes.size());
        assertEquals("waiting in a hold never counts towards the 1 s jam timeout", 0, s.f.jams());
        assertTrue(s.speedAtBall.get(1) >= s.target - BioBuzzConstants.LAUNCHER_READY_TOLERANCE);
    }

    @Test
    public void aBallThatDipsMidPocketIsHeldAndItsPauseIsNotWaitedTwice() {
        boolean[] b = pockets(".B..");
        double[] at = {1, 0.6, 1, 1};                                // pocket 2's ball dips the flywheel 60 % into it
        PocketSim s = new PocketSim(true, b, new double[b.length], at);
        while (s.f.shots() < 3 && s.t < 5) {
            s.step(true);
        }
        assertTrue("pocket 2 started without the pause and its own ball held it: " + s.events, s.sawEvent("HOLD"));
        double end2 = s.pocketEnds.get(1);
        double start3 = s.pushStarts.get(2);
        assertTrue("the pause after the ball ran during the hold, pocket 3 goes as soon as pocket 2 ends: "
                + (start3 - end2) + " s, " + s.events, start3 - end2 <= DT + 1e-9);
        assertTrue("and still not before the pause after the ball", start3 - s.ballTimes.get(0) >= PocketSim.PAUSE);
        assertTrue(s.events.toString(), s.sawEvent("NO PAUSE (pocket 2's pause ran during its hold)"));
    }

    @Test
    public void aLateBallFromAHeldPocketStopsThePocketAfterIt() {
        // ball 1 is late: pocket 2 starts early and is held by it. Ball 2 is late too, so pocket 3 (started at once: the
        // pause ran during the hold) must be held by ball 2 before ball 3 goes
        boolean[] b = pockets("BBB");
        double[] late = {0.20, 0.12, 0.0};
        PocketSim s = new PocketSim(true, b, late);
        s.runUntilBalls(3, 10);
        assertEquals(3, s.ballTimes.size());
        for (int i = 0; i < 3; i++) {
            assertTrue("ball " + (i + 1) + " went in at " + s.speedAtBall.get(i) + ": " + s.events,
                    s.speedAtBall.get(i) >= s.target - BioBuzzConstants.LAUNCHER_READY_TOLERANCE);
        }
        assertEquals(0, s.f.jams());
    }

    @Test
    public void aHeldPocketTurnsAgainInTheLoopTheHoldEnds() {
        // a loop with the windmill stopped after the hold would count towards the 1 s jam timeout (review, 9 Nov)
        boolean[] b = pockets("BB");
        double[] late = {0.20, 0.0};
        PocketSim s = new PocketSim(true, b, late);
        while (!s.f.holding() && s.t < 5) {
            s.step(true);
        }
        assertTrue("held", s.f.holding());
        while (s.f.holding() && s.t < 5) {
            double before = s.ticks;
            s.step(true);
            if (!s.f.holding()) {
                assertTrue("the windmill turns in the loop the hold ends", s.ticks > before);
            }
        }
        assertFalse(s.f.holding());
    }

    @Test
    public void releasingFireDuringAHoldStopsThereAndTheNextFireFinishesThePocket() {
        for (boolean preSpin : new boolean[]{true, false}) {
            boolean[] b = pockets("BB");
            double[] late = {0.20, 0.0};
            PocketSim s = new PocketSim(true, b, late);
            while (!s.f.holding() && s.t < 5) {
                s.step(true);
            }
            assertTrue("held", s.f.holding());
            double boundary = s.f.targetTicks();
            double ticks = s.ticks;
            int shots = s.f.shots();
            if (!preSpin) {
                s.target = 0;                                        // FIRE let go: TeleOp turns the flywheel off
            }
            double released = s.t;
            while (s.t < released + 1.0) {
                s.step(false);
            }
            assertEquals("pre-spin " + preSpin + ": stopped where it was held", ticks, s.ticks, 1e-9);
            assertEquals(ShotFeeder.State.IDLE, s.f.state());
            assertFalse(s.f.holding());
            assertEquals("ball 2 stays in its pocket", 1, s.ballTimes.size());
            assertEquals(shots, s.f.shots());
            assertTrue(s.events.toString(), s.sawEvent("RELEASED"));
            s.target = 1250;
            while (s.f.shots() == shots && s.t < released + 5) {
                s.step(true);
            }
            assertEquals("pre-spin " + preSpin + ": the next FIRE finishes the held pocket, not one further",
                    boundary, s.f.targetTicks(), 1e-9);
            s.runUntilBalls(2, released + 6);
            assertEquals(2, s.ballTimes.size());
            assertTrue("pre-spin " + preSpin + ": ball 2 went in at " + s.speedAtBall.get(1),
                    s.speedAtBall.get(1) >= s.target - BioBuzzConstants.LAUNCHER_READY_TOLERANCE);
            assertEquals(0, s.f.jams());
        }
    }

    /**
     * RealSim's windmill (0.40 power, 60-76 ms loops, motor lag) with PocketSim's flywheel (330 t/s dip per ball, back
     * ~0.4 s later) and pockets like the 7 Nov logs: 2.7 pockets per ball; of the balls, 58 % reach the flywheel 0.3-0.8
     * through their pocket, 19 % at its end, 13 % 0.01-0.08 s and 10 % 0.08-0.2 s after the windmill stopped.
     */
    private static class RealPocketSim {
        static final double P = BioBuzzConstants.INDEXER_TICKS_PER_POCKET;
        final ShotFeeder f;
        final java.util.Random rnd = new java.util.Random(5);
        final LauncherGate gate = LauncherGate.standard();
        final boolean[] ball;
        final double[] at, late;
        double t = 0, ticks = 0, w = 0, vel = 1150, target = 1150;
        int nextBall = 0, balls = 0, slowBalls = 0;
        final java.util.List<Double> due = new java.util.ArrayList<>();

        RealPocketSim(boolean skip, long seed, int pockets) {
            this(skip, seed, pockets, BioBuzzConstants.SHOT_EMPTY_WAIT_SEC);
        }

        RealPocketSim(boolean skip, long seed, int pockets, double emptyWait) {
            f = new ShotFeeder(P, 0.40, 0.35, 0.25, 1.0, 0.25, 3, P, 0.15).skipEmptyPockets(skip, emptyWait);
            java.util.Random r = new java.util.Random(seed);
            ball = new boolean[pockets];
            at = filled(pockets, 1.0);
            late = new double[pockets];
            for (int k = 0; k < pockets; k++) {
                ball[k] = r.nextDouble() < 1 / 2.7;
                double u = r.nextDouble();
                if (u < 0.58) {
                    at[k] = 0.3 + 0.5 * r.nextDouble();
                } else if (u < 0.77) {
                    late[k] = 0;
                } else if (u < 0.90) {
                    late[k] = 0.01 + 0.07 * r.nextDouble();
                } else {
                    late[k] = 0.08 + 0.12 * r.nextDouble();
                }
            }
            gate.update(-1.0, target, vel);
        }

        int ballsIn() {
            int n = 0;
            for (boolean x : ball) {
                n += x ? 1 : 0;
            }
            return n;
        }

        double run() {
            int n = ballsIn();
            while (balls < n && t < 120) {
                boolean ready = gate.update(t, target, vel);
                double p = f.update(t, (int) Math.round(ticks), ready, true, true);
                double dt = 0.060 + rnd.nextDouble() * 0.016;
                for (int ms = 0; ms < Math.round(dt * 1000); ms++) {
                    double wss = p == 0 ? 0 : Math.signum(p) * Math.max(0, 2200 * Math.abs(p) - 30);
                    w += (wss - w) * 0.001 / (p == 0 ? 0.012 : 0.015);
                    ticks += w * 0.001;
                    double now = t + ms * 0.001;
                    while (nextBall < ball.length && ticks >= (nextBall + at[nextBall]) * P) {
                        if (ball[nextBall]) {
                            due.add(now + late[nextBall]);
                        }
                        nextBall++;
                    }
                    for (java.util.Iterator<Double> it = due.iterator(); it.hasNext(); ) {
                        if (now >= it.next()) {
                            it.remove();
                            balls++;
                            if (vel < target - BioBuzzConstants.LAUNCHER_READY_TOLERANCE) {
                                slowBalls++;
                            }
                            vel -= 330;
                        }
                    }
                    vel = Math.min(target, vel + 0.8);                // 800 t/s per s
                }
                t += dt;
            }
            return t;
        }
    }

    @Test
    public void realisticEmptyPocketsSkipSavesTimeAndRarelyFollowsALateBall() {
        // a ball pushed in while a late one is on its way can't be stopped: the empty wait sets how often (see
        // SHOT_EMPTY_WAIT_SEC: 0.05 s = 19 % faster, 1.6 % of balls into a slowed flywheel; 0.10 s = 11 %, 0.45 %)
        double plainT = 0, skipT = 0;
        int plainSlow = 0, skipSlow = 0, n = 0;
        for (long seed = 1; seed <= 100; seed++) {
            RealPocketSim plain = new RealPocketSim(false, seed, 40);
            RealPocketSim skip = new RealPocketSim(true, seed, 40);
            plainT += plain.run();
            skipT += skip.run();
            assertEquals(plain.ballsIn(), plain.balls);
            assertEquals(skip.ballsIn(), skip.balls);
            plainSlow += plain.slowBalls;
            skipSlow += skip.slowBalls;
            n += plain.balls;
            assertEquals(0, skip.f.jams());
        }
        System.out.printf("realistic: %d balls, plain %.2f s/ball (%d slow), skip %.2f s/ball (%d slow), %.1f %% faster%n",
                n, plainT / n, plainSlow, skipT / n, skipSlow, 100 * (1 - skipT / plainT));
        assertEquals("the plain pause never follows a late ball", 0, plainSlow);
        assertTrue("skip " + skipT / n + " s/ball vs plain " + plainT / n, skipT < 0.94 * plainT);
        assertTrue(skipSlow + " of " + n + " balls into a slowed flywheel", skipSlow <= 0.01 * n);
    }

    private static double[] filled(int n, double v) {
        double[] a = new double[n];
        java.util.Arrays.fill(a, v);
        return a;
    }
}

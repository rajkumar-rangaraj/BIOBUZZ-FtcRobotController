package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;
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
}

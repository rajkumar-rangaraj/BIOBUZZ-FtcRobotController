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
        int ballsOut = 0;
        double lastBallBoundary = 0;
        final ShotFeeder f = new ShotFeeder(POCKET, 0.6, 0.35, 0.35, 1.0, 0.25, 3);

        boolean ready() {
            return vel >= target - 100;
        }

        void step(boolean wantFire) {
            double p = f.update(t, (int) Math.round(ticks), ready(), wantFire);
            if (!(jammed && p > 0)) {
                ticks += p * ticksPerSecAtFull * DT;
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
        for (int i = 0; i < 300; i++) {                              // hold FIRE for 3 s
            double velBefore = s.vel;
            s.step(true);
            if (s.ballsOut > lastBalls) {
                minSpeedAtShot = Math.min(minSpeedAtShot, velBefore);
                lastBalls = s.ballsOut;
            }
        }
        assertTrue("several shots in 3 s, got " + s.f.shots(), s.f.shots() >= 4);
        assertEquals("one ball per counted shot", s.f.shots(), s.ballsOut);
        assertTrue("every ball left at speed (>= target-100), worst " + minSpeedAtShot, minSpeedAtShot >= 1150);
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
}

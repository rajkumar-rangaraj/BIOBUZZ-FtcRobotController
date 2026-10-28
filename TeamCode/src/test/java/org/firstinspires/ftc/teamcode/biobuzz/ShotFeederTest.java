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

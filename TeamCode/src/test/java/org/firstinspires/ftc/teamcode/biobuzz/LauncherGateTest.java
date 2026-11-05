package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Flywheel ready-to-fire gate: settled inside the band for the hold time, with a fallback so it can't block forever. */
public class LauncherGateTest {

    private static LauncherGate gate() {
        return new LauncherGate(30, 0.10, 100, 1.0);
    }

    @Test
    public void readyOnlyAfterHoldingInsideTheBand() {
        LauncherGate g = gate();
        assertFalse(g.update(0.00, 1150, 900));             // spinning up
        assertFalse(g.update(0.05, 1150, 1140));            // in the band, not held yet
        assertFalse(g.update(0.10, 1150, 1160));
        assertTrue(g.update(0.16, 1150, 1140));             // 0.11 s inside
        assertFalse(g.usedFallback());
    }

    @Test
    public void oneLuckyReadingIsNotEnough() {
        LauncherGate g = gate();
        assertFalse(g.update(0.00, 1150, 1150));
        assertFalse(g.update(0.05, 1150, 1100));            // dropped out (50 below)
        assertFalse(g.update(0.10, 1150, 1150));            // back in: the hold starts again
        assertFalse(g.update(0.18, 1150, 1150));
        assertTrue(g.update(0.21, 1150, 1150));
    }

    @Test
    public void theOldMarginStillFiresAfterTheFallbackTime() {
        LauncherGate g = gate();
        // weak battery: the speed settles 60 below the target, outside the band but inside the old 100 margin
        assertFalse(g.update(0.0, 1150, 1090));
        assertFalse(g.update(0.9, 1150, 1090));
        assertTrue(g.update(1.0, 1150, 1090));
        assertTrue(g.usedFallback());
        // too slow even for the old rule: never ready
        LauncherGate slow = gate();
        assertFalse(slow.update(0.0, 1150, 1000));
        assertFalse(slow.update(5.0, 1150, 1000));
    }

    @Test
    public void aShotDipMakesItWaitForTheRecovery() {
        LauncherGate g = gate();
        g.update(0.0, 1150, 1150);
        assertTrue(g.update(0.2, 1150, 1150));
        assertFalse(g.update(0.25, 1150, 1060));            // ball passed: 90 t/s dip (still above the old margin)
        assertFalse(g.update(0.40, 1150, 1130));            // recovering, back in the band
        assertFalse(g.update(0.45, 1150, 1150));
        assertTrue(g.update(0.51, 1150, 1150));
        // later in the match the same dip must still wait: the fallback is timed from the last settled moment
        assertTrue(g.update(3.00, 1150, 1150));
        assertFalse(g.update(3.05, 1150, 1060));
        assertFalse(g.usedFallback());
    }

    @Test
    public void huntingAcrossTheBandEdgeFallsBackInsteadOfBlocking() {
        LauncherGate g = gate();
        boolean ready = false;
        for (int i = 0; i <= 40 && !ready; i++) {           // 25 ms loop, readings alternate 1120 (in) / 1100 (out)
            ready = g.update(i * 0.025, 1150, i % 2 == 0 ? 1120 : 1100);
        }
        assertTrue(ready);
        assertTrue(g.usedFallback());
    }

    @Test
    public void aNewTargetMustSettleAgainAndOffIsNeverReady() {
        LauncherGate g = gate();
        g.update(0.0, 1150, 1150);
        assertTrue(g.update(0.2, 1150, 1150));
        assertFalse(g.update(0.25, 1165, 1150));            // trim +15: still inside the band, but the hold restarts
        assertTrue(g.update(0.36, 1165, 1160));
        assertFalse(g.update(0.40, 0, 1160));               // launcher off
        assertFalse(g.update(0.60, -5, 1160));
    }

    @Test
    public void overspeedIsNotReady() {
        LauncherGate g = gate();
        g.update(0.0, 1150, 1240);
        assertFalse(g.update(0.5, 1150, 1240));             // above the band (e.g. just switched FAR -> NEAR)
        assertFalse(g.update(2.0, 1150, 1240));             // and the fallback never fires a ball long
        assertFalse(g.usedFallback());
    }

    @Test
    public void launcherOffAndOnRestartsTheFallbackTimer() {
        LauncherGate g = gate();
        g.update(0.0, 1150, 1090);
        assertFalse(g.update(0.9, 1150, 1090));
        assertFalse(g.update(0.95, 0, 0));                  // launcher off
        assertFalse(g.update(1.0, 1150, 1090));             // on again: the timer starts over
        assertFalse(g.update(1.9, 1150, 1090));
        assertTrue(g.update(2.0, 1150, 1090));
    }

    @Test
    public void sameTimeGivesTheSameAnswer() {
        LauncherGate g = gate();
        g.update(0.0, 1150, 1150);
        assertTrue(g.update(0.2, 1150, 1150));
        assertTrue(g.update(0.2, 1150, 1150));
        assertFalse(g.update(0.25, 1150, 1050));
        assertFalse(g.update(0.25, 1150, 1050));
    }

    @Test
    public void huntingDoesNotFallBackEarly() {
        LauncherGate g = gate();
        for (int i = 0; i < 39; i++) {                      // up to 0.95 s, readings 1120 / 1100
            assertFalse(g.update(i * 0.025, 1150, i % 2 == 0 ? 1120 : 1100));
        }
    }
}

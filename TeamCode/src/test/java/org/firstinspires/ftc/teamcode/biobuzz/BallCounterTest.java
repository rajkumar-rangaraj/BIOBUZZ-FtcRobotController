package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Ball counting from flywheel dips: only balls the windmill actually fed. */
public class BallCounterTest {

    private static BallCounter auto() {
        return new BallCounter(35, 0.6, 1);
    }

    @Test
    public void aBallAfterAPocketCountsOnce() {
        BallCounter c = auto();
        assertFalse(c.update(0.00, true, 1240, 1240));      // pocket turning
        assertFalse(c.update(0.10, false, 1240, 1240));     // pocket done
        assertTrue(c.update(0.15, false, 1240, 1180));      // the ball goes through the wheel
        assertFalse(c.update(0.20, false, 1240, 1160));     // same dip
        assertFalse(c.update(0.40, false, 1240, 1240));     // recovered
        assertEquals(1, c.balls());
    }

    @Test
    public void noiseWithNothingFedNeverCounts() {
        BallCounter c = auto();                             // GPT review: 1200/1240 readings at FAR, flywheel idle
        for (int i = 0; i < 200; i++) {
            c.update(i * 0.02, false, 1240, i % 2 == 0 ? 1200 : 1240);
        }
        assertEquals(0, c.balls());
        assertEquals(100, c.ignoredDips());
    }

    @Test
    public void spinUpAndSpeedChangesAreNotBalls() {
        BallCounter c = auto();
        assertFalse(c.update(0.0, false, 1150, 0));         // spin-up from rest
        assertFalse(c.update(1.0, false, 1150, 1150));
        assertFalse(c.update(1.1, false, 1240, 1150));      // NEAR -> FAR
        assertFalse(c.update(1.5, false, 1240, 1240));
        assertEquals(0, c.balls());
    }

    @Test
    public void onlyOneBallPerPocketInAuto() {
        BallCounter c = auto();
        c.update(0.00, true, 1150, 1150);
        assertTrue(c.update(0.05, true, 1150, 1100));       // dip while the pocket still turns
        c.update(0.10, false, 1150, 1150);
        assertFalse(c.update(0.15, false, 1150, 1100));     // a second dip in the same window: not counted
        c.update(0.30, false, 1150, 1150);
        c.update(0.60, true, 1150, 1150);                   // next pocket
        assertTrue(c.update(0.70, false, 1150, 1090));
        assertEquals(2, c.balls());
    }

    @Test
    public void lateDipAfterTheWindowIsIgnored() {
        BallCounter c = auto();
        c.update(0.0, true, 1150, 1150);
        c.update(0.1, false, 1150, 1150);                   // armed until 0.6
        assertFalse(c.update(0.8, false, 1150, 1100));
        assertEquals(0, c.balls());
    }

    @Test
    public void tuningCountsTwoBallsFromOnePocket() {
        BallCounter c = new BallCounter(35, 0.6, 2);
        c.update(0.00, true, 1150, 1150);
        assertTrue(c.update(0.10, false, 1150, 1100));
        c.update(0.20, false, 1150, 1150);
        assertTrue(c.update(0.30, false, 1150, 1100));
        c.update(0.40, false, 1150, 1150);
        assertFalse(c.update(0.50, false, 1150, 1100));     // a third: not plausible, ignored
        assertEquals(2, c.balls());
        c.reset();
        assertEquals(0, c.balls());
    }
}

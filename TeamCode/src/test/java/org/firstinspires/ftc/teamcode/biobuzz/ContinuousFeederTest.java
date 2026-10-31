package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** AUTO continuous windmill feed: pauses for the flywheel, backs off on a stall, stops after the set travel. */
public class ContinuousFeederTest {

    private final ContinuousFeeder f = new ContinuousFeeder(200, 0.3, 0.35, 0.25);

    @Test
    public void feedsOnlyWhileReadyAndStopsAfterTravel() {
        assertEquals(0.3, f.update(0, 10, true), 0);
        assertEquals(0, f.update(0.1, 50, false), 0);           // flywheel recovering: pause
        assertEquals(0.3, f.update(0.2, 50, true), 0);
        assertFalse(f.done(209));
        assertTrue(f.done(210));
        assertEquals(0, f.update(0.3, 210, true), 0);
    }

    @Test
    public void stallBacksOffThenFeedsAgain() {
        f.update(0, 0, true);
        f.update(0.3, 2, true);
        assertEquals(-0.35, f.update(0.7, 3, true), 0);         // no progress for > 0.6 s
        assertEquals(1, f.stalls());
        assertEquals(-0.35, f.update(0.8, -10, true), 0);
        assertEquals(0.3, f.update(1.0, -12, true), 0);         // back-off over
        assertEquals(0.3, f.update(1.3, 20, true), 0);
        assertEquals(1, f.stalls());
    }

    @Test
    public void waitingForTheFlywheelIsNotAStall() {
        f.update(0, 0, true);
        for (double t = 0.1; t < 2; t += 0.1) {
            assertEquals(0, f.update(t, 0, false), 0);
        }
        assertEquals(0.3, f.update(2.1, 0, true), 0);
        assertEquals(0, f.stalls());
    }
}

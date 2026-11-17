package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Indexer v2: the two flywheel motors' encoders must agree (one shaft); a dead or reversed one stops both. */
public class FlywheelPairCheckTest {

    private final FlywheelPairCheck c = new FlywheelPairCheck(50, 300, 150, 0.3);

    @Test
    public void spinUpAndNormalNoiseAreFine() {
        for (int i = 0; i <= 60; i++) {                          // 3 s spin-up to 1400 with 20 t/s steps between them
            double v = Math.min(1400, i * 40);
            assertNull(c.update(i * 0.05, v, v + (i % 2 == 0 ? 20 : -20)));
        }
        assertNull(c.update(3.1, 1400, 1300));                   // a ball's dip read a few ms apart: < 150
    }

    @Test
    public void unpluggedEncoderIsFoundAfterTheFaultTime() {
        assertNull(c.update(0.0, 1400, 0));
        assertNull("one bad pair is not a fault", c.update(0.2, 1400, 0));
        String why = c.update(0.31, 1400, 0);
        assertNotNull(why);
        assertTrue(why, why.contains("launcher2 reads ~0"));
        c.reset();
        assertNull(c.update(1.0, 0, 1400));
        assertTrue(c.update(1.3, 10, 1400).contains("launcher reads ~0"));
    }

    @Test
    public void reversedEncoderOrBigDifference() {
        c.update(0, 1400, -1400);
        assertTrue(c.update(0.4, 1400, -1400).contains("LAUNCHER2_DIRECTION"));
        FlywheelPairCheck d = new FlywheelPairCheck(50, 300, 150, 0.3);
        d.update(0, 1400, 1100);
        assertTrue(d.update(0.35, 1400, 1100).contains("disagree"));
    }

    @Test
    public void aShortDisagreementResets() {
        c.update(0, 1400, 0);
        c.update(0.2, 1400, 0);
        assertNull(c.update(0.25, 1400, 1400));                  // back in step: the timer starts again
        assertNull(c.update(0.4, 1400, 0));
        assertNull(c.update(0.6, 1400, 0));
        assertNotNull(c.update(0.71, 1400, 0));
    }

    @Test
    public void bothStoppedIsFine() {
        assertNull(c.update(0, 0, 0));
        assertNull(c.update(1, 10, -10));
    }
}

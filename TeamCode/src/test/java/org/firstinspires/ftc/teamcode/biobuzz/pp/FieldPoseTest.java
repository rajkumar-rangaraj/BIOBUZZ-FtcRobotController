package org.firstinspires.ftc.teamcode.biobuzz.pp;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Field pose from raw Pinpoint readings: start, heading change, history, glitches, camera snaps. */
public class FieldPoseTest {

    private static FieldPose started(double x, double y, double h) {
        FieldPose p = FieldPose.standard();
        p.update(0, 5, -3, 30, true);                       // raw frame: anywhere
        p.setPose(x, y, h);
        return p;
    }

    @Test
    public void nothingIsKnownBeforeAStart() {
        FieldPose p = FieldPose.standard();
        p.update(0, 0, 0, 0, true);
        assertFalse(p.headingKnown());
        assertFalse(p.positionKnown());
        assertNull(p.fieldAt(0));
    }

    @Test
    public void rawMovesAreTurnedIntoTheField() {
        FieldPose p = started(-12.7, -61.5, -90);           // CELL-wall start, intake toward the audience wall
        assertEquals(-12.7, p.x(), 1e-9);
        assertEquals(-61.5, p.y(), 1e-9);
        assertEquals(-90, p.headingDeg(), 1e-9);
        // raw: 10 in forward along the raw heading (30 deg) = 10 in toward the audience wall on the field (-Y)
        p.update(0.2, 5 + 10 * Math.cos(Math.toRadians(30)), -3 + 10 * Math.sin(Math.toRadians(30)), 30, true);
        assertEquals(-12.7, p.x(), 1e-9);
        assertEquals(-71.5, p.y(), 1e-9);
    }

    @Test
    public void newHeadingKeepsThePosition() {
        FieldPose p = started(10, 20, 0);
        p.setHeading(45);
        assertEquals(10, p.x(), 1e-9);
        assertEquals(20, p.y(), 1e-9);
        assertEquals(45, p.headingDeg(), 1e-9);
    }

    @Test
    public void historyGivesThePoseWhenAFrameWasTaken() {
        FieldPose p = started(0, -50, 0);
        for (int i = 1; i <= 10; i++) {                     // raw heading 30: raw forward = field +X after the start
            double d = 2.0 * i;
            p.update(0.05 * i, 5 + d * Math.cos(Math.toRadians(30)), -3 + d * Math.sin(Math.toRadians(30)), 30, true);
        }
        double[] at = p.fieldAt(0.125);                     // halfway between 0.10 and 0.15 = 5 in
        assertNotNull(at);
        assertEquals(5, at[0], 1e-6);
        assertEquals(-50, at[1], 1e-6);
        assertEquals(20, p.x(), 1e-6);
        assertEquals(40, p.speedIps(), 1e-6);
    }

    @Test
    public void aSnapAtAnOlderFrameMovesTheWholePose() {
        FieldPose p = started(0, -50, 0);
        p.update(0.1, 5 + 10 * Math.cos(Math.toRadians(30)), -3 + 10 * Math.sin(Math.toRadians(30)), 30, true);
        assertTrue(p.snapAt(0, 1, -48));                    // the camera says we were at (1, -48), not (0, -50)
        assertEquals(11, p.x(), 1e-6);
        assertEquals(-48, p.y(), 1e-6);
    }

    @Test
    public void aRawJumpDropsThePose() {
        FieldPose p = started(0, -50, 0);
        p.update(0.02, 40, 40, 0, true);                    // a Pinpoint restart reads elsewhere
        assertNotNull(p.event());
        assertFalse(p.positionKnown());
        assertFalse(p.headingKnown());
    }

    @Test
    public void aHeadingOnlyRestartDropsThePose() {
        FieldPose p = started(0, -50, 0);                   // raw heading 30 at the start
        p.update(0.05, 5, -3, -60, true);                   // x / y unchanged, heading reset: 90 deg in 0.05 s
        assertNotNull(p.event());
        assertFalse(p.headingKnown());
        assertFalse(p.positionKnown());
    }

    @Test
    public void aStartSetBeforeTheFirstGoodReadingWaitsForIt() {
        FieldPose p = FieldPose.standard();
        p.update(0, 0, 0, 0, false);                        // START during a bad read
        p.setPose(-12.7, -61.5, -90);
        assertTrue(p.pending());
        assertFalse(p.positionKnown());
        p.update(0.03, 4, 1, 20, true);                     // first good reading
        assertFalse(p.pending());
        assertTrue(p.positionKnown());
        assertEquals(-12.7, p.x(), 1e-9);
        assertEquals(-61.5, p.y(), 1e-9);
        assertEquals(-90, p.headingDeg(), 1e-9);
    }

    @Test
    public void aStartWaitingTooLongIsDropped() {
        FieldPose p = FieldPose.standard();
        p.update(0, 0, 0, 0, false);
        p.setPose(-12.7, -61.5, -90);
        for (int i = 1; i <= 30; i++) {
            p.update(0.05 * i, 0, 0, 0, false);             // 1.5 s without a good reading: the robot may have moved
        }
        p.update(1.55, 20, 5, 40, true);
        assertFalse(p.pending());
        assertFalse(p.positionKnown());
        assertFalse(p.headingKnown());
        assertNotNull(p.event());
    }

    @Test
    public void aGapInReadingsWhileDrivingIsNotAJump() {
        FieldPose p = started(0, -50, 0);
        for (int i = 1; i <= 10; i++) {
            p.update(0.05 * i, 0, 0, 0, false);             // 0.5 s of bad reads ...
        }
        p.update(0.55, 5 + 33 * Math.cos(Math.toRadians(30)), -3 + 33 * Math.sin(Math.toRadians(30)), 30, true);
        assertNull(p.event());                              // ... while driving 60 in/s: 33 in further is fine
        assertTrue(p.positionKnown());
        assertEquals(33, p.x(), 1e-6);
    }

    @Test
    public void badReadsAreIgnored() {
        FieldPose p = started(0, -50, 0);
        p.update(0.02, 0, 0, 0, false);
        assertFalse(p.rawOk());
        assertTrue(p.positionKnown());
        assertEquals(0, p.x(), 1e-9);
    }

    @Test
    public void positionFixWhileOnlyTheHeadingIsKnown() {
        FieldPose p = FieldPose.standard();
        p.update(0, 1, 2, 10, true);
        p.setHeading(-90);
        assertTrue(p.headingKnown());
        assertFalse(p.positionKnown());
        assertTrue(p.snapAt(0, -12.5, -58));
        assertTrue(p.positionKnown());
        assertEquals(-58, p.y(), 1e-9);
    }
}

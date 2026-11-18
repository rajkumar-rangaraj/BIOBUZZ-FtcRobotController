package org.firstinspires.ftc.teamcode.biobuzz.pp;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Camera sighting -> robot position: geometry, first fix, corrections, outliers, snaps, swapped names. */
public class TagFixTest {

    private static final double CAM = 8;                    // camera 8 in behind the centre
    private static final double AX = -12.5, AY = -17, BX = -12.5, BY = 16.5;

    private static CellMap cells() {
        return new CellMap(AX, AY, BX, BY, true);
    }

    /** What the camera sees of a CELL from a robot at (x, y, h): {fwd, right}. */
    private static double[] sight(double x, double y, double h, double cx, double cy) {
        double c = Math.cos(Math.toRadians(h)), s = Math.sin(Math.toRadians(h));
        double ex = cx - x, ey = cy - y;
        double rx = c * ex + s * ey, ry = -s * ex + c * ey;  // robot frame: x forward, y left
        return new double[]{-rx - CAM, ry};
    }

    private static FieldPose headingOnly(double h, double t) {
        FieldPose p = FieldPose.standard();
        p.update(t, 0, 0, 0, true);
        p.setHeading(h);
        return p;
    }

    @Test
    public void geometryRoundTrips() {
        double[] v = sight(-12.5, -58, -90, AX, AY);        // straight in front of A: 33 in, dead ahead
        assertEquals(33, v[0], 1e-9);
        assertEquals(0, v[1], 1e-9);
        double[] r = TagFix.robotXY(-90, v[0], v[1], AX, AY, CAM, 0);
        assertEquals(-12.5, r[0], 1e-9);
        assertEquals(-58, r[1], 1e-9);
        double[] w = sight(-30, -50, -60, AX, AY);
        double[] r2 = TagFix.robotXY(-60, w[0], w[1], AX, AY, CAM, 0);
        assertEquals(-30, r2[0], 1e-9);
        assertEquals(-50, r2[1], 1e-9);
        double[] c = TagFix.cellXY(-30, -50, -60, w[0], w[1], CAM, 0);
        assertEquals(AX, c[0], 1e-9);
        assertEquals(AY, c[1], 1e-9);
    }

    @Test
    public void floorVectorUndoesTheTilt() {
        double t = Math.toRadians(30);                      // a point 33 in out on the floor, 40 in up, seen tilted 30 deg
        double fwd = 33, up = 40;
        double y = fwd * Math.cos(t) + up * Math.sin(t), z = -fwd * Math.sin(t) + up * Math.cos(t);
        double[] v = TagFix.floorVector(2, y, z, 30);
        assertEquals(33, v[0], 1e-9);
        assertEquals(2, v[1], 1e-9);
        assertEquals(Math.toDegrees(Math.atan2(-2, 33)), TagFix.bearingDeg(v[0], v[1]), 1e-9);
    }

    @Test
    public void firstFixPlacesThePose() {
        FieldPose p = headingOnly(-90, 0);
        TagFix f = new TagFix(CAM, 0);
        double[] v = sight(-12.5, -58, -90, AX, AY);
        assertEquals(TagFix.Result.FIRST_FIX, f.apply(0.05, p, cells(), "RED_REAR", 4, v[0], v[1], 51, 0));
        assertTrue(p.positionKnown());
        assertEquals(-12.5, p.x(), 1e-6);
        assertEquals(-58, p.y(), 1e-6);
    }

    @Test
    public void downCellsAndFarOnesAreNotUsed() {
        FieldPose p = headingOnly(-90, 0);
        TagFix f = new TagFix(CAM, 0);
        double[] v = sight(-12.5, -58, -90, AX, AY);
        assertEquals(TagFix.Result.DOWN, f.apply(0.05, p, cells(), "RED_REAR", 4, v[0], v[1], 30, 0));
        assertEquals(TagFix.Result.FAR, f.apply(0.05, p, cells(), "RED_REAR", 4, 80, 0, 51, 0));
        assertEquals(TagFix.Result.OLD_FRAME, f.apply(1.0, p, cells(), "RED_REAR", 4, v[0], v[1], 51, 0));
        assertFalse(p.positionKnown());
    }

    @Test
    public void anImpossibleFirstFixTriesTheOtherPlace() {
        // Robot near the rear wall facing B, but the name says A: from A it would be inside the HIVE frame
        FieldPose p = headingOnly(90, 0);
        TagFix f = new TagFix(CAM, 0);
        CellMap m = cells();
        double[] v = sight(-12.5, 50, 90, BX, BY);
        assertEquals(TagFix.Result.FIRST_FIX, f.apply(0.05, p, m, "RED_REAR", 2, v[0], v[1], 51, 0));
        assertEquals(50, p.y(), 1e-6);
        assertEquals(CellMap.Place.B, m.place("RED_REAR"));     // the mapping follows the fix ...
        assertEquals(CellMap.Place.B, m.target());              // ... and so does the one CELL seen UP
        assertEquals(1, m.upStreak(CellMap.Place.B));           // one frame = one sighting
    }

    @Test
    public void fixesPullAnOffPoseBackAndOutliersSnapOnlyWhenTheyAgree() {
        FieldPose p = FieldPose.standard();
        p.update(0, 0, 0, 0, true);
        p.setPose(-12.5, -55, -90);                         // pose 3 in off: the robot is at -58
        TagFix f = new TagFix(CAM, 0);
        double[] v = sight(-12.5, -58, -90, AX, AY);
        for (int i = 0; i < 20; i++) {
            p.update(0.05 * (i + 1), 0, 0, 0, true);
            assertEquals(TagFix.Result.CORRECTED, f.apply(0.05 * (i + 1), p, cells(), "RED_REAR", 4, v[0], v[1], 51,
                    0.05 * (i + 1)));
        }
        assertEquals(-58, p.y(), 0.01);

        double[] bad = sight(-12.5, -40, -90, AX, AY);      // 18 in off: outliers ...
        double t = 1.1;
        for (int i = 0; i < 3; i++, t += 0.05) {
            p.update(t, 0, 0, 0, true);
            assertEquals(TagFix.Result.OUTLIER, f.apply(t, p, cells(), "RED_REAR", 4, bad[0], bad[1], 51, t));
        }
        p.update(t, 0, 0, 0, true);                         // ... the 4th that agrees snaps
        assertEquals(TagFix.Result.SNAPPED, f.apply(t, p, cells(), "RED_REAR", 4, bad[0], bad[1], 51, t));
        assertEquals(-40, p.y(), 1e-6);
    }

    @Test
    public void swappedNamesAreLearnedFromAnIndependentStartPose() {
        CellMap m = new CellMap(AX, AY, BX, BY, false);     // first guess wrong: RED_REAR mapped to B
        FieldPose p = FieldPose.standard();
        p.update(0, 0, 0, 0, true);
        p.setPose(-12.5, -58, -90);                         // known start in front of A
        TagFix f = new TagFix(CAM, 0);
        f.poseFromStart();
        double[] v = sight(-12.5, -58, -90, AX, AY);
        TagFix.Result last = null;
        for (int i = 1; i <= 3; i++) {
            p.update(0.05 * i, 0, 0, 0, true);
            last = f.apply(0.05 * i, p, m, "RED_REAR", 4, v[0], v[1], 51, 0.05 * i);
        }
        assertEquals(CellMap.Place.A, m.place("RED_REAR"));
        assertEquals(TagFix.Result.CORRECTED, last);
        assertTrue(f.namesConfirmed());
        assertEquals(-58, p.y(), 1e-6);
    }

    @Test
    public void onceNamesAreConfirmedAPoseOffByTheCellGapSnapsInsteadOfRenaming() {
        CellMap m = cells();
        FieldPose p = FieldPose.standard();
        p.update(0, 0, 0, 0, true);
        p.setPose(-12.5, -58, -90);
        TagFix f = new TagFix(CAM, 0);
        f.poseFromStart();
        double[] v = sight(-12.5, -58, -90, AX, AY);
        p.update(0.05, 0, 0, 0, true);
        assertEquals(TagFix.Result.CORRECTED, f.apply(0.05, p, m, "RED_REAR", 4, v[0], v[1], 51, 0.05));
        assertTrue(f.namesConfirmed());
        p.shift(0, 33.5);                                   // the pose is now off by exactly A -> B
        double t = 0.1;
        TagFix.Result r = null;
        for (int i = 0; i < 4; i++, t += 0.05) {
            p.update(t, 0, 0, 0, true);
            r = f.apply(t, p, m, "RED_REAR", 4, v[0], v[1], 51, t);
        }
        assertEquals(TagFix.Result.SNAPPED, r);
        assertEquals(CellMap.Place.A, m.place("RED_REAR"));  // names untouched
        assertEquals(-58, p.y(), 1e-6);
    }

    @Test
    public void cameraStartWithAWrongNameGuessIsFixedByTheOtherCell() {
        CellMap m = new CellMap(AX, AY, BX, BY, false);     // RED_REAR guessed at B, really at A
        FieldPose p = headingOnly(-165, 0);
        TagFix f = new TagFix(CAM, 0);
        double rx = -50, ry = -10, h = -165;                // the robot really is here
        double[] a = sight(rx, ry, h, AX, AY), b = sight(rx, ry, h, BX, BY);
        p.update(0.05, 0, 0, 0, true);
        assertEquals(TagFix.Result.FIRST_FIX, f.apply(0.05, p, m, "RED_REAR", 4, a[0], a[1], 51, 0.05));
        assertEquals(23.5, p.y(), 1e-6);                    // placed from the wrong guess: 33.5 in off
        assertFalse(f.namesConfirmed());
        p.update(0.10, 0, 0, 0, true);
        assertEquals(TagFix.Result.OUTLIER, f.apply(0.10, p, m, "RED_AUDIENCE", 4, b[0], b[1], 51, 0.10));
        p.update(0.15, 0, 0, 0, true);
        assertEquals(TagFix.Result.SNAPPED, f.apply(0.15, p, m, "RED_AUDIENCE", 4, b[0], b[1], 51, 0.15));
        assertEquals(CellMap.Place.A, m.place("RED_REAR"));
        assertEquals(CellMap.Place.B, m.place("RED_AUDIENCE"));
        assertEquals(rx, p.x(), 1e-6);
        assertEquals(ry, p.y(), 1e-6);
        assertTrue(f.namesConfirmed());
    }

    @Test
    public void cameraStartWithARightNameGuessIsConfirmedByTheOtherCell() {
        CellMap m = cells();
        FieldPose p = headingOnly(-165, 0);
        TagFix f = new TagFix(CAM, 0);
        double[] a = sight(-50, -10, -165, AX, AY), b = sight(-50, -10, -165, BX, BY);
        p.update(0.05, 0, 0, 0, true);
        f.apply(0.05, p, m, "RED_REAR", 4, a[0], a[1], 51, 0.05);
        p.update(0.10, 0, 0, 0, true);
        assertEquals(TagFix.Result.CORRECTED, f.apply(0.10, p, m, "RED_AUDIENCE", 4, b[0], b[1], 51, 0.10));
        assertTrue(f.namesConfirmed());
        assertEquals(-10, p.y(), 1e-6);
    }

    @Test
    public void movingFastIsSkipped() {
        FieldPose p = FieldPose.standard();
        p.update(0, 0, 0, 0, true);
        p.setPose(-12.5, -58, -90);
        p.update(0.05, 0, 0, 20, true);                     // 400 deg/s
        TagFix f = new TagFix(CAM, 0);
        double[] v = sight(-12.5, -58, -70, AX, AY);
        assertEquals(TagFix.Result.MOVING, f.apply(0.05, p, cells(), "RED_REAR", 4, v[0], v[1], 51, 0.05));
    }
}

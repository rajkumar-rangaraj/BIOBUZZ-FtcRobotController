package org.firstinspires.ftc.teamcode.fieldtour;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.List;

/** Field map + tour plan: every start tile, every segment stays on the field and out of the HIVE frame. */
public class TourPlannerTest {

    @Test
    public void tilesCoverTheField() {
        assertEquals(-58.75, FieldMap.tileCentreX(1), 1e-9);
        assertEquals(58.75, FieldMap.tileCentreX(6), 1e-9);
        assertEquals(23.5, FieldMap.TILE_IN, 1e-9);
    }

    @Test
    public void centreTilesAreUnderTheHiveFrame() {
        assertNotNull(FieldMap.startProblem(FieldMap.tileCentreX(3), FieldMap.tileCentreY(3)));
        assertNotNull(FieldMap.startProblem(FieldMap.tileCentreX(4), FieldMap.tileCentreY(4)));
        assertNull(FieldMap.startProblem(FieldMap.tileCentreX(2), FieldMap.tileCentreY(2)));
    }

    @Test
    public void everyValidStartTileGivesASafeClosedTour() {
        int valid = 0;
        for (int c = 1; c <= 6; c++) {
            for (int r = 1; r <= 6; r++) {
                double hx = FieldMap.tileCentreX(c), hy = FieldMap.tileCentreY(r);
                if (FieldMap.startProblem(hx, hy) != null) {
                    continue;
                }
                valid++;
                List<TourPlanner.Point> pts = TourPlanner.plan(hx, hy);
                TourPlanner.Point last = pts.get(pts.size() - 1);
                assertEquals("ends at HOME (" + c + "," + r + ")", hx, last.x, 1e-9);
                assertEquals(hy, last.y, 1e-9);
                double px = hx, py = hy;
                for (TourPlanner.Point p : pts) {
                    assertTrue("on the field " + p.label, p == last || FieldMap.insideDriveLimit(p.x, p.y));
                    assertFalse("segment to " + p.label + " from tile " + c + "," + r + " enters the HIVE frame",
                            FieldMap.segmentHitsKeepOut(px, py, p.x, p.y));
                    px = p.x;
                    py = p.y;
                }
                assertTrue("tour length " + TourPlanner.length(hx, hy, pts),
                        TourPlanner.length(hx, hy, pts) > 300 && TourPlanner.length(hx, hy, pts) < 900);
            }
        }
        assertEquals("all tiles except the 8 at the HIVE frame", 28, valid);
    }

    @Test
    public void segmentKeepOutTest() {
        assertTrue(FieldMap.segmentHitsKeepOut(-60, 0, 60, 0));          // straight through the middle
        assertFalse(FieldMap.segmentHitsKeepOut(-60, 55, 60, 55));       // along the far side
        assertFalse(FieldMap.segmentHitsKeepOut(-60, -60, -60, 60));     // along a wall
    }

    @Test
    public void nearestPlace() {
        assertEquals(FieldMap.Place.BLUE_HIVE, FieldMap.nearest(20, 0));
        assertEquals(FieldMap.Place.FLOWER_4, FieldMap.nearest(60, 30));
    }
}

package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoField.*;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.fieldtour.FieldMap;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class AutoPlannerTest {

    private static final double FIRE_SEC = 8.0;     // aim + settle + 4 pockets, slow case

    @Test
    public void startsAreLegal() {
        for (Start s : Start.values()) {
            Pose p = start(s);
            assertNull(s.name(), AutoPlanner.footprintProblem(p, null));
            double[][] c = AutoPlanner.corners(p);
            boolean touching = false;
            for (double[] k : c) {
                touching |= Math.abs(Math.abs(k[0]) - WALL_IN) < 0.01 || Math.abs(Math.abs(k[1]) - WALL_IN) < 0.01;
                assertTrue(s + " in the LOADING ZONE", !(k[0] < ZONE_INNER_X && k[1] > ZONE_Y_MIN && k[1] < ZONE_Y_MAX));
            }
            assertTrue(s + " must touch the wall (G304.C)", touching);
        }
    }

    @Test
    public void slotsArePartlyInTheZoneOffTheWallAndApart() {
        for (Slot s : Slot.values()) {
            Pose p = slot(s);
            assertNull(AutoPlanner.footprintProblem(p, s.other()));
            double robotYMin = p.y - HALF_WIDTH_IN, robotYMax = p.y + HALF_WIDTH_IN;
            double inside = Math.min(robotYMax, ZONE_Y_MAX) - Math.max(robotYMin, ZONE_Y_MIN);
            assertEquals(PARK_IN_ZONE_IN, inside, 1e-9);
            assertTrue("intake must be off the wall for LEAVE", p.x - HALF_LENGTH_IN > -WALL_IN + 1.5);
            assertTrue("robot must reach into the zone", p.x - HALF_LENGTH_IN < ZONE_INNER_X);
        }
        double gap = (SLOT_A_Y - HALF_WIDTH_IN) - (SLOT_B_Y + HALF_WIDTH_IN);
        assertTrue("gap between the two parked robots: " + gap, gap >= 8);
    }

    @Test
    public void shotIsStraightOnAndOnOurSide() {
        assertEquals(CELL_X, SHOT.x, 1e-9);
        assertEquals(FACE_AUDIENCE, SHOT.headingDeg, 1e-9);
        assertNull(AutoPlanner.footprintProblem(SHOT, null));
    }

    @Test
    public void gardenSweepReachesEveryPollen() {
        Pose end = AutoPlanner.end(SHOT, AutoPlanner.toGarden());
        double front = end.x - HALF_LENGTH_IN;
        for (double bx : GARDEN_BALL_X) {
            assertTrue("intake front passes POLLEN at x " + bx, front <= bx);
        }
        assertTrue("POLLEN centre within the intake width",
                Math.abs(GARDEN_BALL_Y - GARDEN_LANE_Y) <= HALF_WIDTH_IN + 0.5);
    }

    @Test
    public void everyRouteIsClear() {
        for (Start s : Start.values()) {
            assertNull(s.name(), AutoPlanner.routeProblem(start(s), AutoPlanner.toShot(s), null));
        }
        assertNull(AutoPlanner.routeProblem(SHOT, AutoPlanner.toGarden(), null));
        Pose gardenEnd = AutoPlanner.end(SHOT, AutoPlanner.toGarden());
        assertNull(AutoPlanner.routeProblem(gardenEnd, AutoPlanner.gardenToShot(), null));
    }

    /** Parking must work from wherever a step is cut short, without touching the partner in the other slot. */
    @Test
    public void parkIsClearFromAnywhereOnTheRoutes() {
        List<Pose> from = new ArrayList<>();
        for (Start s : Start.values()) {
            from.addAll(samples(start(s), AutoPlanner.toShot(s)));
        }
        from.addAll(samples(SHOT, AutoPlanner.toGarden()));
        from.addAll(samples(AutoPlanner.end(SHOT, AutoPlanner.toGarden()), AutoPlanner.gardenToShot()));
        assertTrue(from.size() > 100);
        for (Slot slot : Slot.values()) {
            for (Pose p : from) {
                List<AutoPlanner.Leg> legs = AutoPlanner.park(p, slot);
                assertNull("park " + slot + " from " + p, AutoPlanner.routeProblem(p, legs, slot.other()));
                Pose end = AutoPlanner.end(p, legs);
                assertEquals(slot(slot).x, end.x, 1e-9);
                assertEquals(slot(slot).y, end.y, 1e-9);
            }
        }
    }

    @Test
    public void fullPlanFitsWithParkingFirst() {
        for (Start s : Start.values()) {
            for (Slot slot : Slot.values()) {
                double t = AutoPlanner.estimateSec(start(s), AutoPlanner.toShot(s)) + FIRE_SEC;
                assertTrue(s + "/" + slot + ": GARDEN must fit after the first volley",
                        AutoPlanner.fits(t, SHOT, AutoPlanner.toGarden(), 0.5, slot, 24));
                Pose gardenEnd = AutoPlanner.end(SHOT, AutoPlanner.toGarden());
                t += AutoPlanner.estimateSec(SHOT, AutoPlanner.toGarden()) + 0.5;
                double parkStart = AutoPlanner.parkStartSec(gardenEnd, slot, 24);
                double parkSec = AutoPlanner.estimateSec(gardenEnd, AutoPlanner.park(gardenEnd, slot));
                assertTrue(parkStart <= 24);
                assertTrue(s + "/" + slot + ": parked by " + (parkStart + parkSec),
                        parkStart + parkSec <= AutoPlanner.MATCH_SEC - AutoPlanner.PARK_MARGIN_SEC + 1e-9);
                assertTrue(s + "/" + slot + ": GARDEN done at " + t + " s", t < parkStart);
            }
        }
    }

    @Test
    public void parkStartMovesEarlierForALongDrive() {
        Pose far = SHOT;
        double est = AutoPlanner.estimateSec(far, AutoPlanner.park(far, Slot.A));
        assertEquals(Math.min(29.0, AutoPlanner.MATCH_SEC - AutoPlanner.PARK_MARGIN_SEC - est),
                AutoPlanner.parkStartSec(far, Slot.A, 29.0), 1e-9);
        assertTrue(AutoPlanner.parkStartSec(far, Slot.A, 29.0) < 29.0);
        assertEquals(10.0, AutoPlanner.parkStartSec(far, Slot.A, 10.0), 1e-9);
    }

    @Test
    public void blueIsRedTurnedAboutTheCentre() {
        Pose red = START_CELL_WALL;
        Pose blue = red.forAlliance(Alliance.BLUE);
        assertEquals(-red.x, blue.x, 1e-9);
        assertEquals(-red.y, blue.y, 1e-9);
        assertEquals(90, blue.headingDeg, 1e-9);
        assertTrue("BLUE start on the BLUE side", blue.x - HALF_WIDTH_IN > 0);
        Pose p = new Pose(-20, 33, 170);
        Pose back = p.mirror().mirror();
        assertEquals(p.x, back.x, 1e-9);
        assertEquals(p.y, back.y, 1e-9);
        assertEquals(p.headingDeg, back.headingDeg, 1e-9);
    }

    @Test
    public void footprintCheckCatchesProblems() {
        assertTrue(AutoPlanner.footprintProblem(new Pose(-20, 0, 0), null).contains("HIVE"));
        assertTrue(AutoPlanner.footprintProblem(new Pose(-5, -55, 90), null).contains("opponent"));
        assertTrue(AutoPlanner.footprintProblem(new Pose(-65, -40, 0), null).contains("outside"));
        FieldMap.Place f3 = FieldMap.Place.FLOWER_3;
        assertTrue(AutoPlanner.footprintProblem(new Pose(f3.x + 12, f3.y, 180), null).contains("Flower"));
        assertTrue(AutoPlanner.footprintProblem(new Pose(-45, SLOT_B_Y, 180), Slot.B).contains("partner"));
    }

    /** A blocked park re-plans from where it stopped: it must stay clear and never back out to the lane once in. */
    @Test
    public void parkReplanFromAlongTheParkRoute() {
        Pose gardenEnd = AutoPlanner.end(SHOT, AutoPlanner.toGarden());
        Pose[] origins = {SHOT, gardenEnd, START_ALLIANCE_WALL, START_CELL_WALL};
        for (Slot slot : Slot.values()) {
            assertTrue("a parked robot counts as in the zone", AutoPlanner.partlyInZone(slot(slot)));
            for (Pose o : origins) {
                for (Pose p : samples(o, AutoPlanner.park(o, slot))) {
                    List<AutoPlanner.Leg> legs = AutoPlanner.park(p, slot);
                    assertNull("replan " + slot + " from " + p, AutoPlanner.routeProblem(p, legs, slot.other()));
                    if (p.x < AutoField.LANE_X - 0.5 && Math.abs(p.y - slot(slot).y) < 1) {
                        assertEquals("straight in from " + p, 1, legs.size());
                    }
                }
            }
        }
        assertTrue(!AutoPlanner.partlyInZone(new Pose(LANE_X, SLOT_A_Y, FACE_ALLIANCE_WALL)));
    }

    private static List<Pose> samples(Pose from, List<AutoPlanner.Leg> legs) {
        List<Pose> out = new ArrayList<>();
        Pose p = from;
        for (AutoPlanner.Leg l : legs) {
            double dh = normalizeDeg(l.to.headingDeg - p.headingDeg);
            for (int i = 0; i <= 10; i++) {
                double t = i / 10.0;
                out.add(new Pose(p.x + t * (l.to.x - p.x), p.y + t * (l.to.y - p.y), p.headingDeg + t * dh));
            }
            p = l.to;
        }
        return out;
    }
}

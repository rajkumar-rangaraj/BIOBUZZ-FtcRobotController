package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoField.*;
import static org.firstinspires.ftc.teamcode.auto.HiveAutoField.*;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.fieldtour.FieldMap;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class HivePlannerTest {

    /** Pre-load volley after reaching the spot: aim 1.5 + settle 0.5 + 4 balls (9 Nov logs: 2.3-4.8 s, median ~3.2). */
    private static final double NEAR_FIRE_TYPICAL = 1.5 + 0.5 + 3.3;
    private static final double NEAR_FIRE_SLOW = 1.5 + 0.5 + 5.0;
    private static final double GARDEN_DWELL = BioBuzzHiveAutoBase.GARDEN_DWELL_SEC;
    private static final double FAR_AIM_SETTLE = BioBuzzHiveAutoBase.FAR_AIM_SEC + 0.5;

    private static final Pose GARDEN_END = AutoPlanner.end(SHOT, HivePlanner.toGarden());

    /** The leg model reproduces the CAD slices of am-5876 (A-Frame Leg, RED side) at 4, 8 and 14 in. */
    @Test
    public void legModelMatchesTheCad() {
        double[][] cad = {{4, -22.28, 16.47, 17.56}, {8, -21.19, 14.69, 15.78}, {14, -19.55, 12.02, 13.11}};
        for (double[] c : cad) {
            assertEquals("inner x at " + c[0], c[1], LEG_X0 + LEG_LEAN * c[0], 0.05);
            assertEquals("|y| lo at " + c[0], c[2], LEG_Y_LO0 - LEG_Y_SLOPE * c[0], 0.05);
            assertEquals("|y| hi at " + c[0], c[3], LEG_Y_HI0 - LEG_Y_SLOPE * c[0], 0.05);
        }
        assertTrue("the HIVE hangs well above the robot", HIVE_LOWEST_IN - ROBOT_TOP_IN > 15);
    }

    @Test
    public void laneUnderTheHiveIsClearOnOurSide() {
        for (double y = -HIVE_CLEAR_Y; y <= HIVE_CLEAR_Y; y += 0.5) {
            Pose p = new Pose(HIVE_LANE_X, y, FACE_REAR);
            assertNull("lane at y " + y, HivePlanner.footprintProblem(p, null, false));
            assertNotNull("BioBuzz AUTO's solid frame box would block it", y > -20 && y < 20
                    ? AutoPlanner.footprintProblem(p, null) : "ok");
        }
        double lowGap = (HIVE_LANE_X - HALF_WIDTH_IN) - LEG_LOW_X_MAX;
        double highGap = (HIVE_LANE_X - ROBOT_UPPER_HALF_WIDTH_IN) - LEG_HIGH_X_MAX;
        double middle = -(HIVE_LANE_X + HALF_WIDTH_IN);
        System.out.printf("lane x %.1f: low body %.2f in from the legs, upper body %.2f in, %.2f in from the middle%n",
                HIVE_LANE_X, lowGap, highGap, middle);
        assertTrue(lowGap >= HivePlanner.FRAME_MARGIN_IN && highGap >= HivePlanner.FRAME_MARGIN_IN);
        assertTrue(middle >= AutoPlanner.OWN_SIDE_MARGIN_IN);
        // 2 in toward the legs is caught, so is the robot turned sideways under the HIVE
        assertNotNull(HivePlanner.footprintProblem(new Pose(HIVE_LANE_X - 2.5, 12, FACE_REAR), null, false));
        assertNotNull(HivePlanner.footprintProblem(new Pose(-14, 0, 45), null, false));
    }

    @Test
    public void farShotSpotIsLegalAndAimedAtTheFarCell() {
        assertNull(HivePlanner.footprintProblem(FAR_SHOT, null, false));
        assertNull(AutoPlanner.footprintProblem(FAR_SHOT, null));
        double d = AutoPlanner.distanceToRobot(FAR_SHOT, FieldMap.Place.FLOWER_1.x, FieldMap.Place.FLOWER_1.y);
        System.out.printf("far spot %s: %.2f in from FLOWER 1's centre (circle %.1f + margin %.1f)%n", FAR_SHOT, d,
                FLOWER_RADIUS_IN, AutoPlanner.FLOWER_MARGIN_IN);
        // straight back down the lane: the shot line passes within 3 in of the far CELL's centre (opening 20 in wide)
        assertEquals(FACE_REAR, FAR_SHOT.headingDeg, 1e-9);
        assertTrue(Math.abs(FAR_SHOT.x - FAR_CELL_X) <= 3.0);
        assertTrue("about as far as the near shot", FAR_SHOT.distanceTo(new Pose(FAR_CELL_X, FAR_CELL_Y, 0)) > 40);
        // the plain mirror of the near spot would hit FLOWER 1
        assertNotNull(HivePlanner.footprintProblem(new Pose(CELL_X, -SHOT.y, FACE_REAR), null, false));
    }

    @Test
    public void everyRouteIsClear() {
        for (Start s : Start.values()) {
            assertNull(s.name(), HivePlanner.routeProblem(start(s), HivePlanner.toShot(s), null));
        }
        assertNull(HivePlanner.routeProblem(SHOT, HivePlanner.toGarden(), null));
        assertNull(HivePlanner.routeProblem(GARDEN_END, HivePlanner.gardenWiggle(), null));
        Pose wiggleOut = HivePlanner.gardenWiggle().get(0).to;
        for (Pose from : new Pose[]{GARDEN_END, wiggleOut, SHOT}) {
            assertNull("under the HIVE from " + from, HivePlanner.routeProblem(from, HivePlanner.underHive(from), null));
            assertEquals(FAR_SHOT.x, AutoPlanner.end(from, HivePlanner.underHive(from)).x, 1e-9);
        }
        assertNull(HivePlanner.routeProblem(GARDEN_END, HivePlanner.gardenToNearShot(), null));
        assertNull(HivePlanner.routeProblem(FAR_SHOT, HivePlanner.toFlower(), null));
        List<AutoPlanner.Leg> flowerIn = new ArrayList<>();
        flowerIn.add(HivePlanner.flowerCreep());
        flowerIn.add(HivePlanner.flowerBackOut());
        assertNull(HivePlanner.routeProblem(FLOWER_APPROACH, flowerIn, null));
        assertNull(HivePlanner.routeProblem(FLOWER_APPROACH, HivePlanner.flowerToFarShot(), null));
        // the FLOWER legs touch FLOWER 1 on purpose, nothing else does
        assertNotNull(HivePlanner.footprintProblem(FLOWER_STOP, null, false));
    }

    @Test
    public void underTheHiveTheIntakeIsOffAndTheHeadingConstant() {
        List<AutoPlanner.Leg> legs = HivePlanner.underHive(GARDEN_END);
        for (AutoPlanner.Leg l : legs) {
            assertEquals(l.label, AutoPlanner.Action.DRIVE, l.action);
        }
        Pose entry = legs.get(0).to;
        assertEquals(-HIVE_CLEAR_Y, entry.y, 1e-9);
        assertEquals(FACE_REAR, entry.headingDeg, 1e-9);
        assertTrue("turned before the legs", Math.abs(FAR_SHOT.headingDeg - FACE_REAR) < 5);
    }

    @Test
    public void flowerStopIsCentredOnFlower1() {
        assertEquals(FieldMap.Place.FLOWER_1.x, FLOWER_STOP.x, 0.01);
        assertEquals(FACE_REAR, FLOWER_STOP.headingDeg, 1e-9);
        assertTrue("approach short of the ring", FLOWER_APPROACH.y + HALF_LENGTH_IN < FLOWER_RING_FRONT_Y - 3);
        assertTrue(HivePlanner.flowerCreep().action == AutoPlanner.Action.INTAKE);
    }

    /** Parking must work from wherever a step is cut short, without touching the partner in the other slot. */
    @Test
    public void parkIsClearFromAnywhere() {
        List<Pose> from = new ArrayList<>();
        for (Start s : Start.values()) {
            from.addAll(samples(start(s), HivePlanner.toShot(s)));
        }
        from.addAll(samples(SHOT, HivePlanner.toGarden()));
        from.addAll(samples(GARDEN_END, HivePlanner.gardenWiggle()));
        from.addAll(samples(GARDEN_END, HivePlanner.underHive(GARDEN_END)));
        from.addAll(samples(SHOT, HivePlanner.underHive(SHOT)));
        from.addAll(samples(GARDEN_END, HivePlanner.gardenToNearShot()));
        from.addAll(samples(FAR_SHOT, HivePlanner.toFlower()));
        List<AutoPlanner.Leg> flowerIn = new ArrayList<>();
        flowerIn.add(HivePlanner.flowerCreep());
        from.addAll(samples(FLOWER_APPROACH, flowerIn));
        from.addAll(samples(FLOWER_APPROACH, HivePlanner.flowerToFarShot()));
        assertTrue(from.size() > 150);
        for (Slot slot : Slot.values()) {
            for (Pose p : from) {
                List<AutoPlanner.Leg> legs = HivePlanner.park(p, slot);
                assertNull("park " + slot + " from " + p, HivePlanner.routeProblem(p, legs, slot.other()));
                Pose end = AutoPlanner.end(p, legs);
                assertEquals(slot(slot).x, end.x, 1e-9);
                assertEquals(slot(slot).y, end.y, 1e-9);
                assertTrue(AutoPlanner.partlyInZone(end));
                for (Pose q : samples(p, legs)) {               // a blocked park re-plans from where it stopped
                    assertNull("re-plan " + slot + " from " + q,
                            HivePlanner.routeProblem(q, HivePlanner.park(q, slot), slot.other()));
                }
            }
        }
    }

    @Test
    public void parkFromUnderTheHiveLeavesAlongTheLaneFirst() {
        for (double y = -28; y <= 28; y += 4) {
            Pose p = new Pose(HIVE_LANE_X, y, FACE_REAR);
            assertTrue(HivePlanner.underTheHive(p));
            AutoPlanner.Leg first = HivePlanner.park(p, Slot.A).get(0);
            assertEquals("OUT FROM UNDER", first.label);
            assertEquals(HIVE_LANE_X, first.to.x, 1e-9);
            assertEquals(HIVE_CLEAR_Y, Math.abs(first.to.y), 1e-9);
        }
    }

    @Test
    public void parkFromTheFarSpotIsShortAndStraightIntoSlotA() {
        List<AutoPlanner.Leg> legs = HivePlanner.park(FAR_SHOT, Slot.A);
        double sec = AutoPlanner.estimateSec(FAR_SHOT, legs);
        System.out.printf("park A from the far spot: %s, %.1f s; park B: %.1f s%n", legs, sec,
                AutoPlanner.estimateSec(FAR_SHOT, HivePlanner.park(FAR_SHOT, Slot.B)));
        assertTrue(sec < 3.5);
        assertEquals(FACE_REAR, AutoPlanner.end(FAR_SHOT, legs).headingDeg, 1e-9);
    }

    /**
     * The whole plan against the clock (time model fitted to the 9 Nov runs). Prints the timeline; the park must
     * always finish, and with slot A (the HIVE AUTO default) the far CELL gets ~4 balls of feed time in the typical case.
     * Dwell: "auto" = the op mode's (up to 2 s, shortened for the far CELL); 4 s = BioBuzz AUTO's corner time.
     */
    @Test
    public void timelineFitsWithParkingFirst() {
        double farWork = BioBuzzHiveAutoBase.FAR_AIM_SEC + 0.5 + BioBuzzHiveAutoBase.FAR_FEED_TARGET_SEC;
        for (Slot slot : Slot.values()) {
            for (double nearFire : new double[]{NEAR_FIRE_TYPICAL, NEAR_FIRE_SLOW}) {
                for (boolean autoDwell : new boolean[]{true, false}) {
                    StringBuilder line = new StringBuilder();
                    double parkBy = BioBuzzHiveAutoBase.PARK_BY_DEFAULT_SEC;
                    double t = AutoPlanner.estimateSec(START_CELL_WALL, HivePlanner.toShot(Start.CELL_WALL)) + nearFire;
                    line.append(String.format("slot %s near %.1f s dwell %-4s | pre-loads done %5.1f", slot,
                            nearFire, autoDwell ? "auto" : "4 s", t));
                    assertTrue(HivePlanner.fits(t, SHOT, HivePlanner.toGarden(), BioBuzzHiveAutoBase.GARDEN_MIN_DWELL_SEC,
                            slot, parkBy));
                    t += AutoPlanner.estimateSec(SHOT, HivePlanner.toGarden());
                    double atCorner = t;
                    t = autoDwell ? HivePlanner.gardenDwellEnd(t, GARDEN_END, slot, parkBy,
                            BioBuzzHiveAutoBase.GARDEN_MIN_DWELL_SEC, GARDEN_DWELL, farWork)
                            : Math.min(t + 4.0, HivePlanner.parkStartSec(GARDEN_END, slot, parkBy));
                    line.append(String.format(" | GARDEN done %5.1f (%.1f s at the corner)", t, t - atCorner));
                    List<AutoPlanner.Leg> under = HivePlanner.underHive(GARDEN_END);
                    boolean far = HivePlanner.fits(t, GARDEN_END, under, BioBuzzHiveAutoBase.FAR_MIN_FIRE_SEC, slot, parkBy);
                    Pose at = GARDEN_END;
                    if (far) {
                        t += AutoPlanner.estimateSec(GARDEN_END, under);
                        at = FAR_SHOT;
                        double fireUntil = HivePlanner.parkStartSec(at, slot, parkBy);
                        double feedSec = Math.min(fireUntil - t - FAR_AIM_SETTLE, BioBuzzHiveAutoBase.FAR_FEED_TARGET_SEC);
                        line.append(String.format(" | far spot %5.1f, feed %4.1f s", t, feedSec));
                        t = Math.min(t + FAR_AIM_SETTLE + BioBuzzHiveAutoBase.FAR_FEED_TARGET_SEC, fireUntil);
                        if (nearFire == NEAR_FIRE_TYPICAL && autoDwell && slot == Slot.A) {
                            assertTrue("typical: ~4 balls at the far CELL, got " + feedSec + " s", feedSec >= 3.0);
                        }
                    } else {
                        line.append(" | far CELL: NO TIME");
                    }
                    double parkStart = Math.min(t, HivePlanner.parkStartSec(at, slot, parkBy));
                    double parkSec = AutoPlanner.estimateSec(at, HivePlanner.park(at, slot));
                    line.append(String.format(" | park %5.1f + %.1f = %5.1f", parkStart, parkSec, parkStart + parkSec));
                    System.out.println(line);
                    assertTrue(line.toString(), parkStart + parkSec <= AutoPlanner.MATCH_SEC - AutoPlanner.PARK_MARGIN_SEC + 1e-9);
                    if (autoDwell && (slot == Slot.A || nearFire == NEAR_FIRE_TYPICAL)) {
                        assertTrue("reaches the far CELL: " + line, far);
                    }
                }
            }
        }
    }

    @Test
    public void gardenDwellShrinksForTheFarCellButKeepsOneBackAndForth() {
        double farWork = BioBuzzHiveAutoBase.FAR_AIM_SEC + 0.5 + BioBuzzHiveAutoBase.FAR_FEED_TARGET_SEC;
        double early = HivePlanner.gardenDwellEnd(10, GARDEN_END, Slot.A, 27, 1.0, 2.0, farWork);
        assertEquals("plenty of time: the full 2 s", 12.0, early, 1e-9);
        double late = HivePlanner.gardenDwellEnd(15, GARDEN_END, Slot.A, 27, 1.0, 2.0, farWork);
        assertEquals("late: at least 1 s", 16.0, late, 1e-9);
        double mid = HivePlanner.gardenDwellEnd(12.8, GARDEN_END, Slot.A, 27, 1.0, 2.0, farWork);
        assertTrue("in between: what the far CELL leaves, got " + mid, mid > 13.8 && mid < 14.8);
    }

    @Test
    public void blueIsTheMirror() {
        Pose blue = FAR_SHOT.forAlliance(Alliance.BLUE);
        assertTrue("BLUE far spot on the BLUE side", blue.x - HALF_WIDTH_IN > 0);
        assertEquals(-FAR_SHOT.y, blue.y, 1e-9);
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

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

    /**
     * Pre-load volley after reaching the spot (lit field: the camera sees the CELL): aim ~0.8 s + settle 0.5 + 4 balls
     * (9 Nov logs: 2.3-4.8 s, median ~3.2) + the TIP watch (~1 s while it sees our HIVE). Slow: aim cap 1.5 s, slow feed,
     * full watch.
     */
    private static final double NEAR_FIRE_TYPICAL = 0.8 + 0.5 + 3.3 + 1.0;
    private static final double NEAR_FIRE_SLOW = 1.5 + 0.5 + 5.0 + 1.2;
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
        assertNull(HivePlanner.footprintProblem(FAR_AIM, null, false));
        // as built (robot not centred on Pedro's pose): intake ~2 in off the REAR wall, FLOWER 1's ring 1+ in away
        double h = Math.toRadians(FAR_AIM.headingDeg);
        double intakeY = FAR_AIM.y + START_OFFSET_IN + ROBOT_FRONT_IN * Math.sin(h);
        double[] arrive = clearances(FAR_SHOT), aimed = clearances(FAR_AIM);
        System.out.printf("far spot %s, aimed %s: intake %.2f in off the REAR wall; corners arriving / aimed: wall %.2f /"
                + " %.2f, middle line %.2f / %.2f, FLOWER 1 %.2f / %.2f in%n", FAR_SHOT, FAR_AIM, WALL_IN - intakeY,
                arrive[0], aimed[0], arrive[1], aimed[1], arrive[2], aimed[2]);
        assertTrue(WALL_IN - intakeY >= 1.5 && WALL_IN - intakeY <= 3.0);
        assertTrue(arrive[2] >= 1.0 && aimed[2] >= 1.0);
        // the camera (on the centre line, 4.63 in behind the centre) looks straight at the far CELL's opening
        double camX = FAR_AIM.x - CAMERA_BACK_IN * Math.cos(h);
        double camY = FAR_AIM.y + START_OFFSET_IN - CAMERA_BACK_IN * Math.sin(h);
        double toCell = Math.toDegrees(Math.atan2(FAR_CELL_Y - camY, FAR_CELL_X - camX));
        assertEquals(0, normalizeDeg(toCell - (FAR_AIM.headingDeg + 180)), 0.01);
        assertTrue("a small turn on the spot", Math.abs(FAR_AIM_HEADING - FACE_REAR) < 4);
        assertTrue(Math.abs(FAR_SHOT.x - FAR_CELL_X) <= 3.0);
        // the plain mirror of the near spot (x -12.7) would hit FLOWER 1
        assertNotNull(HivePlanner.footprintProblem(new Pose(CELL_X, -SHOT.y, FACE_REAR), null, false));
    }

    /**
     * The far spot as built (START_OFFSET_IN; 9.76 / 8.03 / 8.9 in), with 1 deg of Pedro heading error: arriving down
     * the lane (heading 90), turning on the spot to FAR_AIM_HEADING and the whole camera-aim range around it
     * (+- FAR_AIM_MAX_TURN_DEG), the robot stays 0.4 in off the REAR wall, our side's line (x 0) and FLOWER 1's CAD box.
     */
    @Test
    public void farSpotAimRangeIsClearAsBuilt() {
        double lo = Double.NaN, hi = Double.NaN;
        for (double hd = 70; hd <= 110; hd += 0.05) {
            if (clear(new Pose(FAR_SHOT.x, FAR_SHOT.y, hd), 0.4)) {
                lo = Double.isNaN(lo) ? hd : lo;
                hi = hd;
            }
        }
        System.out.printf("far spot: 0.4 in clear as built from %.2f to %.2f deg; arrive 90, aim %.2f +- %.1f (+1 Pedro)%n",
                lo, hi, FAR_AIM_HEADING, FAR_AIM_MAX_TURN_DEG);
        double from = Math.min(FACE_REAR, FAR_AIM_HEADING - FAR_AIM_MAX_TURN_DEG) - 1.0;
        double to = Math.max(FACE_REAR, FAR_AIM_HEADING + FAR_AIM_MAX_TURN_DEG) + 1.0;
        for (double hd = from; hd <= to + 1e-9; hd += 0.1) {
            Pose p = new Pose(FAR_SHOT.x, FAR_SHOT.y, hd);
            assertTrue("heading " + hd + ": " + java.util.Arrays.toString(clearances(p)), clear(p, 0.4));
        }
        List<AutoPlanner.Leg> legs = HivePlanner.underHive(GARDEN_END);
        AutoPlanner.Leg under = legs.get(legs.size() - 1);
        assertEquals("straight down the lane to the far spot", FAR_SHOT, under.to);
        assertEquals(FACE_REAR, under.to.headingDeg, 1e-9);
    }

    /**
     * Both tag stickers face the end that is up, so the far spot looks at the far CELL's sticker the way the near spot
     * looks at the near one (seen there on 9 Nov): the camera must be as far from it, or the sticker leaves the top of
     * the picture (it was 10-15 deg above the camera axis there; the C270's picture ends at 16.3 deg).
     */
    @Test
    public void farSpotCameraSeesTheFarCellLikeTheNearSpot() {
        double nearCamY = SHOT.y + START_OFFSET_IN + CAMERA_BACK_IN;          // facing the audience: camera toward +y
        double near = Math.abs(nearCamY - (-TAG_UP_Y));
        double farCamY = FAR_SHOT.y + START_OFFSET_IN - CAMERA_BACK_IN;       // facing the REAR: camera toward -y
        double far = Math.hypot(FAR_SHOT.x - CELL_X, farCamY - TAG_UP_Y);
        System.out.printf("camera to the UP sticker: near spot %.2f in, far spot %.2f in%n", near, far);
        assertEquals(near, far, 0.6);
    }

    /** The FLOWER footprint is FLOWER 1's CAD slice (rings and pipes at robot heights): x -26.37..-20.42, y 65.63..70.5. */
    @Test
    public void flowerFootprintIsTheCadSlice() {
        assertEquals(-20.42, FLOWER_X + FLOWER_HALF_WIDTH_IN, 0.05);
        assertEquals(-26.37, FLOWER_X - FLOWER_HALF_WIDTH_IN, 0.05);
        assertEquals(65.63, WALL_IN - FLOWER_DEPTH_IN, 0.01);
        double m = AutoPlanner.FLOWER_MARGIN_IN;
        FieldMap.Place f1 = FieldMap.Place.FLOWER_1, f3 = FieldMap.Place.FLOWER_3;
        // a robot 0.1 in outside the margin is clear, 0.1 in inside is not (FLOWER 1, rear wall; FLOWER 3, RED wall)
        assertTrue(!HivePlanner.hitsFlower(AutoPlanner.corners(new Pose(-20.42 + m + 9.1, 60, 90)), f1));
        assertTrue(HivePlanner.hitsFlower(AutoPlanner.corners(new Pose(-20.42 + m + 8.9, 60, 90)), f1));
        assertTrue(!HivePlanner.hitsFlower(AutoPlanner.corners(new Pose(-60, f3.y + FLOWER_HALF_WIDTH_IN + m + 9.1, 0)), f3));
        assertTrue(HivePlanner.hitsFlower(AutoPlanner.corners(new Pose(-60, f3.y + FLOWER_HALF_WIDTH_IN + m + 8.9, 0)), f3));
    }

    /** Parked facing the REAR wall the robot's short back is toward the LOADING ZONE: still well inside it. */
    @Test
    public void rearSlotAIsWellInsideTheZoneAsBuilt() {
        Pose a = HivePlanner.rearSlot(Slot.A);
        double back = a.y + START_OFFSET_IN - ROBOT_BACK_IN;
        System.out.printf("rear slot A %s: %.2f in inside the LOADING ZONE as built%n", a, ZONE_Y_MAX - back);
        assertTrue(ZONE_Y_MAX - back >= 4.0);
        assertTrue(AutoPlanner.partlyInZone(a));
        assertNull(HivePlanner.footprintProblem(a, Slot.B, false));
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
        for (AutoPlanner.Leg l : HivePlanner.gardenToNearShot()) {
            assertEquals("plan B, G407: " + l.label, AutoPlanner.Action.DRIVE, l.action);
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
                assertTrue("slot " + slot + " row, from " + p, Math.abs(end.y - slot(slot).y) < 1e-9
                        || (slot == Slot.A && Math.abs(end.y - REAR_SLOT_A_Y) < 1e-9));
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

    @Test
    public void farAimHeadingLimitWorksTheShortWayRound() {
        double c = FAR_AIM_HEADING, m = FAR_AIM_MAX_TURN_DEG, lo = c - m, hi = c + m;
        assertEquals(c + 1.0, HivePlanner.limitHeadingDeg(c + 1.0, lo, hi), 1e-12);
        assertEquals(hi, HivePlanner.limitHeadingDeg(c + 7.0, lo, hi), 1e-9);
        assertEquals(lo, HivePlanner.limitHeadingDeg(c - 7.0, lo, hi), 1e-9);
        assertEquals(lo, HivePlanner.limitHeadingDeg(c - 7.0 + 360, lo, hi), 1e-9);   // Pedro may report 0..360
        Pose blue = FAR_AIM.forAlliance(Alliance.BLUE);
        double b = blue.headingDeg;
        assertEquals(b + m, HivePlanner.limitHeadingDeg(b + 20 - 360, b - m, b + m), 1e-9);
        assertEquals(179.0, HivePlanner.limitHeadingDeg(-179.0, 177.0, 179.0), 1e-9);   // across +-180
    }

    /** The op mode's as-built clearances (coarser, so cheap on the robot) agree with this test's fine sampling. */
    @Test
    public void asBuiltClearancesMatchAFineSampling() {
        for (double dx = -1; dx <= 1 + 1e-9; dx += 0.5) {
            for (double hd = 78; hd <= 102 + 1e-9; hd += 0.25) {
                Pose p = new Pose(FAR_SHOT.x + dx, FAR_SHOT.y, hd);
                double[] fast = HivePlanner.asBuiltClearances(p), fine = clearances(p);
                for (int i = 0; i < 3; i++) {
                    assertEquals(p + " #" + i, fine[i], fast[i], 0.02);
                }
            }
        }
    }

    /**
     * The far aim turns only as far as is clear where Pedro really stopped: from the planned spot the whole
     * FAR_AIM_HEADING +- FAR_AIM_MAX_TURN_DEG; stopped up to 0.75 in sideways, 1 in short or long and 1 deg turned, it
     * can always turn, every heading on the way and +- 1 deg around the limits keeps the as-built robot 0.4 in clear (or,
     * from something it is already closer to, never closer than it was), and up to 0.5 in off it aims within 0.5 deg of
     * the plan; right next to the middle line it does not turn at all.
     */
    @Test
    public void farAimLimitsFollowWherePedroStops() {
        double[] nominal = HivePlanner.farAimLimits(FAR_SHOT);
        assertNotNull(nominal);
        assertEquals(FAR_AIM_HEADING - FAR_AIM_MAX_TURN_DEG, nominal[0], 1e-9);
        assertEquals(FAR_AIM_HEADING + FAR_AIM_MAX_TURN_DEG, nominal[1], 1e-9);
        double worst = 0;
        for (double dx = -0.75; dx <= 0.75 + 1e-9; dx += 0.25) {
            for (double dy = -1; dy <= 1 + 1e-9; dy += 0.5) {
                for (double dh = -1; dh <= 1 + 1e-9; dh += 0.5) {
                    Pose at = new Pose(FAR_SHOT.x + dx, FAR_SHOT.y + dy, FACE_REAR + dh);
                    double[] lim = HivePlanner.farAimLimits(at);
                    assertNotNull("stopped at " + at, lim);
                    double start = HivePlanner.limitHeadingDeg(FAR_AIM_HEADING, lim[0], lim[1]);
                    worst = Math.max(worst, Math.abs(start - FAR_AIM_HEADING));
                    if (Math.abs(dx) <= 0.5 + 1e-9 && Math.abs(dy) <= 0.5 + 1e-9) {
                        assertEquals("aims near the plan from " + at, FAR_AIM_HEADING, start, 0.5);
                    }
                    double[] was = clearances(at);
                    double from = Math.min(at.headingDeg, lim[0] - FAR_AIM_HEADING_ERR_DEG);
                    double to = Math.max(at.headingDeg, lim[1] + FAR_AIM_HEADING_ERR_DEG);
                    for (double hd = from; hd <= to + 1e-9; hd += 0.1) {
                        double[] c = clearances(new Pose(at.x, at.y, hd));
                        for (int i = 0; i < 3; i++) {
                            assertTrue("from " + at + " at " + hd + " #" + i + ": " + c[i],
                                    c[i] >= Math.min(FAR_AIM_CLEAR_IN, was[i]) - 0.02);
                        }
                    }
                }
            }
        }
        System.out.printf("far aim limits: every stop pose can turn; first aim at most %.2f deg off the plan%n", worst);
        assertNull("1.2 in toward the middle line: no turn", HivePlanner.farAimLimits(
                new Pose(FAR_SHOT.x + 1.2, FAR_SHOT.y, FACE_REAR)));
        assertNull("touching the middle line: no turn", HivePlanner.farAimLimits(
                new Pose(FAR_SHOT.x + 1.45, FAR_SHOT.y, FACE_REAR)));
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

    /** The as-built robot's corners at a Pedro pose (RED frame): really START_OFFSET_IN further toward the REAR wall. */
    private static double[][] asBuiltCorners(Pose p) {
        double h = Math.toRadians(p.headingDeg), c = Math.cos(h), s = Math.sin(h);
        double cx = p.x, cy = p.y + START_OFFSET_IN;
        double[][] local = {{ROBOT_FRONT_IN, ROBOT_HALF_WIDTH_IN}, {ROBOT_FRONT_IN, -ROBOT_HALF_WIDTH_IN},
                {-ROBOT_BACK_IN, -ROBOT_HALF_WIDTH_IN}, {-ROBOT_BACK_IN, ROBOT_HALF_WIDTH_IN}};
        double[][] out = new double[4][];
        for (int i = 0; i < 4; i++) {
            double f = local[i][0], l = local[i][1];
            out[i] = new double[]{cx + f * c - l * s, cy + f * s + l * c};
        }
        return out;
    }

    /** As-built clearances (in) at a Pedro pose: {REAR wall, our side's line x 0, FLOWER 1's CAD box}. Fine sampling:
     *  the reference for HivePlanner.asBuiltClearances. */
    private static double[] clearances(Pose p) {
        double[][] k = asBuiltCorners(p);
        double fx0 = FLOWER_X - FLOWER_HALF_WIDTH_IN, fx1 = FLOWER_X + FLOWER_HALF_WIDTH_IN;
        double fy0 = WALL_IN - FLOWER_DEPTH_IN, fy1 = WALL_IN;
        double wall = Double.MAX_VALUE, middle = Double.MAX_VALUE, flower = Double.MAX_VALUE;
        for (int i = 0; i < 4; i++) {
            double[] a = k[i], b = k[(i + 1) % 4];
            wall = Math.min(wall, WALL_IN - a[1]);
            middle = Math.min(middle, -a[0]);
            for (int j = 0; j <= 400; j++) {                    // along the robot's side, every ~0.05 in
                double x = a[0] + (b[0] - a[0]) * j / 400.0, y = a[1] + (b[1] - a[1]) * j / 400.0;
                double dx = Math.max(Math.max(fx0 - x, x - fx1), 0), dy = Math.max(Math.max(fy0 - y, y - fy1), 0);
                flower = Math.min(flower, Math.hypot(dx, dy));
            }
        }
        return new double[]{wall, middle, flower};
    }

    private static boolean clear(Pose p, double margin) {
        for (double c : clearances(p)) {
            if (c < margin) {
                return false;
            }
        }
        return true;
    }
}

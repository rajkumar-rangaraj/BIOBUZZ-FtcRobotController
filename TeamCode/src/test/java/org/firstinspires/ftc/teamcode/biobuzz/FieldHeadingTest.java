package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class FieldHeadingTest {
    private static final double DT = 0.05;

    @Test
    public void stickUpIsAwayFromTheDriversWhicheverWayTheRobotFaces() {
        double[] r = MecanumMath.driverToRobot(1, 0, 0);               // intake away: same as robot-centric
        assertEquals(1, r[0], 1e-9);
        assertEquals(0, r[1], 1e-9);
        r = MecanumMath.driverToRobot(1, 0, 90);                       // intake to the left: away = the robot's right
        assertEquals(0, r[0], 1e-9);
        assertEquals(1, r[1], 1e-9);
        r = MecanumMath.driverToRobot(1, 0, -90);                      // intake to the right (AUTO's CELL-wall start)
        assertEquals(0, r[0], 1e-9);
        assertEquals(-1, r[1], 1e-9);
        r = MecanumMath.driverToRobot(1, 0, 180);                      // intake toward the drivers: away = backwards
        assertEquals(-1, r[0], 1e-9);
        assertEquals(0, r[1], 1e-9);
        r = MecanumMath.driverToRobot(0, 1, 90);                       // stick right, intake left: drivers' right = back
        assertEquals(-1, r[0], 1e-9);
        assertEquals(0, r[1], 1e-9);
    }

    @Test
    public void theHeadingStartsWhereAutoLeftItAndFollowsThePinpoint() {
        FieldHeading h = new FieldHeading(-90, 720, 15, 0.3);
        assertEquals(-90, h.update(0, 0, Double.NaN), 1e-9);          // Pinpoint reset at INIT reads 0
        assertEquals(-60, h.update(0.1, 30, Double.NaN), 1e-9);       // turned 30 deg CCW
        assertEquals(FieldHeading.Source.PINPOINT, h.source());
    }

    @Test
    public void aJumpTheRobotCannotMakeIsIgnored() {
        FieldHeading h = new FieldHeading(0, 720, 15, 0.3);
        double t = 0;
        for (; t < 0.5; t += DT) {
            h.update(t, 0, Double.NaN);
        }
        assertEquals(0, h.update(t, 150, Double.NaN), 1e-9);          // +150 deg in 50 ms: the 9 Nov IMU glitch
        assertEquals(1, h.glitches());
        assertTrue(h.event().contains("ignored"));
        assertEquals(2, h.update(t + DT, 2, Double.NaN), 1e-9);       // the next reading is fine again
        assertNull(h.event());
    }

    @Test
    public void aSensorThatRestartsFromZeroCarriesOnFromTheLastGoodHeading() {
        FieldHeading h = new FieldHeading(0, 720, 15, 0.3);
        double t = 0;
        for (double raw = 0; raw <= 120; raw += 20, t += DT) {
            h.update(t, raw, Double.NaN);                              // the robot turns to 120 deg (400 deg/s)
        }
        assertEquals(120, h.heading(), 1e-9);
        for (double end = t + 0.5; t < end; t += DT) {
            h.update(t, 0, Double.NaN);                                // the Pinpoint restarted: reads 0
        }
        assertEquals("still the robot's real heading", 120, h.heading(), 1e-9);
        assertEquals(130, h.update(t, 10, Double.NaN), 1e-9);         // and it follows the restarted sensor
        assertEquals(1, h.glitches());
    }

    @Test
    public void theImuTakesOverWithoutAJumpWhenThePinpointDrops() {
        FieldHeading h = new FieldHeading(0, 720, 15, 0.3);
        h.update(0, 0, 50);                                            // the IMU reads 50 when the Pinpoint reads 0
        h.update(0.05, 20, 70);
        assertEquals(20, h.heading(), 1e-9);
        assertEquals(30, h.update(0.10, Double.NaN, 80), 1e-9);       // Pinpoint gone: the IMU carries on
        assertEquals(FieldHeading.Source.IMU, h.source());
        assertTrue(h.event().contains("IMU"));
        assertTrue(Double.isNaN(h.update(0.15, Double.NaN, Double.NaN)));   // no reading: drive robot-centric
        assertEquals("the last heading is kept for when a sensor returns", 30, h.heading(), 1e-9);
        assertEquals(FieldHeading.Source.NONE, h.source());
    }

    @Test
    public void resetMakesTheCurrentHeadingAway() {
        FieldHeading h = new FieldHeading(0, 720, 15, 0.3);
        h.update(0, 0, Double.NaN);
        h.update(0.1, 45, Double.NaN);
        h.reset();
        assertEquals(0, h.heading(), 1e-9);
        assertEquals(10, h.update(0.2, 55, Double.NaN), 1e-9);
        FieldHeading before = new FieldHeading(-90, 720, 15, 0.3);     // reset during INIT: the start heading is 0
        before.reset();
        assertEquals(0, before.update(0, 0, Double.NaN), 1e-9);
    }

    @Test
    public void aFastRealTurnIsNotAGlitch() {
        FieldHeading h = new FieldHeading(0, 720, 15, 0.3);
        double t = 0, raw = 0;
        for (int i = 0; i < 12; i++, t += DT, raw += 30) {             // 600 deg/s: a full-stick spin
            h.update(t, MecanumMath.normalizeDegrees(raw), Double.NaN);
        }
        assertEquals(0, h.glitches());
        assertEquals(MecanumMath.normalizeDegrees(raw - 30), h.heading(), 1e-9);
    }

    @Test
    public void aPinpointThatComesBackRestartedAfterAShortGapRejoinsWithoutAJump() {
        // review 10 Nov: Pinpoint 120 deg, gone 0.25 s, back restarted at 0: the old rate check over the gap let it in
        FieldHeading h = new FieldHeading(0, 720, 15, 0.3);
        double t = 0;
        for (double raw = 0; raw <= 120; raw += 20, t += DT) {
            h.update(t, raw, Double.NaN);
        }
        for (double end = t + 0.25; t < end; t += DT) {
            assertTrue(Double.isNaN(h.update(t, Double.NaN, Double.NaN)));
        }
        assertEquals("rejoins at the last good heading", 120, h.update(t, 0, Double.NaN), 1e-9);
        assertEquals(125, h.update(t + DT, 5, Double.NaN), 1e-9);
        // the same with the IMU carrying the heading through the gap (the robot turns 10 deg meanwhile)
        FieldHeading k = new FieldHeading(0, 720, 15, 0.3);
        t = 0;
        k.update(t, 120, 40);
        t += DT;
        k.update(t, Double.NaN, 45);                                  // Pinpoint gone, IMU +5
        t += DT;
        k.update(t, Double.NaN, 50);                                  // +5 more
        t += DT;
        assertEquals("the Pinpoint rejoins where the IMU had got to", 10, k.update(t, 0, 50), 1e-9);
        assertEquals(FieldHeading.Source.PINPOINT, k.source());
    }

    @Test
    public void theImuCarriesTheHeadingWhileThePinpointRestartsDuringATurn() {
        // review 10 Nov: a rejected Pinpoint reading used to freeze the heading and lose the turn made meanwhile
        FieldHeading h = new FieldHeading(0, 720, 15, 0.3);
        double t = 0, turned = 0;
        h.update(t, 100, 20);
        for (int i = 0; i < 12; i++) {                                 // 0.6 s turning 200 deg/s; the Pinpoint reads 0
            t += DT;
            turned += 10;
            h.update(t, i < 2 ? 100 + turned : 0 + (turned - 30), 20 + turned);   // restarted after 2 loops
        }
        assertEquals("the turn made during the Pinpoint restart is kept", turned, h.heading(), 1e-9);
        t += DT;
        turned += 10;
        h.update(t, turned - 30, 20 + turned);
        assertEquals(FieldHeading.Source.PINPOINT, h.source());
        assertEquals(turned, h.heading(), 1e-9);
    }

    @Test
    public void aHardSpinSeenByBothSensorsIsFollowed() {
        FieldHeading h = new FieldHeading(0, 720, 15, 0.3);
        h.update(0, 0, 30);
        h.update(0.05, 0, 30);
        assertEquals("150 deg in 50 ms, but both sensors saw it", 150, h.update(0.10, 150, 180), 1e-9);
        assertTrue(h.event().contains("both"));
        assertEquals(155, h.update(0.15, 155, 185), 1e-9);
    }

    @Test
    public void headingsWrapAround() {
        FieldHeading h = new FieldHeading(170, 720, 15, 0.3);
        h.update(0, 0, Double.NaN);
        assertEquals(-175, h.update(0.05, 15, Double.NaN), 1e-9);     // 170 + 15 = 185 = -175
        assertEquals(0, h.glitches());
    }
}

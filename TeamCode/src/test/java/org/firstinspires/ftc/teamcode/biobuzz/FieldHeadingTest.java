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
    public void aResetDuringAGlitchDoesNotJump() {
        // review 10 Nov: reset based on a stale accepted reading undid itself on the next loop
        FieldHeading h = FieldHeading.standard(0);
        h.update(0, 40, Double.NaN);
        h.update(0.02, 0, Double.NaN);                                 // 40 -> 0 in 20 ms: rejected
        h.reset();
        assertEquals(0, h.heading(), 1e-9);
        assertEquals("not -40", 0, h.update(0.06, 0, Double.NaN), 1e-9);
        assertEquals("a glitch or a restart? not known yet", 0, h.update(0.10, 5, Double.NaN), 1e-9);
        assertEquals("0.3 s on the new level: a restart", 0, h.update(0.34, 5, Double.NaN), 1e-9);
        assertEquals(5, h.update(0.38, 10, Double.NaN), 1e-9);
    }

    @Test
    public void aResetJustAfterAGlitchDoesNotJumpWhenTheSensorComesBack() {
        // final review 10 Nov: a reset anchored to the rejected 20 made the heading -20 when the Pinpoint read 0 again
        FieldHeading h = FieldHeading.standard(0);
        h.update(0, 0, Double.NaN);
        h.update(0.02, 20, Double.NaN);                                // rejected (15 deg allowed in 20 ms)
        h.reset();
        assertEquals("not -20", 0, h.update(0.07, 0, Double.NaN), 1e-9);
        for (double t = 0.12; t < 0.5; t += 0.05) {
            assertEquals(0, h.update(t, 0, Double.NaN), 1e-9);
        }
        assertEquals(4, h.update(0.55, 4, Double.NaN), 1e-9);
    }

    @Test
    public void aResetWhileTheImuIsOffItsTrackKeepsBothLinedUp() {
        FieldHeading h = FieldHeading.standard(0);
        h.update(0, 30, 0);
        h.update(0.05, 40, 10);
        assertEquals("IMU glitch: the Pinpoint leads", 20, h.update(0.10, 50, 100), 1e-9);
        h.reset();
        assertEquals("the IMU back on its track agrees", 0, h.update(0.15, 50, 20), 1e-9);
        assertEquals(FieldHeading.Source.PINPOINT, h.source());
        assertEquals(5, h.update(0.20, 55, 25), 1e-9);
        assertEquals(1, h.glitches());
    }

    @Test
    public void aPinpointRestartAfterALoopStallDoesNotPassAsATurn() {
        // final review 10 Nov: after a 0.4 s stall the rate check allowed 186 deg, so a Pinpoint restart led
        FieldHeading h = FieldHeading.standard(0);
        h.update(0, 170, 30);
        h.update(0.05, 170, 30);
        assertEquals(0, h.update(0.45, 0, 30), 1e-9);                 // stationary: Pinpoint 170 -> 0, IMU unchanged
        assertEquals(FieldHeading.Source.IMU, h.source());
        FieldHeading k = FieldHeading.standard(0);                    // a restart small enough for the rate check:
        k.update(0, 100, 30);                                         // the IMU is closer to the prediction
        k.update(0.05, 100, 30);
        assertEquals(0, k.update(0.45, 0, 30), 1e-9);
        assertEquals(FieldHeading.Source.IMU, k.source());
        assertTrue(k.event().contains("off the IMU"));
        assertEquals("the restarted Pinpoint stays out", 0, k.update(0.50, 0, 30), 1e-9);
    }

    @Test
    public void aTurnDuringALongLoopStallSeenByBothIsKept() {
        // final review 10 Nov: a sensor silent for 0.5 s used to rejoin at the old heading, losing the turn meanwhile
        FieldHeading h = FieldHeading.standard(0);
        h.update(0, 0, 50);
        h.update(0.05, 0, 50);
        assertEquals(120, h.update(0.65, 120, 170), 1e-9);            // a 0.6 s stall: both turned 120 deg (200 deg/s)
        assertEquals(125, h.update(0.70, 125, 175), 1e-9);
        assertEquals(0, h.glitches());
    }

    @Test
    public void aFrozenImuDoesNotStopTheHeadingWhileTurning() {
        FieldHeading h = FieldHeading.standard(0);
        double t = 0, raw = 0;
        for (int i = 0; i < 25; i++, t += 0.06, raw += 18) {          // 300 deg/s; the IMU freezes after 5 loops
            h.update(t, MecanumMath.normalizeDegrees(raw), i < 5 ? raw : 72);
            assertEquals("loop " + i, MecanumMath.normalizeDegrees(raw), h.heading(), 1e-9);
        }
        assertEquals(FieldHeading.Source.PINPOINT, h.source());
    }

    @Test
    public void anImuGlitchBurstDoesNotMoveTheHeadingWhileThePinpointIsFine() {
        FieldHeading h = FieldHeading.standard(0);
        double[] imu = {10, 10, 10, -48, 99, -58, 10, 10, 10};         // the IMU reads 10 deg off the Pinpoint
        double t = 0;
        for (double v : imu) {
            assertEquals(0, h.update(t, 0, v), 1e-9);
            t += 0.05;
        }
        assertEquals(FieldHeading.Source.PINPOINT, h.source());
        assertEquals(1, h.glitches());
    }

    @Test
    public void anImuGlitchBurstWithoutThePinpointIsIgnored() {
        FieldHeading h = FieldHeading.standard(0);
        double[] imu = {0, 0, 0, -58, 89, -68, 0};
        double t = 0;
        for (int i = 0; i < imu.length; i++, t += 0.05) {
            assertEquals("reading " + i, 0, h.update(t, Double.NaN, imu[i]), 1e-9);
        }
        assertEquals(3, h.update(t, Double.NaN, 3), 1e-9);
        assertEquals(1, h.glitches());
    }

    @Test
    public void anImuYawThatFlips180DegreesAtPlusMinus90IsCorrected() {
        // diag logs 30 Oct - 9 Nov: the IMU yaw stays within +-90 and jumps 180 deg where the robot turns past it
        FieldHeading alone = FieldHeading.standard(0);
        FieldHeading both = FieldHeading.standard(0);
        double t = 0, real = 60;
        for (int i = 0; i < 30; i++, t += 0.06, real += 18) {          // turning CCW 300 deg/s from 60 deg
            double imuYaw = MecanumMath.normalizeDegrees(real);
            if (Math.abs(imuYaw) > 90) {
                imuYaw = MecanumMath.normalizeDegrees(imuYaw + 180);   // what the IMU reports
            }
            alone.update(t, Double.NaN, imuYaw);
            both.update(t, MecanumMath.normalizeDegrees(real + 25), imuYaw);
            assertEquals("IMU alone, loop " + i, MecanumMath.normalizeDegrees(real - 60), alone.heading(), 1e-9);
            assertEquals("both, loop " + i, MecanumMath.normalizeDegrees(real - 60), both.heading(), 1e-9);
        }
        assertEquals(0, alone.glitches());
        assertEquals(0, both.glitches());
        assertEquals(FieldHeading.Source.PINPOINT, both.source());
    }

    @Test
    public void aPinpointJumpOf180IsNotTakenForAFlip() {
        FieldHeading h = FieldHeading.standard(0);
        h.update(0, 0, Double.NaN);
        assertEquals(0, h.update(0.05, 180, Double.NaN), 1e-9);       // only the IMU flips
        assertEquals(1, h.glitches());
    }

    @Test
    public void aPlausiblePinpointJumpWhileTheImuStaysStillIsAGlitch() {
        // review 10 Nov: with the old 720 deg/s + 15 gate a 20 deg Pinpoint error in 20 ms passed and overrode the IMU
        FieldHeading h = FieldHeading.standard(0);
        h.update(0, 20, 50);
        h.update(0.02, 20, 50);
        assertEquals("the IMU leads", 0, h.update(0.04, 0, 50), 1e-9);
        assertEquals(FieldHeading.Source.IMU, h.source());
        assertEquals(1, h.glitches());
    }

    @Test
    public void aHardTurnAcrossPlusMinus180SeenByBothIsFollowed() {
        FieldHeading h = new FieldHeading(0, 720, 15, 0.3);
        h.update(0, 0, 0);
        h.update(0.05, 0, 0);
        assertEquals("+179 and -179 jumps agree", 179, h.update(0.10, 179, -179), 1e-9);
        assertTrue(h.event().contains("both"));
    }

    @Test
    public void theStandardGateLetsTheRobotsFastestTurnThrough() {
        for (double dt : new double[] {0.02, 0.06, 0.15}) {            // normal loops and a slow one
            FieldHeading pinOnly = FieldHeading.standard(0);
            FieldHeading imuOnly = FieldHeading.standard(0);
            double t = 0, raw = 0;
            for (int i = 0; i < 40; i++, t += dt, raw += 300 * dt) {   // 300 deg/s: the robot's fastest turn
                pinOnly.update(t, MecanumMath.normalizeDegrees(raw), Double.NaN);
                imuOnly.update(t, Double.NaN, MecanumMath.normalizeDegrees(raw + 7));
            }
            assertEquals(0, pinOnly.glitches());
            assertEquals(0, imuOnly.glitches());
            assertEquals(MecanumMath.normalizeDegrees(raw - 300 * dt), pinOnly.heading(), 1e-6);
            assertEquals(MecanumMath.normalizeDegrees(raw - 300 * dt), imuOnly.heading(), 1e-6);
        }
    }

    @Test
    public void headingsWrapAround() {
        FieldHeading h = new FieldHeading(170, 720, 15, 0.3);
        h.update(0, 0, Double.NaN);
        assertEquals(-175, h.update(0.05, 15, Double.NaN), 1e-9);     // 170 + 15 = 185 = -175
        assertEquals(0, h.glitches());
    }
}

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
        assertEquals("turning on at 450 deg/s", 45, h.update(0.2, 90, Double.NaN), 1e-9);
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
    public void aTurnDuringALoopStallSeenByBothSensorsIsFollowed() {
        // 130 deg in a 0.45 s stall: more than the rate check allows (it looks back 0.25 s at most), but both saw it
        FieldHeading h = FieldHeading.standard(0);
        h.update(0, 0, 30);
        h.update(0.05, 0, 30);
        assertEquals(130, h.update(0.50, 130, 160), 1e-9);
        assertTrue(h.event().contains("both"));
        assertEquals(135, h.update(0.55, 135, 165), 1e-9);
        assertEquals(0, h.glitches());
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
        assertEquals("0.3 s on the new level: a restart, with the 5 deg it turned meanwhile", 5,
                h.update(0.34, 5, Double.NaN), 1e-9);
        assertEquals(10, h.update(0.38, 10, Double.NaN), 1e-9);
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
    public void theSensorsAgreeAcrossPlusMinus180() {
        FieldHeading h = FieldHeading.standard(175);
        h.update(0, 0, 0);
        assertEquals("179 and -179 are 2 deg apart", 179, h.update(0.05, 4, 6), 1e-9);
        assertEquals(FieldHeading.Source.PINPOINT, h.source());
        assertEquals(0, h.glitches());
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
    public void aFlippedImuReadingAfterALoopStallIsNotTakenAsATurnTheOtherWay() {
        // final review 10 Nov: after a 0.25 s stall both readings fit the rate check; the prediction picks the flip
        FieldHeading h = FieldHeading.standard(0);
        h.update(0, Double.NaN, 0);
        assertEquals(15, h.update(0.05, Double.NaN, 15), 1e-9);
        assertEquals("turned on to +90 (the IMU reads -90)", 90, h.update(0.30, Double.NaN, -90), 1e-9);
        assertEquals(0, h.glitches());
        assertEquals(1, h.imuFlips());
    }

    @Test
    public void aFrozenImuDoesNotHoldTheHeadingWhenATurnStarts() {
        // final review 10 Nov: at the start of a turn the prediction (no turn) matched the frozen IMU
        FieldHeading h = FieldHeading.standard(0);
        h.update(0, 0, 0);
        h.update(0.05, 0, 0);
        assertEquals(22.5, h.update(0.20, 22.5, 0), 1e-9);
        double t = 0.25, raw = 37.5;
        for (int k = 0; k < 20; k++, t += 0.05, raw += 15) {          // 300 deg/s on; the IMU stays at 0
            double pin = MecanumMath.normalizeDegrees(raw);
            assertEquals("loop " + k, pin, h.update(t, pin, 0), 1e-9);
        }
        assertEquals(FieldHeading.Source.PINPOINT, h.source());
    }

    @Test
    public void aPinpointBackFromAGlitchCountsWhileTheImuGlitches() {
        // final review 10 Nov: the IMU, glitching in the same update, still "vouched" and kept the good Pinpoint out
        FieldHeading h = FieldHeading.standard(0);
        h.update(0, 0, 0);
        assertEquals("Pinpoint glitch: the IMU leads", 10, h.update(0.05, 100, 10), 1e-9);
        assertEquals("the Pinpoint is back, the IMU glitches", 20, h.update(0.10, 20, 100), 1e-9);
        double t = 0.15;
        for (double raw = 30; raw <= 80; raw += 10, t += 0.05) {
            assertEquals(raw, h.update(t, raw, 100), 1e-9);
        }
        assertEquals(FieldHeading.Source.PINPOINT, h.source());
    }

    @Test
    public void aRestartedLoneSensorNearTheOldHeadingIsNotTakenAsBack() {
        // the IMU alone reads 50 and restarts (reads 0 on): a return check that widens with time would take it
        FieldHeading h = FieldHeading.standard(0);
        for (double t = 0; t < 0.18; t += 0.05) {
            assertEquals(0, h.update(t, Double.NaN, 50), 1e-9);
        }
        for (double t = 0.20; t < 0.6; t += 0.05) {
            assertEquals("t=" + t, 0, h.update(t, Double.NaN, 0), 1e-9);
        }
        assertEquals("carries on from 0", 5, h.update(0.65, Double.NaN, 5), 1e-9);
        assertEquals(1, h.glitches());
    }

    @Test
    public void aRestartedLoneSensorKeepsTheTurnItSawWhileWaiting() {
        FieldHeading h = FieldHeading.standard(0);
        double t = 0, truth = 0;
        for (; t < 0.31; t += 0.05, truth += 10) {                     // 200 deg/s
            assertEquals(truth, h.update(t, Double.NaN, truth), 1e-6);
        }
        double restartAt = truth;                                       // the IMU restarts: reads 0 from here
        for (; t < 0.9; t += 0.05, truth += 10) {
            h.update(t, Double.NaN, truth - restartAt);
        }
        assertEquals("only the loop of the jump is lost", truth - 20, h.heading(), 1e-6);
        assertEquals(1, h.glitches());
    }

    @Test
    public void aLoneSensorsBadReadingWithinTheRateCheckIsIgnoredWhileStill() {
        // simulation 10 Nov: 38.4 deg in 73 ms passes the rate check (39 deg) but not from standing still
        FieldHeading h = FieldHeading.standard(0);
        double t = 0;
        for (int k = 0; k < 6; k++, t += 0.05) {
            assertEquals(0, h.update(t, Double.NaN, -74.6), 1e-9);
        }
        assertEquals(0, h.update(t + 0.023, Double.NaN, -36.2), 1e-9);
        assertEquals(1, h.glitches());
        assertEquals("back at once", 0, h.update(t + 0.093, Double.NaN, -74.7), 0.2);
    }

    @Test
    public void aBadPinpointReadingThatMatchesTheUnflippedImuIsNotFollowed() {
        // simulation 10 Nov: in the loop where the IMU flipped, a bad Pinpoint reading matched the IMU taken as is
        FieldHeading h = FieldHeading.standard(0);
        double t = 0;
        for (int k = 0; k <= 8; k++, t += 0.06) {                      // turning clockwise at 250 deg/s
            double truth = -30 - 15 * k;
            double imu = truth + 26;
            if (Math.abs(imu) > 90) {
                imu = MecanumMath.normalizeDegrees(imu + 180);
            }
            double pin = k == 6 ? 63 : truth;
            assertEquals("loop " + k, -15 * k, h.update(t, pin, imu), 1e-6);
        }
        assertEquals(1, h.glitches());
    }

    @Test
    public void imuFlipsAreLoggedAtMostOnceASecond() {
        FieldHeading h = FieldHeading.standard(0);
        h.update(0, 0, 89.9);
        int events = 0;
        for (int k = 1; k <= 20; k++) {                               // the yaw sits on +-90 and flips every loop
            h.update(k * 0.05, 0, k % 2 == 0 ? 89.9 : -89.9);
            if (h.event() != null && h.event().contains("flipped")) {
                events++;
            }
            assertEquals(0, h.heading(), 1e-9);
        }
        assertEquals(20, h.imuFlips());
        assertEquals(1, events);
        assertEquals(0, h.glitches());
    }

    @Test
    public void badPinpointReadingsThatAgreeCanNotDragTheImu() {
        // simulation 10 Nov: two bad Pinpoint readings within the agree window pulled the IMU along, then the IMU won
        FieldHeading h = FieldHeading.standard(0);
        h.update(0, 0, 0.01);                                          // a live IMU: a little noise
        h.update(0.05, 0, -0.01);
        h.update(0.10, 8, 0.01);                                       // bad, but within 10 deg of the IMU
        h.update(0.15, 17, -0.01);                                     // bad again
        assertEquals("the IMU kept the real heading", 0, h.update(0.20, 0, 0.01), 1e-9);
        assertEquals(FieldHeading.Source.PINPOINT, h.source());
        assertEquals("and the IMU alone agrees", 0, h.update(0.25, Double.NaN, -0.01), 1.0);
    }

    @Test
    public void aShortPinpointExcursionDoesNotCorruptALiveImu() {
        // review 10 Nov (GPT-6.1): Pinpoint 8, 30, 60 deg then back to 0 (80 ms loops, the robot still): the IMU was
        // lined up with it after two disagreements, so the real heading was lost for good (~60 deg off)
        FieldHeading h = FieldHeading.standard(0);
        double[] pin = {0, 0, 0, 0, 8, 30, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        double t = 0;
        for (int k = 0; k < pin.length; k++, t += 0.08) {
            h.update(t, pin[k], k % 2 == 0 ? 0.01 : -0.01);
        }
        assertEquals(0, h.heading(), 0.5);
        FieldHeading fast = FieldHeading.standard(0);                  // the same at 23 ms: 8, 17, 26 then back
        double[] pin2 = {0, 0, 0, 0, 8, 17, 26, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        t = 0;
        for (int k = 0; k < pin2.length; k++, t += 0.023) {
            fast.update(t, pin2[k], k % 2 == 0 ? 0.01 : -0.01);
        }
        assertEquals(0, fast.heading(), 0.5);
        assertEquals("the IMU alone agrees", 0, fast.update(t, Double.NaN, 0.01), 0.5);
    }

    @Test
    public void shortBadPinpointBurstsLeaveNoLastingError() {
        // review 10 Nov (Opus 5.5): still, 23 ms loops, the IMU at 37. Pinpoint 9, 20, 32 then right again left 32 deg
        // for good (with the IMU constant: it looked frozen); 9, 60, -120, 180 then right again left 180 (stick up
        // reversed: the IMU was ruled out by a fake turn rate, then a Pinpoint reading matched it flipped)
        double[][] bursts = {{9, 20, 32}, {9, 60, -120, 180}, {5, 15, 25}};
        for (double[] burst : bursts) {
            for (double noise : new double[] {0, 0.01}) {
                String what = java.util.Arrays.toString(burst) + " IMU noise " + noise;
                FieldHeading h = FieldHeading.standard(0);
                double t = 0;
                int k = 0;
                for (; k < 44; k++, t += 0.023) {
                    h.update(t, 0, 37 + (k % 2 == 0 ? noise : -noise));
                }
                for (double bad : burst) {
                    h.update(t, bad, 37 + (k++ % 2 == 0 ? noise : -noise));
                    t += 0.023;
                }
                for (int j = 0; j < 10; j++, k++, t += 0.023) {
                    h.update(t, 0, 37 + (k % 2 == 0 ? noise : -noise));
                }
                assertEquals(what, 0, h.heading(), 1.5);
                for (int j = 1; j <= 20; j++, k++, t += 0.023) {               // then a real 90 deg turn (196 deg/s)
                    double imu = 37 + 4.5 * j + (k % 2 == 0 ? noise : -noise);
                    h.update(t, 4.5 * j, Math.abs(imu) > 90 ? MecanumMath.normalizeDegrees(imu + 180) : imu);
                }
                assertEquals(what, 90, h.heading(), 1.5);
            }
        }
    }

    @Test
    public void aHardHitWhileTheImuIsApartIsFollowed() {
        // the IMU restarted a little (14 deg: a possible move, so the two are apart for a while), then a hit turns the
        // robot 60 deg in one loop: both sensors jump alike, so it is a real move, not two glitches
        FieldHeading h = FieldHeading.standard(0);
        h.update(0, 0, 0.01);
        h.update(0.023, 0, -0.01);
        double t = 0.046;
        h.update(t, 0, 14.01);
        assertTrue(h.event(), h.event() != null && h.event().contains("IMU 14 deg off the Pinpoint"));
        for (int k = 0; k < 5; k++) {
            t += 0.023;
            h.update(t, 0, k % 2 == 0 ? 13.99 : 14.01);
        }
        t += 0.023;
        assertEquals(60, h.update(t, 60, 74.01), 1e-9);
        assertTrue(h.event(), h.event() != null && h.event().contains("seen by both"));
        for (int k = 0; k < 20; k++) {
            t += 0.023;
            assertEquals("loop " + k, 60, h.update(t, 60, k % 2 == 0 ? 73.99 : 74.01), 1e-9);
        }
        assertEquals(FieldHeading.Source.PINPOINT, h.source());
    }

    @Test
    public void aPinpointThatWandersThenDropsOutHandsOverToTheImusOwnHeading() {
        FieldHeading h = FieldHeading.standard(0);
        double[] pin = {0, 0, 0, 0, 8, 30, 60};
        double t = 0;
        for (int k = 0; k < pin.length; k++, t += 0.08) {
            h.update(t, pin[k], k % 2 == 0 ? 0.01 : -0.01);
        }
        assertEquals("the Pinpoint leads meanwhile", 60, h.heading(), 0.5);
        assertEquals("the IMU kept its own heading (1 deg nudge while they agreed at 8)", 0,
                h.update(t, Double.NaN, 0.01), 1.5);
    }

    @Test
    public void anImuThatJumpedALittleIsLinedUpWithThePinpointAfterAWhile() {
        // an IMU restart small enough to be a possible move, which loses to the Pinpoint: it is apart, then lined up
        FieldHeading h = FieldHeading.standard(0);
        h.update(0, 0, 0.01);
        h.update(0.05, 0, -0.01);
        double t = 0.10;
        assertEquals(0, h.update(t, 0, 20.01), 1e-9);
        assertTrue(h.event(), h.event() != null && h.event().contains("IMU 20 deg off the Pinpoint"));
        for (int k = 1; t < 0.10 + FieldHeading.IMU_REALIGN_SEC + 0.1; k++) {
            t += 0.05;
            assertEquals(0, h.update(t, 0, k % 2 == 0 ? 20.01 : 19.99), 1e-9);
        }
        assertEquals("then the IMU alone carries on from the Pinpoint's heading", 0,
                h.update(t + 0.05, Double.NaN, 20.01), 0.05);
    }

    @Test
    public void aSmallPinpointRestartLosesToALiveImu() {
        // a restart small enough to be a possible move: the IMU (nearer the prediction, not frozen) wins
        FieldHeading h = FieldHeading.standard(0);
        double[] noise = {0, 0.02, -0.01, 0.01, -0.02, 0.02, -0.01, 0.01, 0, 0.02, -0.02, 0.01};
        h.update(0, 12, noise[0]);
        h.update(0.05, 12, noise[1]);
        h.update(0.10, 12, noise[2]);
        for (int k = 3; k < noise.length; k++) {                       // the Pinpoint restarted: reads 0 from here
            assertEquals("loop " + k, 0, h.update(k * 0.05, 0, noise[k]), 0.05);
        }
        assertTrue(h.glitches() >= 1);
        assertEquals("the Pinpoint is lined up again", FieldHeading.Source.PINPOINT, h.source());
        assertEquals(1, h.update(0.65, 1, 1), 0.05);
    }

    @Test
    public void aLoneSensorsBadReadingIsDroppedWhenTheNextOneShowsIt() {
        // 8 deg off but a possible move, so it passes; the next reading only fits if that one is dropped
        FieldHeading h = FieldHeading.standard(0);
        for (int k = 0; k <= 4; k++) {                                 // 200 deg/s
            h.update(k * 0.05, Double.NaN, 10 * k);
        }
        assertEquals(58, h.update(0.25, Double.NaN, 58), 1e-9);
        assertEquals(60, h.update(0.30, Double.NaN, 60), 1e-9);
        assertTrue(h.event().contains("dropped"));
        assertEquals(70, h.update(0.35, Double.NaN, 70), 1e-9);
    }

    @Test
    public void headingsWrapAround() {
        FieldHeading h = new FieldHeading(170, 720, 15, 0.3);
        h.update(0, 0, Double.NaN);
        assertEquals(-175, h.update(0.05, 15, Double.NaN), 1e-9);     // 170 + 15 = 185 = -175
        assertEquals(0, h.glitches());
    }
}

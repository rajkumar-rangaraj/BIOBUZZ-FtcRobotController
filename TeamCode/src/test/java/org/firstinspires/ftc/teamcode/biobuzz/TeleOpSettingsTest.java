package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Saved alliance + SHOT SPOT, and the drive-to-spot direction. */
public class TeleOpSettingsTest {

    @Test
    public void formatParseRoundTrip() {
        TeleOpSettings s = new TeleOpSettings();
        s.alliance = HiveVision.Alliance.BLUE;
        s.spotRangeIn = 52.4;
        s.spotSpeed = 1275;
        TeleOpSettings back = TeleOpSettings.parse(s.format());
        assertEquals(HiveVision.Alliance.BLUE, back.alliance);
        assertTrue(back.hasSpot());
        assertEquals(52.4, back.spotRangeIn, 0.05);
        assertEquals(1275, back.spotSpeed, 0.5);
    }

    @Test
    public void allianceOnlyAndBadFiles() {
        TeleOpSettings s = TeleOpSettings.parse("alliance=BLUE\n");
        assertEquals(HiveVision.Alliance.BLUE, s.alliance);
        assertFalse(s.hasSpot());
        assertEquals(HiveVision.Alliance.RED, TeleOpSettings.parse("").alliance);
        assertEquals(HiveVision.Alliance.RED, TeleOpSettings.parse("alliance=GREEN").alliance);
        assertFalse(TeleOpSettings.parse("spot_range_in=abc\nspot_speed=1250").hasSpot());
        assertFalse(TeleOpSettings.parse("spot_range_in=50\nspot_speed=99999").hasSpot());
    }

    @Test
    public void spotForwardDrivesTowardTheSavedDistance() {
        // camera + launcher are on the back: too far from the HIVE -> drive backward (negative forward)
        assertTrue(MotionController.spotForward(70, 50) < 0);
        assertTrue(MotionController.spotForward(30, 50) > 0);
        assertEquals(0, MotionController.spotForward(50.8, 50), 0);
        assertEquals(-BioBuzzConstants.SPOT_MAX_POWER, MotionController.spotForward(200, 50), 1e-9);
        assertEquals(BioBuzzConstants.SPOT_MIN_POWER, MotionController.spotForward(48, 50), 1e-9);
    }

    @Test
    public void tuneValuesRoundTripAndBadValuesFallBack() {
        TeleOpSettings s = new TeleOpSettings();
        s.aimTrimDeg = -2.5;
        s.feedPower = 0.45;
        s.feedPauseSec = 0.7;
        TeleOpSettings back = TeleOpSettings.parse(s.format());
        assertEquals(-2.5, back.aimTrimDeg, 1e-9);
        assertEquals(0.45, back.feedPower, 1e-9);
        assertEquals(0.7, back.feedPauseSec, 1e-9);
        TeleOpSettings bad = TeleOpSettings.parse("aim_trim_deg=45\nfeed_power=2\nfeed_pause_sec=-1\n");
        assertEquals(0, bad.aimTrimDeg, 0);
        assertEquals(BioBuzzConstants.INDEXER_FEED_POWER, bad.feedPower, 0);
        assertEquals(BioBuzzConstants.SHOT_MIN_INTERVAL_SEC, bad.feedPauseSec, 0);
    }

    @Test
    public void spotSavedBeforeClusterAimingIsDropped() {
        // 31 Oct saves measured to the visible-tag average, not the CELL opening: must be saved again
        assertFalse(TeleOpSettings.parse("alliance=BLUE\nspot_range_in=55.6\nspot_speed=1250\n").hasSpot());
        assertTrue(TeleOpSettings.parse("spot_range_in=55.6\nspot_speed=1250\nspot_ref=cluster\n").hasSpot());
    }

    @Test
    public void holdTurnTurnsBackToTheAimedHeading() {
        assertEquals(0, MotionController.holdTurnCw(-35, -35.5), 0);                 // inside the dead band
        assertTrue(MotionController.holdTurnCw(-35, -30) > 0);                        // drifted CCW -> turn clockwise
        assertTrue(MotionController.holdTurnCw(-35, -40) < 0);
        assertTrue(Math.abs(MotionController.holdTurnCw(-35, -37)) >= BioBuzzConstants.TURN_MIN_POWER);
        assertTrue(Math.abs(MotionController.holdTurnCw(0, 90)) <= BioBuzzConstants.HOLD_MAX_TURN);
    }

    @Test
    public void positiveAimTrimTurnsTheRobotLeft() {
        // CELL dead ahead of the camera, trim +2: aiming must turn counter-clockwise (left), so the shots go left
        HiveVision.Target t = new HiveVision.Target(HiveVision.Cell.BLUE_REAR, 4, 0, 2, 55, 0, 0, 55, 0);
        assertEquals(2, t.bearingDeg, 1e-9);
        assertTrue(HiveVision.aimTurnCw(t.bearingDeg) < 0);
    }
}
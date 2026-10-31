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
}

package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.CAMERA_LENS_HEIGHT_IN;
import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.CAMERA_TILT_DEG;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

/** Floor distance and tag height from the tilted camera's x/y/z, built the way the FTC SDK reports a pose. */
public class VisionGeometryTest {

    /** A tag at floor offsets (right, ahead) and height above the floor, as the tilted camera sees it. */
    private static HiveVision.Target seen(double rightIn, double aheadIn, double heightIn) {
        double tilt = Math.toRadians(CAMERA_TILT_DEG);
        double up = heightIn - CAMERA_LENS_HEIGHT_IN;
        double x = rightIn;
        double y = aheadIn * Math.cos(tilt) + up * Math.sin(tilt);
        double z = -aheadIn * Math.sin(tilt) + up * Math.cos(tilt);
        double range = Math.hypot(x, y);                                     // SDK: range = hypot(x, y)
        double bearing = Math.toDegrees(Math.atan2(-x, y));
        double elevation = Math.toDegrees(Math.atan2(z, y));
        return new HiveVision.Target(HiveVision.Cell.RED_AUDIENCE, 4, bearing, range, elevation, x, y, z);
    }

    @Test
    public void tagStraightBehindTheRobot() {
        HiveVision.Target t = seen(0, 48, 40);
        assertEquals(48, t.horizontalIn(), 1e-9);
        assertEquals(40, t.heightIn(), 1e-9);
        assertEquals(0, t.bearingDeg, 1e-9);
    }

    @Test
    public void tagOffToTheSideKeepsTheSameHeight() {
        for (double[] p : new double[][]{{20, 60}, {-20, 30}, {10, 90}}) {
            HiveVision.Target t = seen(p[0], p[1], 40);
            assertEquals(Math.hypot(p[0], p[1]), t.horizontalIn(), 1e-9);
            assertEquals(40, t.heightIn(), 1e-9);
        }
    }

    @Test
    public void tagOnTheRightNeedsAClockwiseTurn() {
        HiveVision.Target t = seen(20, 60, 40);
        assertEquals(true, t.bearingDeg < 0);
        assertEquals(true, MotionController.aimTurnCw(t.bearingDeg) > 0);
    }
}

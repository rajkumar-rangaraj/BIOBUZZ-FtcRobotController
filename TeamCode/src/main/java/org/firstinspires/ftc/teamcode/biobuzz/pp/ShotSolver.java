package org.firstinspires.ftc.teamcode.biobuzz.pp;

/**
 * From the pose: how far the camera is from the target CELL (the flywheel table is keyed by the camera's floor
 * distance, so this matches the camera TeleOp) and which heading points the launcher at it.
 *
 * "Aimed" means the same thing as the camera aim: the camera's bearing to the CELL plus the aim trim is 0 (+ trim turns
 * the robot and the shots LEFT, see HiveVision.Target.bearingDeg). The camera sits on the launcher's side, behind the
 * robot centre, so the robot's back faces the CELL. No FTC SDK dependency: unit-tested (ShotSolverTest).
 */
public final class ShotSolver {
    private ShotSolver() {
    }

    public static final class Aim {
        /** Camera floor distance to the CELL opening now (in). */
        public final double distanceIn;
        /** Heading (CCW deg, alliance frame) at which the robot is aimed. */
        public final double aimHeadingDeg;
        /** Aim heading minus the heading now (CCW +: turn counter-clockwise to aim). */
        public final double errorDeg;

        Aim(double distanceIn, double aimHeadingDeg, double errorDeg) {
            this.distanceIn = distanceIn;
            this.aimHeadingDeg = aimHeadingDeg;
            this.errorDeg = errorDeg;
        }
    }

    /** Floor-plane aim (camera not tilted, CELL at lens height): the trim is a floor angle. */
    public static Aim solve(double x, double y, double headingDeg, double cellX, double cellY, double camBackIn,
                            double camLeftIn, double aimTrimDeg) {
        return solve(x, y, headingDeg, cellX, cellY, camBackIn, camLeftIn, aimTrimDeg, 0, 0);
    }

    /**
     * @param tiltDeg   camera tilt up (CAMERA_TILT_DEG)
     * @param cellUpIn  CELL opening height above the camera lens. The trim is calibrated on the SDK bearing, which is
     *                  measured in the tilted camera's frame: aimed = SDK bearing + trim = 0, as BioBuzz TeleOp's X.
     */
    public static Aim solve(double x, double y, double headingDeg, double cellX, double cellY, double camBackIn,
                            double camLeftIn, double aimTrimDeg, double tiltDeg, double cellUpIn) {
        double h = Math.toDegrees(Math.atan2(cellY - y, cellX - x)) + 180;
        for (int i = 0; i < 40; i++) {                      // a turn moves the SDK bearing by less than 1 deg: converges
            double step = sdkBearing(x, y, h, cellX, cellY, camBackIn, camLeftIn, tiltDeg, cellUpIn) + aimTrimDeg;
            h = FieldPose.norm(h + step);
            if (Math.abs(step) < 1e-6) {
                break;
            }
        }
        double[] cam = camera(x, y, headingDeg, camBackIn, camLeftIn);
        double dist = Math.hypot(cellX - cam[0], cellY - cam[1]);
        return new Aim(dist, h, FieldPose.norm(h - headingDeg));
    }

    /** The bearing the SDK reports for the CELL at this heading (tilted camera frame, CCW / left +). */
    static double sdkBearing(double x, double y, double headingDeg, double cellX, double cellY, double camBackIn,
                             double camLeftIn, double tiltDeg, double cellUpIn) {
        double[] cam = camera(x, y, headingDeg, camBackIn, camLeftIn);
        double ex = cellX - cam[0], ey = cellY - cam[1];
        double axis = Math.toRadians(headingDeg + 180);
        double fwd = ex * Math.cos(axis) + ey * Math.sin(axis);     // along the camera's floor axis
        double left = -ex * Math.sin(axis) + ey * Math.cos(axis);
        double t = Math.toRadians(tiltDeg);
        double lens = fwd * Math.cos(t) + cellUpIn * Math.sin(t);   // along the tilted lens axis
        return Math.toDegrees(Math.atan2(left, lens));
    }

    /** Camera's horizontal bearing to the CELL at this heading (CCW +, 0 = straight out of the robot's back). */
    static double cameraBearing(double x, double y, double headingDeg, double cellX, double cellY, double camBackIn,
                                double camLeftIn) {
        double[] cam = camera(x, y, headingDeg, camBackIn, camLeftIn);
        double toCell = Math.toDegrees(Math.atan2(cellY - cam[1], cellX - cam[0]));
        return FieldPose.norm(toCell - (headingDeg + 180));
    }

    /** Camera lens position on the field. */
    static double[] camera(double x, double y, double headingDeg, double camBackIn, double camLeftIn) {
        double c = Math.cos(Math.toRadians(headingDeg)), s = Math.sin(Math.toRadians(headingDeg));
        return new double[]{x + c * -camBackIn - s * camLeftIn, y + s * -camBackIn + c * camLeftIn};
    }
}

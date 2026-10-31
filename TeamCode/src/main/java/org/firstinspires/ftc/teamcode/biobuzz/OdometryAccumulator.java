package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * Turns field-frame odometry (Pinpoint: X forward, Y left at reset, heading CCW+) into the robot-relative
 * "forward inches / strafe-right inches travelled" that MotionController uses, by rotating each small field step
 * into the robot frame. Works on deltas, so it can also take wheel-encoder steps if the Pinpoint drops out.
 * No FTC SDK dependency (unit tested on the laptop).
 */
public class OdometryAccumulator {
    private double forward = 0, strafeRight = 0;
    private double lastX, lastY;
    private boolean haveField = false;

    /** One Pinpoint reading. */
    public void fieldUpdate(double xIn, double yIn, double headingDeg) {
        if (haveField) {
            double dx = xIn - lastX, dy = yIn - lastY;
            double h = Math.toRadians(headingDeg);
            double c = Math.cos(h), s = Math.sin(h);
            forward += dx * c + dy * s;          // field step projected on the robot's forward axis
            strafeRight += dx * s - dy * c;      // ... and on its right-hand axis (field Y is LEFT)
        }
        lastX = xIn;
        lastY = yIn;
        haveField = true;
    }

    /** Robot-frame step from another source (wheel encoders) when the Pinpoint is not usable. */
    public void robotStep(double dForward, double dStrafeRight) {
        forward += dForward;
        strafeRight += dStrafeRight;
        haveField = false;                       // next Pinpoint reading restarts from there
    }

    public double forwardInches() {
        return forward;
    }

    public double strafeRightInches() {
        return strafeRight;
    }
}

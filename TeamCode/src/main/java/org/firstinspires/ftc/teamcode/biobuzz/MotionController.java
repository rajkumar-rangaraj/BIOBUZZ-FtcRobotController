package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

/** Closed-loop encoder + IMU moves for a mecanum drive. No FTC SDK dependency. */
public class MotionController {
    private final DriveIO io;

    public MotionController(DriveIO io) {
        this.io = io;
    }

    /** Robot-relative move while holding the starting heading. Returns true if the target was reached. */
    public boolean driveRelative(double forwardIn, double strafeRightIn, double timeoutSec) {
        double f0 = io.forwardInches();
        double s0 = io.strafeRightInches();
        double holdHeading = io.headingDeg();
        double end = io.seconds() + timeoutSec;
        boolean reached = false;
        while (io.active() && io.seconds() < end) {
            double errF = forwardIn - (io.forwardInches() - f0);
            double errS = strafeRightIn - (io.strafeRightInches() - s0);
            double dist = Math.hypot(errF, errS);
            if (dist < AUTO_DRIVE_TOLERANCE_IN) {
                reached = true;
                break;
            }
            double speed = clip(dist * AUTO_DRIVE_KP, AUTO_DRIVE_MIN_POWER, AUTO_DRIVE_POWER);
            double headingErr = MecanumMath.normalizeDegrees(holdHeading - io.headingDeg());
            double turnCw = clip(-headingErr * HEADING_KP, -0.3, 0.3);
            io.drive(errF / dist * speed, errS / dist * speed, turnCw);
            io.loopTick();
        }
        io.drive(0, 0, 0);
        return reached;
    }

    /** Turn to an absolute IMU heading (CCW +). Returns true if it settled inside the tolerance. */
    public boolean turnToHeading(double targetDeg, double timeoutSec) {
        double end = io.seconds() + timeoutSec;
        int settled = 0;
        while (io.active() && io.seconds() < end) {
            double err = MecanumMath.normalizeDegrees(targetDeg - io.headingDeg());
            if (Math.abs(err) < TURN_TOLERANCE_DEG) {
                io.drive(0, 0, 0);
                if (++settled >= 3) {
                    return true;
                }
            } else {
                settled = 0;
                double power = clip(Math.abs(err) * HEADING_KP, TURN_MIN_POWER, TURN_MAX_POWER);
                io.drive(0, 0, -Math.signum(err) * power);
            }
            io.loopTick();
        }
        io.drive(0, 0, 0);
        return false;
    }

    public void waitSeconds(double seconds) {
        double end = io.seconds() + seconds;
        while (io.active() && io.seconds() < end) {
            io.loopTick();
        }
    }

    /** Clockwise turn power that rotates the launcher toward a camera target at this bearing (left = +). */
    public static double aimTurnCw(double bearingDeg) {
        if (Math.abs(bearingDeg) <= AIM_TOLERANCE_DEG) {
            return 0;
        }
        return -Math.signum(bearingDeg) * clip(Math.abs(bearingDeg) * AIM_KP, AIM_MIN_TURN, AIM_MAX_TURN);
    }

    static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}

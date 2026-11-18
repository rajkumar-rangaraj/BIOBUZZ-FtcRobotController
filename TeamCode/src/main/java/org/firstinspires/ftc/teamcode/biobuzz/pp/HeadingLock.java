package org.firstinspires.ftc.teamcode.biobuzz.pp;

/**
 * Turns the robot onto an aim heading from the Pinpoint heading every loop (no waiting for camera frames), and holds
 * it when the robot is pushed. PD: the turn-rate term damps the swing at the higher power needed against a push.
 * No FTC SDK dependency: unit-tested (HeadingLockTest).
 */
public class HeadingLock {

    private final double kp, kd, minTurn, maxTurn, deadbandDeg;

    public HeadingLock(double kp, double kd, double minTurn, double maxTurn, double deadbandDeg) {
        this.kp = kp;
        this.kd = kd;
        this.minTurn = minTurn;
        this.maxTurn = maxTurn;
        this.deadbandDeg = deadbandDeg;
    }

    public static HeadingLock standard() {
        return new HeadingLock(PpConstants.LOCK_KP, PpConstants.LOCK_KD, PpConstants.LOCK_MIN_TURN,
                PpConstants.LOCK_MAX_TURN, PpConstants.LOCK_DEADBAND_DEG);
    }

    /**
     * @param errorDeg   aim heading minus heading (CCW +)
     * @param turnRateDps the robot's turn rate now (CCW +)
     * @return clockwise turn power (BioBuzzRobot.drive's turn), 0 inside the dead band
     */
    public double turnCw(double errorDeg, double turnRateDps) {
        if (Math.abs(errorDeg) <= deadbandDeg) {
            return 0;
        }
        double ccw = kp * errorDeg - kd * turnRateDps;
        if (Math.signum(ccw) == Math.signum(errorDeg)) {
            ccw = Math.signum(ccw) * Math.max(minTurn, Math.min(maxTurn, Math.abs(ccw)));
        } else {
            ccw = Math.signum(ccw) * Math.min(maxTurn, Math.abs(ccw));   // braking a fast swing: no minimum
        }
        return -ccw;
    }
}

package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * TeleOp INIT: the robot stands still, so the Pinpoint's heading must too. A Pinpoint whose gyro was calibrated while
 * the robot moved (the reset at INIT) drifts all match, and field-centric "up" turns with it: this finds it so INIT can
 * recalibrate. Compared with the IMU (which drifts far less), each sensor's turn added up loop by loop; an IMU reading
 * is taken flipped by 180 deg when that is the smaller step (its yaw flips at +-90, and a robot standing there would
 * flip every loop). While the robot is being moved the comparison starts over. No FTC SDK dependency: unit-tested.
 */
public class PinpointInitCheck {
    private final double driftDeg;
    private final double stillDeg;
    private double prevPin = Double.NaN, prevImu;
    private double pinTurn, imuTurn;                 // added up since the comparison started

    /**
     * @param driftDeg the Pinpoint turning this much more than the IMU while the robot is still: recalibrate
     * @param stillDeg the IMU turning more than this: the robot is being moved, start over
     */
    public PinpointInitCheck(double driftDeg, double stillDeg) {
        this.driftDeg = driftDeg;
        this.stillDeg = stillDeg;
    }

    public static PinpointInitCheck standard() {
        return new PinpointInitCheck(BioBuzzConstants.PINPOINT_INIT_DRIFT_DEG, BioBuzzConstants.PINPOINT_INIT_STILL_DEG);
    }

    /**
     * Once per INIT loop.
     *
     * @param pinpointOk the Pinpoint has a reading (not while it calibrates)
     * @return true: recalibrate the Pinpoint now (the check starts over by itself)
     */
    public boolean update(boolean pinpointOk, double pinpointDeg, double imuDeg) {
        if (!pinpointOk || Double.isNaN(pinpointDeg) || Double.isNaN(imuDeg)) {
            prevPin = Double.NaN;
            return false;
        }
        if (Double.isNaN(prevPin)) {
            startOver(pinpointDeg, imuDeg);
            return false;
        }
        double imuStep = MecanumMath.normalizeDegrees(imuDeg - prevImu);
        if (Math.abs(imuStep) > 90) {                        // a yaw flip, not a turn
            imuStep = MecanumMath.normalizeDegrees(imuStep + 180);
        }
        pinTurn += MecanumMath.normalizeDegrees(pinpointDeg - prevPin);
        imuTurn += imuStep;
        prevPin = pinpointDeg;
        prevImu = imuDeg;
        if (Math.abs(imuTurn) > stillDeg) {
            startOver(pinpointDeg, imuDeg);
            return false;
        }
        if (Math.abs(pinTurn - imuTurn) > driftDeg) {
            prevPin = Double.NaN;
            return true;
        }
        return false;
    }

    private void startOver(double pinpointDeg, double imuDeg) {
        prevPin = pinpointDeg;
        prevImu = imuDeg;
        pinTurn = 0;
        imuTurn = 0;
    }
}

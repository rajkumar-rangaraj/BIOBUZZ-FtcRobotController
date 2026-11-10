package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * TeleOp INIT: the robot stands still, so the Pinpoint's heading must too. A Pinpoint whose gyro was calibrated while
 * the robot moved (the reset at INIT) drifts all match, and field-centric "up" turns with it: this finds it so INIT can
 * recalibrate. Compared with the IMU (which drifts far less); while the robot is being moved (or the IMU yaw flips) the
 * comparison starts over. No FTC SDK dependency: unit-tested.
 */
public class PinpointInitCheck {
    private final double driftDeg;
    private final double stillDeg;
    private double pinRef = Double.NaN, imuRef;

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
            pinRef = Double.NaN;
            return false;
        }
        if (Double.isNaN(pinRef)) {
            pinRef = pinpointDeg;
            imuRef = imuDeg;
            return false;
        }
        double imuTurn = MecanumMath.normalizeDegrees(imuDeg - imuRef);
        if (Math.abs(imuTurn) > stillDeg) {
            pinRef = pinpointDeg;
            imuRef = imuDeg;
            return false;
        }
        if (Math.abs(MecanumMath.normalizeDegrees(pinpointDeg - pinRef) - imuTurn) > driftDeg) {
            pinRef = Double.NaN;
            return true;
        }
        return false;
    }
}

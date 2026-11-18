package org.firstinspires.ftc.teamcode.biobuzz.pp;

/**
 * Fire gate with a pose aim (BioBuzz TeleOp PP, POSE mode): may the next volley ball go?
 * <ul>
 *   <li>Aimed: pose aim error within the tolerance, not turning, the camera bearing (when it sees the CELL) within its
 *       own tolerance, not driving to the SHOT SPOT, the driver not turning.</li>
 *   <li>A ready ball held {@code givewaySec} goes anyway only if it is nearly aimed (pose AND camera within
 *       {@code givewayMaxDeg}, turning slowly) and the driver isn't turning. A SHOT SPOT that can't be reached in that
 *       time is given up for the rest of the volley instead ({@link #spotGaveWay}); the aim is still checked.</li>
 * </ul>
 * No FTC SDK dependency: unit-tested (AimAndVolleyTest).
 */
public class PoseFireGate {

    public enum Hold { NONE, SPOT, DRIVER_TURNING, TURNING, CAMERA, AIM }

    private final double toleranceDeg, camToleranceDeg, maxTurnDps, givewaySec, givewayMaxDeg;
    private double holdSince = Double.NaN;
    private boolean spotGaveWay = false;
    private boolean gaveWay = false;
    private Hold hold = Hold.NONE;

    public PoseFireGate(double toleranceDeg, double camToleranceDeg, double maxTurnDps, double givewaySec,
                        double givewayMaxDeg) {
        this.toleranceDeg = toleranceDeg;
        this.camToleranceDeg = camToleranceDeg;
        this.maxTurnDps = maxTurnDps;
        this.givewaySec = givewaySec;
        this.givewayMaxDeg = givewayMaxDeg;
    }

    public static PoseFireGate standard() {
        return new PoseFireGate(PpConstants.FIRE_TOLERANCE_DEG,
                org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.AIM_FIRE_TOLERANCE_DEG,
                PpConstants.FIRE_MAX_TURN_DPS, PpConstants.FIRE_GIVEWAY_SEC, PpConstants.FIRE_GIVEWAY_MAX_ERROR_DEG);
    }

    /**
     * @param camBearingDeg camera bearing to the CELL, trim included; NaN when the camera doesn't see it
     * @return true when the next ball may go (as far as the aim is concerned)
     */
    public boolean update(double t, boolean volleyOn, boolean ready, double aimErrorDeg, double turnRateDps,
                          double camBearingDeg, boolean spotHold, boolean driverTurns) {
        gaveWay = false;
        boolean camSees = !Double.isNaN(camBearingDeg);
        boolean camOk = !camSees || Math.abs(camBearingDeg) <= camToleranceDeg;
        boolean poseOk = Math.abs(aimErrorDeg) <= toleranceDeg;
        boolean steady = Math.abs(turnRateDps) <= maxTurnDps;
        if (!volleyOn || (poseOk && camOk && steady && !spotHold && !driverTurns)) {
            holdSince = Double.NaN;
            hold = Hold.NONE;
            return true;
        }
        hold = spotHold ? Hold.SPOT : driverTurns ? Hold.DRIVER_TURNING : !steady ? Hold.TURNING
                : !poseOk ? Hold.AIM : Hold.CAMERA;
        if (ready && Double.isNaN(holdSince)) {
            holdSince = t;
        }
        boolean waited = !Double.isNaN(holdSince) && t - holdSince >= givewaySec;
        if (!waited) {
            return false;
        }
        if (spotHold) {
            spotGaveWay = true;
            holdSince = t;                              // the aim gets its own time from here
            return false;
        }
        boolean nearly = Math.abs(aimErrorDeg) <= givewayMaxDeg
                && (!camSees || Math.abs(camBearingDeg) <= givewayMaxDeg) && Math.abs(turnRateDps) <= 2 * maxTurnDps;
        if (!driverTurns && nearly) {
            gaveWay = true;
            return true;
        }
        return false;
    }

    /** A ball went: the next one earns its own give-way time. */
    public void ballFed() {
        holdSince = Double.NaN;
    }

    /** The volley ended: everything starts over. */
    public void reset() {
        holdSince = Double.NaN;
        spotGaveWay = false;
        hold = Hold.NONE;
    }

    public Hold hold() {
        return hold;
    }

    /** The last update let a ball go only because the hold timed out. */
    public boolean gaveWay() {
        return gaveWay;
    }

    /** The SHOT SPOT couldn't be reached this volley: stop driving there. */
    public boolean spotGaveWay() {
        return spotGaveWay;
    }
}

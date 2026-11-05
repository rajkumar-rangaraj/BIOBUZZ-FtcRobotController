package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * TeleOp aim gate: decides whether the windmill may feed the next ball, from the camera's view of our CELL.
 *
 * <ul>
 *   <li>CELL in view: feed only when the aim (trim included) is within the tolerance and the robot isn't turning.</li>
 *   <li>CELL just lost (seen within {@code lostHoldSec}): still hold. Most dropouts are motion blur while turning,
 *       exactly when the aim is bad, and one empty frame must not release a held burst.</li>
 *   <li>CELL not seen for longer: no gate. The driver aims by eye.</li>
 *   <li>A ready shot held back for {@code givewaySec} goes anyway ({@link #gaveWay()}), so a knocked camera or a stale
 *       aim trim can slow the shooting but never stop it.</li>
 * </ul>
 * The give-way clock restarts when FIRE is released or after a ball is fed. No FTC SDK dependency: unit-tested.
 */
public class AimGate {

    public enum State { OFF, NO_CELL, AIMED, HOLD_AIM, HOLD_TURNING, HOLD_LOST, GIVE_WAY }

    private final double toleranceDeg;
    private final double lostHoldSec;
    private final double givewaySec;
    private final double maxTurn;

    private boolean enabled = true;
    private double lastSeen = Double.NaN;
    private double holdSince = Double.NaN;
    private State state = State.NO_CELL;

    public AimGate(double toleranceDeg, double lostHoldSec, double givewaySec, double maxTurn) {
        this.toleranceDeg = toleranceDeg;
        this.lostHoldSec = lostHoldSec;
        this.givewaySec = givewaySec;
        this.maxTurn = maxTurn;
    }

    public static AimGate standard() {
        AimGate g = new AimGate(BioBuzzConstants.AIM_FIRE_TOLERANCE_DEG, BioBuzzConstants.AIM_LOST_HOLD_SEC,
                BioBuzzConstants.AIM_GATE_GIVEWAY_SEC, BioBuzzConstants.AIM_GATE_MAX_TURN);
        g.setEnabled(BioBuzzConstants.AIM_GATE_ENABLED);
        return g;
    }

    public void setEnabled(boolean on) {
        enabled = on;
    }

    public boolean enabled() {
        return enabled;
    }

    /**
     * Call once per loop.
     *
     * @param t            seconds, monotonic
     * @param fire         driver holds FIRE
     * @param shotReady    flywheel ready (only a ready shot can be "held back")
     * @param cellInView   our CELL is in view this loop
     * @param bearingDeg   aim error to the CELL, trim included (ignored when not in view)
     * @param turnPower    commanded turn power this loop (auto-aim or stick)
     * @return true when the next ball may be fed
     */
    public boolean update(double t, boolean fire, boolean shotReady, boolean cellInView, double bearingDeg,
                          double turnPower) {
        if (cellInView) {
            lastSeen = t;
        }
        if (!enabled) {
            state = State.OFF;
            holdSince = Double.NaN;
            return true;
        }
        if (cellInView) {
            if (Math.abs(bearingDeg) > toleranceDeg) {
                state = State.HOLD_AIM;
            } else if (Math.abs(turnPower) > maxTurn) {
                state = State.HOLD_TURNING;
            } else {
                state = State.AIMED;
            }
        } else if (!Double.isNaN(lastSeen) && t - lastSeen < lostHoldSec) {
            state = State.HOLD_LOST;
        } else {
            state = State.NO_CELL;
        }
        boolean holding = state == State.HOLD_AIM || state == State.HOLD_TURNING || state == State.HOLD_LOST;
        if (!fire || !shotReady || !holding) {
            holdSince = Double.NaN;                 // nothing held back right now
            return !holding;
        }
        if (Double.isNaN(holdSince)) {
            holdSince = t;
        }
        if (t - holdSince >= givewaySec) {
            state = State.GIVE_WAY;
            return true;
        }
        return false;
    }

    /** Call when a ball was fed: the next one must earn its own give-way time. */
    public void ballFed() {
        holdSince = Double.NaN;
    }

    public State state() {
        return state;
    }

    /** True when the last update let a ball go only because the hold timed out. */
    public boolean gaveWay() {
        return state == State.GIVE_WAY;
    }

    /** True when a ready shot is being held back right now. */
    public boolean holding() {
        return state == State.HOLD_AIM || state == State.HOLD_TURNING || state == State.HOLD_LOST;
    }
}

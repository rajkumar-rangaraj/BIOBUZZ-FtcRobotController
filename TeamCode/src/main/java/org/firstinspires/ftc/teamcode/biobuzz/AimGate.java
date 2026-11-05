package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * TeleOp aim gate: decides whether the windmill may feed the next ball, from the camera's view of our CELL.
 *
 * <ul>
 *   <li>CELL in view: feed only when the aim (trim included) is within the tolerance and the robot isn't turning.
 *       While X drives to a saved SHOT SPOT, also wait until the distance is right.</li>
 *   <li>CELL just lost (seen within {@code lostHoldSec}): still hold. Most dropouts are motion blur while turning,
 *       exactly when the aim is bad, and one empty frame must not release a held burst.</li>
 *   <li>CELL not seen for longer: no gate. The driver aims by eye.</li>
 *   <li>Every hold gives way {@code givewaySec} after the first ready ball was held back ({@link #gaveWay()}), so a
 *       knocked camera, a stale aim trim or a blocked SHOT SPOT can slow the shooting but never stop it. The clock
 *       keeps running while READY flickers; it restarts only when FIRE is released or a ball is fed. A SHOT SPOT that
 *       gave way stays released until FIRE is let go, but aim and turning are still checked (with their own time).</li>
 *   <li>Off (X in INIT): never holds.</li>
 * </ul>
 * The flywheel speed gate is separate: a ball never goes before the flywheel is ready. No FTC SDK dependency: unit-tested.
 */
public class AimGate {

    public enum State { OFF, NO_CELL, AIMED, HOLD_SPOT, HOLD_AIM, HOLD_TURNING, HOLD_LOST, GIVE_WAY }

    private final double toleranceDeg;
    private final double lostHoldSec;
    private final double givewaySec;
    private final double maxTurn;

    private boolean enabled = true;
    private double lastSeen = Double.NaN;
    private double holdSince = Double.NaN;
    private boolean spotReleased = false;
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

    /** Same as {@link #update(double, boolean, boolean, boolean, double, double, boolean)} with no SHOT SPOT drive. */
    public boolean update(double t, boolean fire, boolean shotReady, boolean cellInView, double bearingDeg,
                          double turnPower) {
        return update(t, fire, shotReady, cellInView, bearingDeg, turnPower, false);
    }

    /**
     * Call once per loop.
     *
     * @param t            seconds, monotonic
     * @param fire         driver holds FIRE
     * @param shotReady    flywheel ready (the give-way clock starts at the first ready ball held back)
     * @param cellInView   our CELL is in view this loop
     * @param bearingDeg   aim error to the CELL, trim included (ignored when not in view)
     * @param turnPower    commanded turn power this loop (auto-aim or stick)
     * @param spotHold     X is driving the robot to the saved SHOT SPOT distance and isn't there yet
     * @return true when the next ball may be fed (as far as the aim is concerned)
     */
    public boolean update(double t, boolean fire, boolean shotReady, boolean cellInView, double bearingDeg,
                          double turnPower, boolean spotHold) {
        if (cellInView) {
            lastSeen = t;
        }
        if (!fire) {
            holdSince = Double.NaN;                 // a new FIRE press earns a fresh give-way time
            spotReleased = false;
        }
        if (!enabled) {
            state = State.OFF;
            holdSince = Double.NaN;
            spotReleased = false;
            return true;
        }
        State aim;                                  // what the camera says, apart from the SHOT SPOT distance
        if (cellInView) {
            if (Math.abs(bearingDeg) > toleranceDeg) {
                aim = State.HOLD_AIM;
            } else if (Math.abs(turnPower) > maxTurn) {
                aim = State.HOLD_TURNING;
            } else {
                aim = State.AIMED;
            }
        } else if (!Double.isNaN(lastSeen) && t - lastSeen < lostHoldSec) {
            aim = State.HOLD_LOST;
        } else {
            aim = State.NO_CELL;
        }
        boolean spot = cellInView && spotHold && !spotReleased;
        state = spot ? State.HOLD_SPOT : aim;
        if (!holding()) {
            return true;
        }
        if (fire && shotReady && Double.isNaN(holdSince)) {
            holdSince = t;                          // the first ready ball held back starts the clock
        }
        if (Double.isNaN(holdSince) || t - holdSince < givewaySec) {
            return false;
        }
        if (spot) {
            // The distance gives way (e.g. a robot in the way) until FIRE is released; aim and turning are still
            // checked, with their own give-way time, so the ball never leaves while the robot is still turning.
            spotReleased = true;
            holdSince = Double.NaN;
            if (aim != State.AIMED && aim != State.NO_CELL) {
                state = aim;
                holdSince = t;
                return false;
            }
        }
        state = State.GIVE_WAY;
        return true;
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

    /** True when the gate holds the next ball back right now (whether or not the flywheel is ready). */
    public boolean holding() {
        return state == State.HOLD_SPOT || state == State.HOLD_AIM || state == State.HOLD_TURNING
                || state == State.HOLD_LOST;
    }
}

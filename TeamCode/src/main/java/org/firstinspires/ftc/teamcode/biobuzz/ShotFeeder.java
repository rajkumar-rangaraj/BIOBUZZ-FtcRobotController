package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * Fires one ball at a time: turn the windmill exactly one pocket, then wait until the flywheel is back at speed (and a
 * minimum gap has passed) before the next pocket. A pocket that doesn't finish in time is a jam: back off briefly and
 * retry; after too many jams it stops and waits for the fire request to be released.
 * No FTC SDK dependency, so it is unit-tested on the laptop.
 */
public class ShotFeeder {

    public enum State { IDLE, WAIT_READY, FEEDING, UNJAM, JAMMED }

    /** A pocket is done once the encoder reaches its boundary (slow approach: overshoot is small and never adds up). */
    static final double STOP_TOLERANCE_TICKS = 0;

    private final double pocketTicks;
    private final double feedPower;
    private final double unjamPower;
    private final double minIntervalSec;
    private final double pocketTimeoutSec;
    private final double unjamSec;
    private final int maxJams;
    private final double rampTicks;
    private final double minPower;

    private State state = State.IDLE;
    private double stateStart = 0;
    private int pocketStartTicks = 0;
    private double targetTicks = 0;
    private boolean haveTarget = false;
    private double lastShotTime = -1e9;
    private int shots = 0;
    private int jams = 0;
    private double power = 0;
    private String event = null;

    public ShotFeeder(double pocketTicks, double feedPower, double unjamPower, double minIntervalSec,
                      double pocketTimeoutSec, double unjamSec, int maxJams) {
        this(pocketTicks, feedPower, unjamPower, minIntervalSec, pocketTimeoutSec, unjamSec, maxJams,
                BioBuzzConstants.INDEXER_RAMP_TICKS, BioBuzzConstants.INDEXER_MIN_POWER);
    }

    /**
     * @param rampTicks the last ticks before the pocket boundary are driven at reduced power (proportional), so the
     *                  windmill stops on the boundary instead of overshooting by a loop's worth of travel
     * @param minPower  lowest power used near the boundary (must still turn the windmill with a ball in it)
     */
    public ShotFeeder(double pocketTicks, double feedPower, double unjamPower, double minIntervalSec,
                      double pocketTimeoutSec, double unjamSec, int maxJams, double rampTicks, double minPower) {
        this.pocketTicks = pocketTicks;
        this.feedPower = feedPower;
        this.unjamPower = unjamPower;
        this.minIntervalSec = minIntervalSec;
        this.pocketTimeoutSec = pocketTimeoutSec;
        this.unjamSec = unjamSec;
        this.maxJams = maxJams;
        this.rampTicks = rampTicks;
        this.minPower = minPower;
    }

    /** Standard robot settings from BioBuzzConstants. */
    public static ShotFeeder standard() {
        return new ShotFeeder(BioBuzzConstants.INDEXER_TICKS_PER_POCKET, BioBuzzConstants.INDEXER_FEED_POWER,
                BioBuzzConstants.INDEXER_UNJAM_POWER, BioBuzzConstants.SHOT_MIN_INTERVAL_SEC,
                BioBuzzConstants.SHOT_POCKET_TIMEOUT_SEC, BioBuzzConstants.SHOT_UNJAM_SEC, BioBuzzConstants.SHOT_MAX_JAMS);
    }

    /**
     * @param t              time in seconds (any monotonic clock)
     * @param indexerTicks   windmill encoder
     * @param launcherReady  flywheel at speed
     * @param wantFire       driver holds FIRE / AUTO wants another shot
     * @return windmill power to apply this loop
     */
    public double update(double t, int indexerTicks, boolean launcherReady, boolean wantFire) {
        event = null;
        switch (state) {
            case IDLE:
                power = 0;
                if (wantFire) {
                    go(State.WAIT_READY, t);
                }
                break;
            case WAIT_READY:
                power = 0;
                if (!wantFire) {
                    go(State.IDLE, t);
                } else if (launcherReady && t - lastShotTime >= minIntervalSec) {
                    pocketStartTicks = indexerTicks;
                    if (!haveTarget) {                          // first shot: pockets are counted from here
                        targetTicks = indexerTicks;
                        haveTarget = true;
                    }
                    targetTicks += pocketTicks;                 // next pocket boundary - errors never add up
                    while (targetTicks - indexerTicks < pocketTicks / 2) {
                        targetTicks += pocketTicks;             // already (nearly) past it: aim for the next one
                    }
                    go(State.FEEDING, t);
                    power = feedPower;
                }
                break;
            case FEEDING:                                       // a pocket, once started, is always finished
                double remaining = targetTicks - indexerTicks;
                power = Math.max(minPower, Math.min(feedPower, feedPower * remaining / rampTicks));
                if (remaining <= STOP_TOLERANCE_TICKS) {
                    shots++;
                    lastShotTime = t;
                    event = "SHOT " + shots;
                    power = 0;
                    go(wantFire ? State.WAIT_READY : State.IDLE, t);
                } else if (t - stateStart > pocketTimeoutSec) {
                    jams++;
                    event = "JAM " + jams + " (windmill moved " + Math.abs(indexerTicks - pocketStartTicks)
                            + " of " + Math.round(pocketTicks) + " ticks)";
                    power = -unjamPower;
                    go(State.UNJAM, t);
                }
                break;
            case UNJAM:
                power = -unjamPower;
                if (t - stateStart >= unjamSec) {
                    power = 0;
                    if (jams >= maxJams) {
                        event = "JAMMED - gave up after " + jams + " tries";
                        go(State.JAMMED, t);
                    } else {
                        go(wantFire ? State.WAIT_READY : State.IDLE, t);
                    }
                }
                break;
            case JAMMED:
                power = 0;
                if (!wantFire) {                                // release FIRE to reset and try again
                    jams = 0;
                    go(State.IDLE, t);
                }
                break;
        }
        return power;
    }

    private void go(State s, double t) {
        state = s;
        stateStart = t;
    }

    /** Forget jams and go idle (e.g. the driver used the manual unjam). */
    public void reset() {
        state = State.IDLE;
        jams = 0;
        power = 0;
    }

    public State state() {
        return state;
    }

    public int shots() {
        return shots;
    }

    public int jams() {
        return jams;
    }

    public boolean feeding() {
        return state == State.FEEDING;
    }

    /** Event text from the last update (null if nothing happened). */
    public String event() {
        return event;
    }
}

package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * Fires one ball at a time: turn the windmill exactly one pocket, then wait until the flywheel is back at speed (and a
 * minimum gap has passed) before the next pocket. A pocket that doesn't finish in time is a jam: back off briefly and
 * retry the same pocket; after too many jams it stops and waits for the fire request to be released.
 * A windmill that stops moving during a pocket (the ramp's low power near the end can't push a ball past a tight spot)
 * gets more power, step by step, until it moves again. A pocket cut short (jam, unjam) is finished first; only a windmill
 * more than MAX_PUSH_POCKETS behind its next pocket goes to the first boundary ahead (no catch-up over many pockets).
 * Optional (TeleOp, Tune: skipEmptyPockets): a pocket that launched nothing (the flywheel stayed ready while it turned
 * and for emptyWaitSec after) needs no pause, so the next pocket starts at once. If the flywheel dips during a pocket
 * started that early (a late ball from the pocket before, or this pocket's own ball), the windmill stops at once and
 * waits for the flywheel and the pause before it finishes the pocket, so a second ball never follows a late one into a
 * slowed flywheel. If nothing dips after it starts again, the pause after that dip has already run: the next pocket
 * goes as soon as this one ends (and is watched the same way, in case this pocket's ball is late). FIRE let go during
 * such a hold stops the windmill there (the flywheel may stop too); the next FIRE finishes that pocket first.
 * No FTC SDK dependency, so it is unit-tested on the laptop.
 */
public class ShotFeeder {

    public enum State { IDLE, WAIT_READY, FEEDING, UNJAM, JAMMED }

    /** A pocket is done once the encoder reaches its boundary (slow approach: overshoot is small and never adds up). */
    static final double STOP_TOLERANCE_TICKS = 0;
    /** A new pocket boundary must be at least this far ahead (pockets), otherwise the one after it is used. */
    static final double MIN_PUSH_POCKETS = 0.25;
    /** One push never aims further ahead than this (pockets): further back (a long back-off or the driver's unjam), the
     *  windmill goes to the first boundary ahead and steps forward one pocket per push. A pocket turned back more than
     *  about one pocket can pass the intake and be reloaded (old logs: long pushes after an unjam made extra-deep, long
     *  flywheel dips = several balls). The old code aimed one pocket further at every jam (2-7 pocket pushes). */
    static final double MAX_PUSH_POCKETS = 2.0;
    /** No forward movement for this long while feeding = stuck (logs: 8 % of pockets stalled at 0.15-0.2 power). */
    static final double STUCK_SEC = 0.15;
    static final double STUCK_MIN_TICKS = 2;
    /** Stuck: the power goes up this much every STUCK_SEC until the windmill moves (at most the feed power) and keeps
     *  that extra for the rest of the pocket, fading out over the last STUCK_FADE_POCKETS, so it breaks free with just
     *  enough power and still reaches the boundary as slowly as the plain ramp. */
    static final double STUCK_POWER_STEP = 0.10;
    static final double STUCK_FADE_POCKETS = 0.25;
    /** The automatic back-off after a jam also stops once the windmill has turned back this far (pockets). The check
     *  runs once per loop, so with 60-76 ms loops and braking the real back-off is about twice this. */
    static final double UNJAM_MAX_POCKETS = 0.25;

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
    private boolean lastPocketDone = true;                      // false while a pocket is unfinished (fed, jam, unjam)
    private int progressTicks = 0;                              // stuck check: last forward progress while feeding
    private double progressTime = 0;
    private double lastStepUp = 0;
    private double extraPower = 0;                              // added to the ramp after the windmill got stuck
    private boolean stuckReported = false;
    private int unjamStartTicks = 0;
    private double lastShotTime = -1e9;                         // the pause runs from here (normally the pocket end)
    private double lastDoneTime = -1e9;                         // end of the last finished pocket
    private int shots = 0;
    private int jams = 0;
    private double power = 0;
    private String event = null;
    private boolean skipEmptyPause = false;                     // see skipEmptyPockets()
    private double emptyWaitSec = 0.10;
    private boolean dipSeen = true;                             // flywheel not ready since the current pocket started
    private boolean noPausePocket = false;                       // the current pocket started without the pause
    private boolean holding = false;                            // stopped mid-pocket: the flywheel dipped
    private double holdStart = 0;
    private boolean resumed = false;                            // this pocket was held and has started again
    private boolean dipAfterResume = false;
    private int noPausePockets = 0;

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

    /** Windmill power and pause between pockets as saved on the robot by "BioBuzz Tune" (the rest from constants). */
    public static ShotFeeder fromSettings(TeleOpSettings s) {
        return new ShotFeeder(BioBuzzConstants.INDEXER_TICKS_PER_POCKET, s.feedPower,
                BioBuzzConstants.INDEXER_UNJAM_POWER, s.feedPauseSec,
                BioBuzzConstants.SHOT_POCKET_TIMEOUT_SEC, BioBuzzConstants.SHOT_UNJAM_SEC, BioBuzzConstants.SHOT_MAX_JAMS);
    }

    /**
     * No pause after a pocket that launched nothing (TeleOp, Tune; AUTO keeps the plain pause). The 7 Nov logs: about 2
     * pockets per ball, and the pause after every pocket, empty or not, was 42 % of the time FIRE was held.
     * @param waitSec how long the flywheel must stay ready after a pocket before it counts as empty (most balls dip it
     *                while the pocket turns; a later dip during the next pocket stops the windmill, see the class doc)
     */
    public ShotFeeder skipEmptyPockets(boolean on, double waitSec) {
        skipEmptyPause = on;
        emptyWaitSec = waitSec;
        return this;
    }

    /**
     * @param t              time in seconds (any monotonic clock)
     * @param indexerTicks   windmill encoder
     * @param launcherReady  flywheel at speed
     * @param wantFire       driver holds FIRE / AUTO wants another shot
     * @return windmill power to apply this loop
     */
    public double update(double t, int indexerTicks, boolean launcherReady, boolean wantFire) {
        return update(t, indexerTicks, launcherReady, true, wantFire);
    }

    /**
     * @param flywheelReady  flywheel at speed; a ball's dip makes it false (the empty-pocket check uses only this)
     * @param gateOk         anything else the next pocket must wait for (TeleOp: the aim gate); not a sign of a ball
     */
    public double update(double t, int indexerTicks, boolean flywheelReady, boolean gateOk, boolean wantFire) {
        boolean launcherReady = flywheelReady && gateOk;
        if (!flywheelReady) {
            dipSeen = true;
            dipAfterResume = resumed;
        }
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
                    break;
                }
                boolean afterPause = launcherReady && t - lastShotTime >= minIntervalSec;
                boolean afterEmpty = skipEmptyPause && haveTarget && lastPocketDone && !dipSeen && launcherReady
                        && t - lastDoneTime >= emptyWaitSec;
                if (afterPause || afterEmpty) {
                    pocketStartTicks = indexerTicks;
                    if (!haveTarget) {                          // first shot: pockets are counted from here
                        targetTicks = indexerTicks;
                        haveTarget = true;
                        lastPocketDone = true;
                    }
                    targetTicks = nextTarget(indexerTicks);     // pocket boundaries stay on one grid: errors never add up
                    lastPocketDone = false;
                    progressTicks = indexerTicks;
                    progressTime = t;
                    lastStepUp = t;
                    extraPower = 0;                             // a retry steps up again: kept extra power overshoots
                    stuckReported = false;
                    noPausePocket = skipEmptyPause && t - lastDoneTime < minIntervalSec;   // a late ball may still come
                    holding = false;
                    resumed = false;
                    dipAfterResume = false;
                    dipSeen = !flywheelReady;
                    if (noPausePocket) {
                        noPausePockets++;
                        event = afterPause ? "NO PAUSE (pocket " + shots + "'s pause ran during its hold)"
                                : "NO PAUSE (pocket " + shots + " launched nothing)";
                    }
                    go(State.FEEDING, t);
                    power = rampPower(targetTicks - indexerTicks);
                }
                break;
            case FEEDING:                                       // a pocket, once started, is finished (held: see below)
                if (holding) {
                    power = 0;
                    if (!wantFire) {                            // FIRE let go while held: stop here, like a jam; the
                        holding = false;                        // next FIRE finishes this pocket after the pause and
                        noPausePocket = false;                  // the flywheel (which may be off now)
                        lastShotTime = Math.max(lastShotTime, holdStart);
                        event = "RELEASED during a hold - the next FIRE finishes the pocket";
                        go(State.IDLE, t);
                        break;
                    }
                    if (!launcherReady || t - holdStart < minIntervalSec) {
                        break;
                    }
                    holding = false;                            // finish the pocket as a normal one, from this loop on
                    noPausePocket = false;
                    resumed = true;
                    dipAfterResume = false;
                    stateStart += t - holdStart;                // the jam timeout doesn't count the hold
                    progressTicks = indexerTicks;
                    progressTime = t;
                    lastStepUp = t;
                }
                double remaining = targetTicks - indexerTicks;
                if (noPausePocket && !flywheelReady && remaining > STOP_TOLERANCE_TICKS) {
                    holding = true;                             // a ball reached the flywheel: stop, as after a shot
                    holdStart = t;
                    power = 0;
                    event = "HOLD (flywheel dipped during a no-pause pocket)";
                    break;
                }
                if (indexerTicks - progressTicks >= STUCK_MIN_TICKS) {
                    progressTicks = indexerTicks;
                    progressTime = t;
                } else if (t - Math.max(progressTime, lastStepUp) >= STUCK_SEC) {
                    lastStepUp = t;
                    extraPower = Math.min(extraPower + STUCK_POWER_STEP, feedPower);   // kept for the rest of this pocket
                    if (!stuckReported) {
                        event = "STUCK at " + Math.abs(indexerTicks - pocketStartTicks) + " of "
                                + Math.round(targetTicks - pocketStartTicks) + " ticks - more power";
                        stuckReported = true;
                    }
                }
                double ramp = rampPower(remaining);
                double fade = Math.max(0, Math.min(1, remaining / (STUCK_FADE_POCKETS * pocketTicks)));
                power = Math.min(Math.max(ramp, feedPower), ramp + extraPower * fade);
                if (remaining <= STOP_TOLERANCE_TICKS) {
                    shots++;
                    lastShotTime = resumed && !dipAfterResume ? holdStart : t;   // a held pocket's ball dipped at holdStart:
                    lastDoneTime = t;                                            // its pause already ran during the hold
                    lastPocketDone = true;
                    event = "SHOT " + shots;
                    power = 0;
                    go(wantFire ? State.WAIT_READY : State.IDLE, t);
                } else if (t - stateStart > pocketTimeoutSec) {         // even 1 tick short: its ball may not have left
                    jams++;
                    event = "JAM " + jams + " (windmill moved " + Math.abs(indexerTicks - pocketStartTicks)
                            + " of " + Math.round(targetTicks - pocketStartTicks) + " ticks)";
                    power = -unjamPower;
                    unjamStartTicks = indexerTicks;
                    go(State.UNJAM, t);
                }
                break;
            case UNJAM:
                power = -unjamPower;
                if (t - stateStart >= unjamSec || unjamStartTicks - indexerTicks >= UNJAM_MAX_POCKETS * pocketTicks) {
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

    /**
     * The pocket boundary to feed to: the same one again if the last pocket was cut short (jam, unjam) and its boundary
     * is still ahead, however close; otherwise the next one, at least MIN_PUSH_POCKETS ahead. Never more than
     * MAX_PUSH_POCKETS ahead: then the first boundary ahead of the windmill (no catch-up push over missed pockets).
     */
    private double nextTarget(int indexerTicks) {
        double next = targetTicks;
        if (lastPocketDone || next <= indexerTicks) {
            next += pocketTicks;
            while (next - indexerTicks < MIN_PUSH_POCKETS * pocketTicks) {
                next += pocketTicks;                            // already (nearly) past it: aim for the next one
            }
        }
        if (next - indexerTicks > MAX_PUSH_POCKETS * pocketTicks) {
            next = targetTicks + Math.floor((indexerTicks - targetTicks) / pocketTicks + 1) * pocketTicks;
        }
        return next;
    }

    /** Power that falls in proportion to the ticks left, so the windmill reaches the boundary slowly. */
    private double rampPower(double remaining) {
        return Math.max(minPower, Math.min(feedPower, feedPower * remaining / rampTicks));
    }

    private void go(State s, double t) {
        state = s;
        stateStart = t;
    }

    /** Forget jams and go idle (e.g. the driver used the manual unjam). A pocket cut short is finished next time. */
    public void reset() {
        if (holding) {
            lastShotTime = Math.max(lastShotTime, holdStart);   // the pause still runs from the dip
        }
        state = State.IDLE;
        jams = 0;
        power = 0;
        holding = false;
        noPausePocket = false;
    }

    /** Pockets started before the full pause after the last one (launched nothing, or its pause ran during a hold). */
    public int noPausePockets() {
        return noPausePockets;
    }

    /** Stopped mid-pocket, waiting for the flywheel (a dip during a no-pause pocket). */
    public boolean holding() {
        return holding;
    }

    /** Pocket boundary the windmill is fed to (encoder ticks; only meaningful once a pocket has been fed). */
    public double targetTicks() {
        return targetTicks;
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

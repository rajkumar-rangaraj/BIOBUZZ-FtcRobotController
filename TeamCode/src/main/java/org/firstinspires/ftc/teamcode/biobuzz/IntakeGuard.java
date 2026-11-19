package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * Indexer v2: the intake power after the G407 cap and the pair-jam unjam (TeleOp, Tune and the Pedro AUTO share it).
 * <p>
 * G407 SAFE PUSH-BACK: when the queue becomes FULL (QueueFeeder; a volley only lifts this once a ball has been counted
 * out, QueueFeeder.capacityFreed(), not when it starts: back-off, gate opening, a flywheel-dip hold) the intake reverses at pushbackPower for
 * pushbackSec (a 5th ball goes back out), then, while FULL and intake IN is still asked for, runs at fullPower (a
 * gentle reverse: a 5th ball is never pulled in). A reverse asked for passes through. QueueFeeder holds ball 4 on the
 * belts meanwhile. Both beams broken while not confirmed FULL (and no ball out yet) (a ball crossing ball 4's spot on its way up) only
 * pauses an IN request (power 0), as before.
 * <p>
 * PAIR JAM: two balls side by side can wedge in the funnel mouth. While intake IN is asked for and the queue is not
 * FULL, the intake current above jamAmps for jamSec = jam: reverse at unjamPower for unjamSec, then pull in again.
 * maxTries in a row without beam_staged / beam_full changing: give up (pull in, no more unjams) until the intake is
 * let go. The current is not in the bulk read: read it only when {@link #currentDue(double, double)} says so (pass NaN
 * otherwise). No FTC SDK dependency: unit-tested on the laptop (IntakeGuardTest).
 */
public class IntakeGuard {

    public enum State { NORMAL, CROSSING, PUSHBACK, FULL, UNJAM, GAVE_UP }

    /** Tunables (defaults from BioBuzzConstants); tests change single fields. */
    public static final class Settings {
        public double pushbackPower = BioBuzzConstants.QUEUE_FULL_PUSHBACK_POWER;
        public double pushbackSec = BioBuzzConstants.QUEUE_FULL_PUSHBACK_SEC;
        public double fullPower = BioBuzzConstants.QUEUE_FULL_INTAKE_POWER;
        public boolean jamDetect = BioBuzzConstants.INTAKE_JAM_DETECT;
        public double jamAmps = BioBuzzConstants.INTAKE_JAM_AMPS;
        public double jamSec = BioBuzzConstants.INTAKE_JAM_SEC;
        public double unjamPower = BioBuzzConstants.INTAKE_UNJAM_POWER;
        public double unjamSec = BioBuzzConstants.INTAKE_UNJAM_SEC;
        public int maxTries = BioBuzzConstants.INTAKE_UNJAM_MAX_TRIES;
        public double ampsPeriodSec = BioBuzzConstants.INTAKE_AMPS_PERIOD_SEC;
    }

    private final Settings s;
    private State state = State.NORMAL;
    private String event = null;

    private boolean wasFull = false;
    private double pushbackUntil = Double.NEGATIVE_INFINITY;
    private boolean watching = false;                           // the last update watched the current
    private double lastAmpsT = Double.NEGATIVE_INFINITY;
    private double overSince = Double.NaN;                      // current above jamAmps since
    private double unjamUntil = Double.NEGATIVE_INFINITY;
    private int tries = 0;                                      // unjams in a row without a beam change
    private boolean gaveUp = false;
    private boolean lastStaged = false;
    private boolean lastFull = false;
    private double lastAmps = Double.NaN;

    public IntakeGuard(Settings settings) {
        this.s = settings;
    }

    public static IntakeGuard standard() {
        return new IntakeGuard(new Settings());
    }

    /** Read the intake current this loop (the jam watch is on, intake IN asked, and the last read is old enough). */
    public boolean currentDue(double t, double requested) {
        return s.jamDetect && watching && requested > 0 && t - lastAmpsT >= s.ampsPeriodSec - 1e-9;
    }

    /**
     * Call once per loop.
     *
     * @param t            seconds, monotonic
     * @param requested    intake power the driver / AUTO asks for (positive = in)
     * @param queueFull    QueueFeeder.isFull() (4 held)
     * @param capacityFreed QueueFeeder.capacityFreed(): a ball has left this volley (the cap, the push-back and the
     *                     two-beam pause don't apply)
     * @param amps         intake current (A), NaN = not read this loop
     * @param stagedBroken beam_staged broken
     * @param fullBroken   beam_full broken
     * @return intake power for this loop
     */
    public double update(double t, double requested, boolean queueFull, boolean capacityFreed, double amps,
                         boolean stagedBroken, boolean fullBroken) {
        event = null;
        boolean beamsChanged = stagedBroken != lastStaged || fullBroken != lastFull;
        lastStaged = stagedBroken;
        lastFull = fullBroken;
        if (!Double.isNaN(amps)) {
            lastAmps = amps;
            lastAmpsT = t;
        }

        boolean full = queueFull && !capacityFreed;
        if (full && !wasFull) {
            pushbackUntil = t + s.pushbackSec;
            addEvent("PUSH-BACK (FULL: intake reverses " + s.pushbackSec + " s)");
        }
        wasFull = full;
        if (requested <= 0) {
            releaseJam();
        }
        if (full) {
            watching = false;
            overSince = Double.NaN;
            unjamUntil = Double.NEGATIVE_INFINITY;
            if (t < pushbackUntil - 1e-9) {
                state = State.PUSHBACK;
                return Math.min(requested, s.pushbackPower);
            }
            state = State.FULL;
            return requested > 0 ? s.fullPower : requested;
        }
        pushbackUntil = Double.NEGATIVE_INFINITY;
        if (!capacityFreed && stagedBroken && fullBroken) {             // a ball crossing ball 4's spot: pause
            watching = false;
            overSince = Double.NaN;
            unjamUntil = Double.NEGATIVE_INFINITY;
            state = State.CROSSING;
            return requested > 0 ? 0 : requested;
        }
        watching = !gaveUp;
        if (requested <= 0 || !s.jamDetect) {
            state = State.NORMAL;
            return requested;
        }
        if (beamsChanged) {
            tries = 0;                                          // a ball moved: the next jam is a new one
        }
        if (t < unjamUntil - 1e-9) {
            state = State.UNJAM;
            return s.unjamPower;
        }
        if (gaveUp) {
            state = State.GAVE_UP;
            return requested;
        }
        state = State.NORMAL;
        if (!Double.isNaN(amps)) {
            if (amps > s.jamAmps) {
                if (Double.isNaN(overSince)) {
                    overSince = t;
                } else if (t - overSince >= s.jamSec - 1e-9) {
                    overSince = Double.NaN;
                    if (tries >= s.maxTries) {
                        gaveUp = true;
                        watching = false;
                        state = State.GAVE_UP;
                        addEvent(String.format(java.util.Locale.US, "INTAKE JAMMED - gave up after %d unjams"
                                + " (%.1f A): let the intake go", tries, amps));
                        return requested;
                    }
                    tries++;
                    unjamUntil = t + s.unjamSec;
                    state = State.UNJAM;
                    addEvent(String.format(java.util.Locale.US, "INTAKE JAM %d/%d (%.1f A): reverse", tries,
                            s.maxTries, amps));
                    return s.unjamPower;
                }
            } else {
                overSince = Double.NaN;
            }
        }
        return requested;
    }

    private void releaseJam() {
        tries = 0;
        gaveUp = false;
        overSince = Double.NaN;
        unjamUntil = Double.NEGATIVE_INFINITY;
    }

    public State state() {
        return state;
    }

    /** Unjams in a row without a beam change. */
    public int tries() {
        return tries;
    }

    /** Gave up on a jam: no more unjams until the intake is let go. */
    public boolean gaveUp() {
        return gaveUp;
    }

    /** Last intake current read (A), NaN = never. */
    public double lastAmps() {
        return lastAmps;
    }

    /** Event text from the last update (null if nothing happened). */
    public String event() {
        return event;
    }

    /** One telemetry line. */
    public String status() {
        switch (state) {
            case PUSHBACK:
                return "PUSH-BACK (FULL, G407)";
            case FULL:
                return "FULL: no 5th ball (gentle reverse)";
            case CROSSING:
                return "paused (ball crossing beam_full)";
            case UNJAM:
                return "UNJAM " + tries + "/" + s.maxTries;
            case GAVE_UP:
                return "JAMMED - gave up, let the intake go";
            default:
                return Double.isNaN(lastAmps) ? "ok" : String.format(java.util.Locale.US, "ok (%.1f A)", lastAmps);
        }
    }

    private void addEvent(String e) {
        event = event == null ? e : event + "; " + e;
    }
}

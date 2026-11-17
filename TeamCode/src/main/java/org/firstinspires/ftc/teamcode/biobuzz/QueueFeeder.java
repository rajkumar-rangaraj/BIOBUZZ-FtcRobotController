package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * Indexer v2: the linear ball queue and its gate (replaces the 4-pocket windmill and ShotFeeder in TeleOp, Tune and
 * the Pedro AUTO).
 * <p>
 * The queue is a single-file lane at 40 deg that holds 4 balls: ball 1 "staged" against the gate paddle, ball 4 just
 * behind the intake roller. Belts below and a Gecko row above, all on the queue motor (positive power = toward the
 * flywheel). Two IR break beams: beam_staged (ball 1 at the gate) and beam_full (ball 4's spot). The gate is a servo
 * paddle that holds ball 1 when CLOSED.
 * <p>
 * LOADING (intake running, not firing): the queue runs at loadPower until beam_staged breaks, then stops. While the
 * intake keeps running it gives short packing pulses (off first, then on) so the balls behind come up, and ball 1 is
 * only pressed into the closed gate for a moment at a time. Both beams broken while the queue pushes forward for
 * fullConfirmSec = FULL (4 held: G407, the intake must not take a 5th ball; a ball that only passes the beam_full spot
 * on its way up moves off it sooner).
 * <p>
 * FIRE: wait for the flywheel to be ready ONCE (and the aim, TeleOp), open the gate, wait gateOpenSec, then run the
 * queue at firePower. No per-ball pause: the loop is 33-76 ms, so the spacing comes from the queue speed. The queue
 * only HOLDS (power 0) when the flywheel dips too deep (flywheelDipTooDeep) and resumes when it is ready again.
 * Balls are counted from beam_staged broken -> clear edges after the gate opened (debounced by queue travel: the queue
 * must move beamMinRunTicks while the beam is broken, so a ball flickering at the beam's edge while the queue stands
 * never counts). Touching balls can keep the beam broken, so the count can be low; it is never estimated from time or
 * the encoder. The volley ends on the EMPTY rule instead: beam_staged and beam_full clear for emptyClearSec while the
 * queue runs, and the queue moved emptyFlushTicks since (a ball resting between the beams, where none sees it, would
 * break beam_staged within that travel; the last ball reaches the flywheel).
 * <p>
 * JAM (gate open, queue commanded forward): the encoder moves less than jamMinTicks for jamSec (stalled), or
 * beam_staged stays broken while the queue travels jamBeamTicks (a ball stuck at the gate, belts slipping; touching
 * balls pass the beam within the whole queue's length). Then the queue reverses at unjamPower for unjamSec and tries
 * again; maxJams in a row (a counted ball resets the run) = JAMMED: stopped and reported until FIRE is let go.
 * <p>
 * THE GATE ONLY CLOSES when nothing can be in the paddle's path: the queue stops, backs off at least closeBackoffTicks
 * (a ball past beam_staged, where the beam can't see it, comes back through the beam) and until beam_staged is clear,
 * then stays stopped for a loop. That is how every volley ends (EMPTY, FIRE let go mid-volley, JAMMED let go); the
 * balls left are then re-staged by loading. If the queue can't back off clear in closeBackoffSec the gate stays open
 * (reported; TeleOp's B tries again).
 * <p>
 * An uncalibrated gate (GATE_OPEN / GATE_CLOSED negative) refuses to fire, like the flower arm. No FTC SDK
 * dependency: unit-tested on the laptop (QueueFeederTest).
 */
public class QueueFeeder {

    public enum State { IDLE, LOADING, FULL, WAIT_READY, OPENING, FIRING, UNJAM, CLOSING, EMPTY, JAMMED }

    /** Tunables (defaults from BioBuzzConstants); tests change single fields. */
    public static final class Settings {
        public double loadPower = BioBuzzConstants.QUEUE_LOAD_POWER;
        public double firePower = BioBuzzConstants.QUEUE_FIRE_POWER;
        public double unjamPower = BioBuzzConstants.QUEUE_UNJAM_POWER;
        public double unjamSec = BioBuzzConstants.QUEUE_UNJAM_SEC;
        public double backoffPower = BioBuzzConstants.QUEUE_BACKOFF_POWER;
        public double closeBackoffSec = BioBuzzConstants.QUEUE_CLOSE_BACKOFF_SEC;
        public double closeBackoffTicks = BioBuzzConstants.QUEUE_CLOSE_BACKOFF_TICKS;
        public double gateOpenSec = BioBuzzConstants.GATE_OPEN_SEC;
        public double emptyClearSec = BioBuzzConstants.QUEUE_EMPTY_CLEAR_SEC;
        public double emptyFlushTicks = BioBuzzConstants.QUEUE_EMPTY_FLUSH_TICKS;
        public double beamMinRunTicks = BioBuzzConstants.QUEUE_BEAM_MIN_RUN_TICKS;
        public double jamSec = BioBuzzConstants.QUEUE_JAM_SEC;
        public int jamMinTicks = BioBuzzConstants.QUEUE_JAM_MIN_TICKS;
        public double jamBeamTicks = BioBuzzConstants.QUEUE_JAM_BEAM_TICKS;
        public int maxJams = BioBuzzConstants.QUEUE_MAX_JAMS;
        public double packOnSec = BioBuzzConstants.QUEUE_PACK_ON_SEC;
        public double packOffSec = BioBuzzConstants.QUEUE_PACK_OFF_SEC;
        public double loadLingerSec = BioBuzzConstants.QUEUE_LOAD_LINGER_SEC;
        public double fullConfirmSec = BioBuzzConstants.QUEUE_FULL_CONFIRM_SEC;
        /** False = GATE_OPEN / GATE_CLOSED not calibrated: FIRE is refused. */
        public boolean gateCalibrated = true;
    }

    private final Settings s;

    private State state = State.IDLE;
    private double stateStart = 0;
    private double lastT = Double.NaN;
    private double power = 0;
    private double lastPower = 0;                               // commanded over the interval that just ended
    private boolean gateOpen = false;
    private double gateMovedAt = Double.NEGATIVE_INFINITY;
    private String event = null;

    private double lastIntakeT = Double.NEGATIVE_INFINITY;
    private double packStart = Double.NaN;
    private double fullConfirm = 0;
    private int notFullReadings = 0;
    private boolean full = false;
    private boolean refusedReported = false;

    private boolean inRun = false;                              // beam_staged: a broken run ...
    private double runStartTicks = 0;                           // ... the queue position when it began
    private boolean runForward = false;                         // ... and the queue's direction during that run
    private boolean runReverse = false;
    private double clearSec = 0;                                // EMPTY: both beams clear while the queue ran ...
    private double clearStartTicks = Double.NaN;                // ... and the queue position when they cleared
    private int prevTicks = 0;
    private boolean holding = false;
    private int progressTicks = 0;
    private double progressTime = 0;
    private double brokenStartTicks = Double.NaN;               // beam jam: queue position when the beam run began
    private double closeStart = 0;
    private double backoffStartTicks = Double.NaN;              // closing: queue position when the back-off began
    private State closeNext = State.IDLE;
    private boolean blockedReported = false;

    private int balls = 0;
    private int volleyBalls = 0;
    private int volleys = 0;
    private int jams = 0;
    private int jamsInRow = 0;
    private boolean emptied = false;

    public QueueFeeder(Settings settings) {
        this.s = settings;
    }

    /** Robot settings: BioBuzzConstants plus the gate calibration check. */
    public static QueueFeeder standard() {
        Settings st = new Settings();
        st.gateCalibrated = gateCalibrated(BioBuzzConstants.GATE_OPEN, BioBuzzConstants.GATE_CLOSED);
        return new QueueFeeder(st);
    }

    /** Fire power as saved on the robot by "BioBuzz Tune" (the rest from constants). */
    public static QueueFeeder fromSettings(TeleOpSettings settings) {
        QueueFeeder f = standard();
        f.s.firePower = settings.queueFirePower;
        return f;
    }

    /** BioBuzz Tune: a new fire power, used from the next update. */
    public void setFirePower(double firePower) {
        s.firePower = firePower;
    }

    /** Both gate positions set (0..1) and different. */
    public static boolean gateCalibrated(double open, double closed) {
        return open >= 0 && open <= 1 && closed >= 0 && closed <= 1 && open != closed;
    }

    /**
     * Call once per loop.
     *
     * @param t                  seconds, monotonic
     * @param wantFire           the driver holds FIRE / AUTO wants the volley
     * @param aimOk              TeleOp's aim gate; only checked before the gate opens (true in AUTO)
     * @param intakeIn           the intake is pulling balls in
     * @param stagedBroken       beam_staged broken (ball 1 at the gate)
     * @param fullBroken         beam_full broken (a ball at ball 4's spot)
     * @param flywheelReady      the flywheel is at speed (LauncherGate)
     * @param flywheelDipTooDeep the flywheel is too far below its target (or off): hold the queue
     * @param queueTicks         queue motor encoder
     * @return queue power for this loop (the gate: {@link #gateOpen()})
     */
    public double update(double t, boolean wantFire, boolean aimOk, boolean intakeIn, boolean stagedBroken,
                         boolean fullBroken, boolean flywheelReady, boolean flywheelDipTooDeep, int queueTicks) {
        double dt = Double.isNaN(lastT) ? 0 : Math.max(0, t - lastT);
        lastT = t;
        lastPower = power;
        event = null;
        if (intakeIn) {
            lastIntakeT = t;
        }
        countBalls(stagedBroken, queueTicks);
        trackFull(dt, stagedBroken, fullBroken);
        if (!wantFire) {
            refusedReported = false;
        }

        switch (state) {
            case IDLE:
            case LOADING:
            case FULL:
                if (wantFire && tryStartVolley(t)) {
                    waitReady(t, wantFire, aimOk, stagedBroken, flywheelReady, flywheelDipTooDeep, queueTicks);
                } else {
                    loading(t, stagedBroken);
                }
                break;
            case WAIT_READY:
                waitReady(t, wantFire, aimOk, stagedBroken, flywheelReady, flywheelDipTooDeep, queueTicks);
                break;
            case OPENING:
                power = 0;
                if (!wantFire) {
                    addEvent("FIRE released while the gate opened");
                    beginClosing(t, State.IDLE);
                    closing(t, wantFire, stagedBroken, flywheelReady, flywheelDipTooDeep, queueTicks);
                } else if (t - gateMovedAt >= s.gateOpenSec) {
                    startFiring(t, queueTicks);
                    firing(t, dt, wantFire, stagedBroken, fullBroken, flywheelReady, flywheelDipTooDeep, queueTicks);
                }
                break;
            case FIRING:
                firing(t, dt, wantFire, stagedBroken, fullBroken, flywheelReady, flywheelDipTooDeep, queueTicks);
                break;
            case UNJAM:
                power = -s.unjamPower;
                if (!wantFire) {
                    power = 0;
                    addEvent("FIRE released during an unjam");
                    beginClosing(t, State.IDLE);
                } else if (t - stateStart >= s.unjamSec) {
                    power = 0;
                    if (jamsInRow >= s.maxJams) {
                        addEvent("JAMMED - gave up after " + jamsInRow + " jams in a row (" + volleyBalls
                                + " balls counted): let FIRE go, clear the queue (B)");
                        go(State.JAMMED, t);
                    } else {
                        startFiring(t, queueTicks);             // retry; the queue runs from the next loop
                    }
                }
                break;
            case JAMMED:
                power = 0;                                      // the gate stays open: a ball may be in its path
                if (!wantFire) {
                    jamsInRow = 0;
                    beginClosing(t, State.IDLE);
                    closing(t, false, stagedBroken, flywheelReady, flywheelDipTooDeep, queueTicks);
                }
                break;
            case CLOSING:
                closing(t, wantFire, stagedBroken, flywheelReady, flywheelDipTooDeep, queueTicks);
                break;
            case EMPTY:
                power = 0;
                if (!wantFire) {
                    go(State.IDLE, t);
                    loading(t, stagedBroken);
                } else if (stagedBroken) {                      // FIRE still held and a new ball came up: next volley
                    startVolley(t);
                    waitReady(t, true, aimOk, stagedBroken, flywheelReady, flywheelDipTooDeep, queueTicks);
                } else {
                    power = loadingWanted(t) ? s.loadPower : 0; // gate closed: bring new balls up to it
                }
                break;
        }
        prevTicks = queueTicks;
        return power;
    }

    // ---------------- loading ----------------

    private boolean loadingWanted(double t) {
        return t - lastIntakeT <= s.loadLingerSec;
    }

    private void loading(double t, boolean stagedBroken) {
        if (full) {
            power = 0;
            packStart = Double.NaN;
            if (state != State.FULL) {
                addEvent("FULL (" + BioBuzzConstants.QUEUE_CAPACITY + " held)");
            }
            go(State.FULL, t);
            return;
        }
        if (!loadingWanted(t)) {
            power = 0;
            packStart = Double.NaN;
            go(State.IDLE, t);
            return;
        }
        go(State.LOADING, t);
        if (!stagedBroken) {
            power = s.loadPower;                                // bring ball 1 up to the gate
            packStart = Double.NaN;
            return;
        }
        if (s.packOnSec <= 0) {
            power = 0;
            return;
        }
        if (Double.isNaN(packStart)) {
            packStart = t;                                      // ball 1 staged: stop now, pulses after the off time
        }
        double phase = (t - packStart) % (s.packOffSec + s.packOnSec);
        power = phase >= s.packOffSec ? s.loadPower : 0;
    }

    private void trackFull(double dt, boolean stagedBroken, boolean fullBroken) {
        if (stagedBroken && fullBroken) {
            notFullReadings = 0;
            if (lastPower > 0) {
                fullConfirm += dt;
            }
            if (!full && fullConfirm >= s.fullConfirmSec) {
                full = true;
            }
        } else {
            fullConfirm = 0;
            if (++notFullReadings >= 2) {                       // one odd reading doesn't end FULL
                full = false;
            }
        }
    }

    // ---------------- firing ----------------

    private boolean tryStartVolley(double t) {
        if (!s.gateCalibrated) {
            if (!refusedReported) {
                addEvent("REFUSED: gate not calibrated (GATE_OPEN / GATE_CLOSED) - run BioBuzz Calibration");
                refusedReported = true;
            }
            return false;
        }
        startVolley(t);
        return true;
    }

    private void startVolley(double t) {
        volleys++;
        volleyBalls = 0;
        jamsInRow = 0;
        emptied = false;
        holding = false;
        packStart = Double.NaN;
        go(State.WAIT_READY, t);
    }

    private void waitReady(double t, boolean wantFire, boolean aimOk, boolean stagedBroken, boolean flywheelReady,
                           boolean flywheelDipTooDeep, int queueTicks) {
        if (!wantFire) {
            go(State.IDLE, t);
            loading(t, stagedBroken);
            return;
        }
        if (flywheelReady && !flywheelDipTooDeep && aimOk) {
            power = 0;
            gateOpen = true;
            gateMovedAt = t;
            addEvent("GATE OPEN (volley " + volleys + ", ball 1 " + (stagedBroken ? "staged" : "not at the gate") + ")");
            go(State.OPENING, t);
            return;
        }
        power = stagedBroken ? 0 : s.loadPower;                 // gate closed: bring ball 1 up meanwhile
    }

    private void startFiring(double t, int queueTicks) {
        go(State.FIRING, t);
        holding = false;
        clearSec = 0;
        clearStartTicks = Double.NaN;
        progressTicks = queueTicks;
        progressTime = t;
        brokenStartTicks = Double.NaN;
        power = 0;
    }

    private void firing(double t, double dt, boolean wantFire, boolean stagedBroken, boolean fullBroken,
                        boolean flywheelReady, boolean flywheelDipTooDeep, int queueTicks) {
        if (!wantFire) {
            power = 0;
            addEvent("RELEASED mid-volley after " + volleyBalls + " balls");
            beginClosing(t, State.IDLE);
            return;
        }
        if (holding) {
            if (!flywheelReady || flywheelDipTooDeep) {
                power = 0;
                return;
            }
            holding = false;
            addEvent("RESUME (flywheel ready)");
            progressTicks = queueTicks;
            progressTime = t;
            brokenStartTicks = stagedBroken ? queueTicks : Double.NaN;
        } else if (flywheelDipTooDeep) {
            holding = true;
            power = 0;
            addEvent("HOLD (flywheel dipped too deep) after " + volleyBalls + " balls");
            return;
        }
        if (!stagedBroken && !fullBroken) {
            if (lastPower > 0 && queueTicks != prevTicks) {
                clearSec += dt;                                 // only time the queue really ran (and moved) counts
                if (Double.isNaN(clearStartTicks)) {
                    clearStartTicks = prevTicks;
                }
            }
        } else {
            clearSec = 0;
            clearStartTicks = Double.NaN;
        }
        if (clearSec >= s.emptyClearSec - 1e-9 && queueTicks - clearStartTicks >= s.emptyFlushTicks) {
            power = 0;
            emptied = true;
            addEvent("EMPTY after " + volleyBalls + " balls counted");
            beginClosing(t, State.EMPTY);
            return;
        }
        if (queueTicks - progressTicks >= s.jamMinTicks) {
            progressTicks = queueTicks;
            progressTime = t;
        } else if (t - progressTime > s.jamSec) {
            jam(t, "queue stalled: encoder moved " + (queueTicks - progressTicks) + " ticks in "
                    + String.format(java.util.Locale.US, "%.2f", t - progressTime) + " s");
            return;
        }
        if (stagedBroken) {
            if (Double.isNaN(brokenStartTicks)) {
                brokenStartTicks = queueTicks;
            } else if (queueTicks - brokenStartTicks > s.jamBeamTicks) {
                jam(t, "ball stuck at the gate: beam_staged broken for " + Math.round(queueTicks - brokenStartTicks)
                        + " ticks of queue travel");
                return;
            }
        } else {
            brokenStartTicks = Double.NaN;
        }
        power = s.firePower;
    }

    private void jam(double t, String why) {
        jams++;
        jamsInRow++;
        power = -s.unjamPower;
        addEvent("JAM " + jams + " (" + why + (jamsInRow > 1 ? ", " + jamsInRow + " in a row" : "") + ")");
        go(State.UNJAM, t);
    }

    private void countBalls(boolean stagedBroken, int queueTicks) {
        if (stagedBroken) {
            if (!inRun) {
                inRun = true;
                runStartTicks = queueTicks;
                runForward = false;
                runReverse = false;
            }
            if (lastPower > 0) {
                runForward = true;
            } else if (lastPower < 0) {
                runReverse = true;
            }
            return;
        }
        // Debounced by queue travel: the run counts only if the queue moved beamMinRunTicks while the beam was broken,
        // so a ball resting at the beam's edge and flickering (a hold, a stop) never counts. A ball blocks the beam for
        // only ~0.1 s at fire speed, which can be a single reading at 60-76 ms loops, so one reading can be a ball.
        if (inRun) {
            if (gateOpen && runForward && !runReverse && queueTicks - runStartTicks >= s.beamMinRunTicks) {
                balls++;
                volleyBalls++;
                jamsInRow = 0;
                addEvent("BALL " + volleyBalls + " (beam_staged cleared)");
            }
            inRun = false;
        }
    }

    // ---------------- closing ----------------

    private void beginClosing(double t, State next) {
        holding = false;
        closeStart = t;
        backoffStartTicks = Double.NaN;
        closeNext = next;
        blockedReported = false;
        go(State.CLOSING, t);
    }

    private void closing(double t, boolean wantFire, boolean stagedBroken, boolean flywheelReady,
                         boolean flywheelDipTooDeep, int queueTicks) {
        if (gateOpen) {
            if (wantFire && closeNext == State.IDLE) {          // FIRE pressed again before the gate closed: go on
                startFiring(t, queueTicks);
                holding = !flywheelReady || flywheelDipTooDeep;
                if (!holding) {
                    power = s.firePower;
                }
                addEvent("FIRE again before the gate closed");
                return;
            }
            if (lastPower > 0) {
                power = 0;                                      // stop first
                return;
            }
            if (Double.isNaN(backoffStartTicks)) {
                backoffStartTicks = queueTicks;
            }
            // Back off: a ball past beam_staged (the beam can't see it there) comes back through the beam; keep going
            // until the beam is clear too. Only then is nothing in the paddle's path.
            boolean backedOff = backoffStartTicks - queueTicks >= s.closeBackoffTicks;
            if (!backedOff || stagedBroken) {
                if (t - closeStart < s.closeBackoffSec) {
                    power = -s.backoffPower;
                } else {
                    power = 0;
                    if (!blockedReported) {
                        addEvent("GATE CAN'T CLOSE: " + (stagedBroken ? "a ball is at the gate" : "the queue didn't back"
                                + " off (encoder?)") + " - B backs the queue off and tries again");
                        blockedReported = true;
                    }
                }
                return;
            }
            power = 0;
            if (lastPower == 0) {
                closeGate(t);                                   // backed off, stopped for a loop, beam clear
            }
            return;
        }
        power = 0;
        if (t - gateMovedAt >= s.gateOpenSec) {
            go(closeNext, t);
            if (closeNext == State.IDLE) {
                loading(t, stagedBroken);
            }
        }
    }

    private void closeGate(double t) {
        gateOpen = false;
        gateMovedAt = t;
        addEvent("GATE CLOSED");
    }

    private void go(State next, double t) {
        if (next != state) {
            stateStart = t;
        }
        state = next;
    }

    private void addEvent(String e) {
        event = event == null ? e : event + "; " + e;
    }

    /**
     * Forget jams and stop (the driver's manual unjam). An open gate is closed the safe way on the next updates
     * (queue stopped, beam_staged clear).
     */
    public void reset() {
        jams = 0;
        jamsInRow = 0;
        holding = false;
        power = 0;
        packStart = Double.NaN;
        if (gateOpen) {
            beginClosing(Double.isNaN(lastT) ? 0 : lastT, State.IDLE);
        } else {
            state = State.IDLE;
        }
    }

    public State state() {
        return state;
    }

    public double power() {
        return power;
    }

    /** Gate command: true = OPEN. Starts closed. */
    public boolean gateOpen() {
        return gateOpen;
    }

    /** 4 balls held (both beams broken, packed): the intake must not take a 5th (G407). */
    public boolean isFull() {
        return full;
    }

    /** The last volley ended on the EMPTY rule (AUTO: the pre-loads are gone). Cleared when a volley starts. */
    public boolean emptied() {
        return emptied;
    }

    /** Gate open or opening, or the queue feeding / unjamming with it open. */
    public boolean firing() {
        return state == State.OPENING || state == State.FIRING || state == State.UNJAM;
    }

    /** Stopped mid-volley: the flywheel dipped too deep. */
    public boolean holding() {
        return holding;
    }

    /** Balls counted (beam_staged edges) since this feeder was made. */
    public int balls() {
        return balls;
    }

    /** Balls counted in the current / last volley (can be low: touching balls may pass as one). */
    public int volleyBalls() {
        return volleyBalls;
    }

    public int volleys() {
        return volleys;
    }

    /** All jams since the last reset(). */
    public int jams() {
        return jams;
    }

    /** Event text from the last update (null if nothing happened; several are joined with "; "). */
    public String event() {
        return event;
    }
}

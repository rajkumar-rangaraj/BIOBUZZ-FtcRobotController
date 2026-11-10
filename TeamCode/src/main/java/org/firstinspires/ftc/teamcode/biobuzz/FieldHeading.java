package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * Field-centric driving (TeleOp): the robot's heading relative to the DRIVERS, 0 = the intake points away from them,
 * counter-clockwise positive. Two sensors, each with its own offset to that heading: the Pinpoint (leads) and the
 * Control Hub IMU (read right after the Pinpoint, so the two agree while turning).
 * <ul>
 *   <li>The IMU yaw flips by 180 deg, mostly where it crosses +-90 (diag logs 30 Oct - 9 Nov: ~1290 flips, each one the
 *       wheels' turn +-180). An IMU reading is taken as is or flipped: whichever fits its own track, else whichever is
 *       nearer the Pinpoint (or the predicted heading).</li>
 *   <li>Both sensors agree (within AGREE_DEG): that is the heading, however fast it moved (a hard hit, or a turn during
 *       a loop stall).</li>
 *   <li>Otherwise a reading must be a move the robot can make since that sensor's last good reading (looking back at
 *       most GATE_DT_CAP_SEC: after a loop stall the check would pass anything). A bigger jump is a glitch: that sensor
 *       is off its track. A sensor leading on its own must also read where the robot can get from the predicted heading
 *       (last turn rate, MAX_ACCEL_DPS2): a bad reading can pass the rate check from standing still. If the new reading
 *       only fits without its last one, that last one was bad: it is dropped.</li>
 *   <li>Both fit their tracks but disagree: if both are in reach of the predicted heading, the one nearer it leads,
 *       unless the IMU repeats its last reading exactly (frozen: at the start of a turn it would be nearer); else the
 *       one in reach (e.g. not a Pinpoint restart); neither: the Pinpoint. While the two agree, the IMU follows the
 *       Pinpoint by at most NUDGE_DEG a loop (bad Pinpoint readings within the agree window can't drag it); an IMU that
 *       disagrees two loops in a row is lined up with the Pinpoint.</li>
 *   <li>A sensor off its track is back when it agrees with the other one again or, if the other one isn't on its track,
 *       when it reads near the predicted heading (a restarted sensor moves on smoothly: only very near). If it moves
 *       on smoothly from its jump for rebaseSec instead, it restarted: it is lined up with the current heading, after
 *       adding the turn it saw meanwhile if no sensor gave the heading.</li>
 *   <li>A sensor that comes back after a gap (no reading in the last update) rejoins at the current heading.</li>
 *   <li>Reset turns the whole frame: a glitch in progress is sorted out the same way afterwards.</li>
 * </ul>
 * Limits (FieldHeadingSimTest: 400 runs per fault): a Pinpoint restart small enough to pass as a turn can stay until
 * Options (up to ~10 deg). With the IMU alone, a bad reading that looks like a possible move can leave up to ~20 deg
 * (the heading waits 0.3 s meanwhile). No FTC SDK dependency: unit-tested (FieldHeadingTest, FieldHeadingSimTest).
 */
public class FieldHeading {

    public enum Source { PINPOINT, IMU, NONE }

    static final double AGREE_DEG = 10;              // the two sensors agree (read back to back)
    static final double GATE_DT_CAP_SEC = 0.25;      // rate checks look back at most this long
    static final double MAX_ACCEL_DPS2 = 3000;       // turn acceleration (diag logs, wheel encoders: 99.9 % < 2620)
    static final double BACK_DT_CAP_SEC = 0.1;       // a sensor that moved on smoothly is back only this near
    static final double NUDGE_DEG = 1;               // while both agree, the IMU follows the Pinpoint this much a loop
    static final double FLIP_EVENT_SEC = 1;          // IMU flips are logged at most this often

    /** One heading sensor. */
    private static final class Track {
        final String name;
        final boolean flips;                         // its readings may be 180 deg off (the IMU)
        double off = Double.NaN;                     // driver heading = reading + off (NaN: joins at the next reading)
        double good, goodT;                          // its last reading on its track, and when
        double prev, prevT;                          // its last reading, on its track or not, and when
        double offSince = Double.NaN;                // off its track since (NaN: on it) ...
        double levelSince;                           // ... and moving on smoothly from its last jump since ...
        double levelTurn;                            // ... turning this much meanwhile while nobody gave the heading
        boolean had = false;                         // had a reading in the last update

        Track(String name, boolean flips) {
            this.name = name;
            this.flips = flips;
        }

        boolean onTrack() {
            return Double.isNaN(offSince);
        }
    }

    private final double maxRateDps;
    private final double marginDeg;
    private final double rebaseSec;
    private final Track pin = new Track("Pinpoint", false);
    private final Track imu = new Track("IMU", true);

    private double startDeg;                         // heading relative to the drivers when the first reading comes
    private boolean started = false;
    private double heading = Double.NaN;
    private double headingT;                         // when a sensor last gave the heading
    private double rateDps = 0;                      // the heading's last turn rate (predictions) ...
    private double rateDt = 0;                       // ... over this interval ...
    private boolean rateKnown = false;               // ... measured (not just after a start or a rejoin)
    private double beforeHeading = Double.NaN, beforeT, beforeRate, beforeRateDt;   // the same before the last update
    private boolean beforeRateKnown = false;         // that gave the heading (to drop that one if it was bad)
    private Source lastLeader = Source.NONE;         // the sensor that gave the heading in the last update
    private boolean imuDisagreed = false;            // the IMU fit its track but not the Pinpoint's, last update
    private Source source = Source.NONE;
    private int glitches = 0;
    private int imuFlips = 0;
    private double flipEventT = Double.NEGATIVE_INFINITY;
    private String event = null;

    /**
     * @param startDeg   the robot's heading relative to the drivers at the first reading (0 = intake away from them)
     * @param maxRateDps fastest real turn (deg/s); a faster change between two readings is a glitch
     * @param marginDeg  added to maxRate x dt (reading noise and loop jitter)
     * @param rebaseSec  a jump that lasts this long is a sensor restart, not a glitch
     */
    public FieldHeading(double startDeg, double maxRateDps, double marginDeg, double rebaseSec) {
        this.startDeg = startDeg;
        this.maxRateDps = maxRateDps;
        this.marginDeg = marginDeg;
        this.rebaseSec = rebaseSec;
    }

    public static FieldHeading standard(double startDeg) {
        return new FieldHeading(startDeg, BioBuzzConstants.HEADING_MAX_RATE_DPS, BioBuzzConstants.HEADING_GLITCH_MARGIN_DEG,
                BioBuzzConstants.HEADING_REBASE_SEC);
    }

    /**
     * Once per loop.
     *
     * @param pinpointDeg the Pinpoint heading (CCW +), NaN when it has no good reading
     * @param imuDeg      the IMU heading (CCW +), NaN when not read
     * @return the heading relative to the drivers, degrees in (-180, 180]; NaN while there is no reading (drive
     *         robot-centric then)
     */
    public double update(double t, double pinpointDeg, double imuDeg) {
        event = null;
        boolean pinIn = !Double.isNaN(pinpointDeg), imuIn = !Double.isNaN(imuDeg);
        boolean pinJoins = pinIn && (!pin.had || Double.isNaN(pin.off));    // the first reading, or back after a gap
        boolean imuJoins = imuIn && (!imu.had || Double.isNaN(imu.off));    // (it may have restarted meanwhile)
        pin.had = pinIn;
        imu.had = imuIn;
        if (!pinIn && !imuIn) {
            if (source != Source.NONE) {
                addEvent("no heading reading (Pinpoint and IMU): robot-centric until one comes back");
            }
            source = Source.NONE;
            return Double.NaN;                               // heading() keeps the last one for when a sensor returns
        }
        if (!started) {                                      // the very first reading: the start heading
            started = true;
            heading = MecanumMath.normalizeDegrees(startDeg);
            headingT = t;
        }
        boolean pinT = pinIn && !pinJoins, imuT = imuIn && !imuJoins;      // judged against their tracks
        boolean pinWasOn = pinT && pin.onTrack(), imuWasOn = imuT && imu.onTrack();
        double pred = predicted(t);

        // Each reading against its own sensor's track; the IMU's as is or flipped.
        double p = pinT ? MecanumMath.normalizeDegrees(pinpointDeg + pin.off) : Double.NaN;
        boolean pinFits = pinWasOn && fits(pin, pinpointDeg, t);
        double since = t - headingT;                         // a move the robot can make from the heading (any wait)
        boolean pinPossible = pinWasOn && dist(p, MecanumMath.normalizeDegrees(heading + rateDps * since))
                <= (rateKnown ? envelope(since) : maxRateDps * since + marginDeg);
        double iAs = imuT ? MecanumMath.normalizeDegrees(imuDeg + imu.off) : Double.NaN;
        double iFlip = MecanumMath.normalizeDegrees(iAs + 180);
        boolean asFits = imuWasOn && fits(imu, imuDeg, t);
        boolean flipFits = imuWasOn && fits(imu, imuDeg + 180, t);
        boolean flip;
        if (asFits != flipFits) {
            flip = flipFits;
        } else {                                             // neither or both fit: the one nearer the Pinpoint/prediction
            double ref = pinFits ? p : pred;
            flip = dist(iFlip, ref) < dist(iAs, ref);
        }
        boolean imuFits = flip ? flipFits : asFits;
        double i = flip ? iFlip : iAs;

        boolean pinLeads = false, imuLeads = false;
        // An IMU reading its own track rules out (the other one fits) agrees only with a Pinpoint move the robot can
        // make: else a bad Pinpoint reading that happens to match it would be followed (simulation, 10 Nov).
        boolean agreeAs = pinT && imuT && dist(p, iAs) <= AGREE_DEG && (asFits || !flipFits || pinPossible);
        boolean agreeFlip = pinT && imuT && dist(p, iFlip) <= AGREE_DEG && (flipFits || !asFits || pinPossible);
        if (agreeAs || agreeFlip) {                          // both agree: that's the heading, however fast it moved
            if (pinWasOn && !pinFits) {
                addEvent(String.format("fast turn seen by both sensors (%.0f deg): followed",
                        MecanumMath.normalizeDegrees(p - heading)));
            }
            if (!agreeAs) {
                imuFlipped(t);
            }
            pinLeads = true;
            imuDisagreed = false;
            setHeading(p, t);
            keep(pin, pinpointDeg, t);
            double flipBy = agreeAs ? 0 : 180;               // the IMU follows the Pinpoint at most NUDGE_DEG a loop: a
            double miss = MecanumMath.normalizeDegrees(p - (imuDeg + flipBy + imu.off));     // bad Pinpoint reading
            imu.off = MecanumMath.normalizeDegrees(imu.off + flipBy                         // that agrees can't drag
                    + Math.max(-NUDGE_DEG, Math.min(NUDGE_DEG, miss)));                    // it (sim 10 Nov)
            keep(imu, imuDeg, t);
        } else {
            boolean pinOk = pinFits, imuOk = imuFits;
            if (pinT && !pinWasOn && !imuOk && back(pin, pinpointDeg, p, pred, t)) {
                pinOk = true;                                // back from a glitch (no IMU on its track to agree with)
            }
            if (imuT && !imuWasOn && !pinOk && back(imu, imuDeg, i, pred, t)) {
                imuOk = true;
            }
            double reach = rateKnown ? envelope(Math.min(t - headingT, GATE_DT_CAP_SEC)) : gate(t - headingT);
            boolean pinNear = dist(p, pred) <= reach, imuNear = dist(i, pred) <= reach;
            if (pinOk && imuOk) {                            // both possible, but they disagree
                // Both in reach: the one nearer the prediction, unless the IMU repeats its last reading exactly (frozen:
                // at the start of a turn it would be nearer). Else the one in reach; neither: the Pinpoint.
                boolean pinWins = pinNear && imuNear ? imuDeg == imu.prev || dist(p, pred) <= dist(i, pred)
                        : pinNear || !imuNear;
                if (!pinWins) {
                    pinOk = false;                           // e.g. a Pinpoint restart
                    glitches++;
                    addEvent(String.format("Pinpoint heading %.0f deg off the IMU: ignored", dist(p, i)));
                    offTrack(pin, t);
                }
            } else if (pinOk) {                              // alone: only where the robot can get from the prediction
                pinOk = pinNear || lastLeader != Source.PINPOINT || lastWasOff(p, "Pinpoint", t);   // (a bad reading
            } else if (imuOk) {                              // within the rate check, sim 10 Nov); not when taking over:
                imuOk = imuNear || lastLeader != Source.IMU || lastWasOff(i, "IMU", t);   // the prediction came from
            }                                                // the other sensor, maybe from a bad reading
            if (pinWasOn && !pinOk && pin.onTrack()) {
                glitch(pin, pinpointDeg, t);
            }
            if (imuWasOn && !imuOk) {
                glitch(imu, imuDeg, t);
                imuDisagreed = false;
            }
            if (pinOk) {
                pinLeads = true;
                setHeading(p, t);
                keep(pin, pinpointDeg, t);
                if (imuOk) {                                 // the IMU fits its own track but not the Pinpoint
                    if (flip) {
                        imu.off = MecanumMath.normalizeDegrees(imu.off + 180);
                        imuFlipped(t);
                    }
                    if (imuDisagreed) {
                        lineUp(imu, imuDeg, t);              // again: it's the IMU that is off (e.g. frozen)
                    } else {
                        keep(imu, imuDeg, t);                // once: maybe a bad Pinpoint reading (sim 10 Nov)
                    }
                    imuDisagreed = true;
                }
            } else if (imuOk) {                              // the Pinpoint is missing or off its track: the IMU leads
                imuLeads = true;
                imuDisagreed = false;
                if (flip) {
                    imu.off = MecanumMath.normalizeDegrees(imu.off + 180);
                    imuFlipped(t);
                }
                setHeading(i, t);
                keep(imu, imuDeg, t);
            }                                                // else nobody on its track: keep the last good heading
        }
        boolean held = !pinLeads && !imuLeads;
        boolean pinBack = settle(pin, pinT, pinpointDeg, t, held);
        settle(imu, imuT, imuDeg, t, held && !pinBack);      // (the Pinpoint already carried this turn)
        if (pinJoins) {                                      // after this update's heading, so they line up with it
            lineUp(pin, pinpointDeg, t);
        }
        if (imuJoins) {
            lineUp(imu, imuDeg, t);
        }
        if (held && !pinBack && (pinJoins || imuJoins)) {
            rateDps = 0;                                     // a sensor (re)joined at a kept heading: no turn known
            rateDt = 0;
            rateKnown = false;
            headingT = t;
        }
        if (pinLeads || pinJoins || pinBack) {
            use(Source.PINPOINT);
        } else if (imuLeads || imuJoins) {
            use(Source.IMU);
        }
        lastLeader = pinLeads ? Source.PINPOINT : imuLeads ? Source.IMU : Source.NONE;
        if (pinIn) {
            pin.prev = pinpointDeg;
            pin.prevT = t;
        }
        if (imuIn) {
            imu.prev = imuDeg;
            imu.prevT = t;
        }
        return heading;
    }

    /** The driver pointed the intake away from them and pressed reset: that is heading 0 from now on. */
    public void reset() {
        if (!started) {
            startDeg = 0;
        } else {                                             // turn the whole frame; glitch tracking carries on
            pin.off = MecanumMath.normalizeDegrees(pin.off - heading);      // (NaN stays NaN: joins at 0 later)
            imu.off = MecanumMath.normalizeDegrees(imu.off - heading);
            heading = 0;
        }
        event = "reset: intake away from the drivers";
    }

    public double heading() {
        return heading;
    }

    public Source source() {
        return source;
    }

    public int glitches() {
        return glitches;
    }

    /** IMU readings taken flipped by 180 deg so far. */
    public int imuFlips() {
        return imuFlips;
    }

    /** What happened in the last update (glitch, sensor restart, source change), or null. */
    public String event() {
        return event;
    }

    private boolean fits(Track s, double raw, double t) {
        return dist(raw, s.good) <= gate(t - s.goodT);
    }

    /** The sensor's move since its last reading if the robot can make it (as is or, for the IMU, flipped), else NaN. */
    private double smoothStep(Track s, double raw, double t) {
        double g = gate(t - s.prevT);
        double step = MecanumMath.normalizeDegrees(raw - s.prev);
        if (Math.abs(step) <= g) {
            return step;
        }
        double flipped = MecanumMath.normalizeDegrees(raw + 180 - s.prev);
        return s.flips && Math.abs(flipped) <= g ? flipped : Double.NaN;
    }

    /**
     * An off-track sensor reading near the predicted heading again. A restarted sensor moves on smoothly from its
     * jump, so after a smooth move only a reading very near counts (else the restart would pass as a return).
     */
    private boolean back(Track s, double raw, double driver, double pred, double t) {
        double d = dist(driver, pred);
        if (d <= envelope(Math.min(t - headingT, BACK_DT_CAP_SEC))) {
            return true;
        }
        return Double.isNaN(smoothStep(s, raw, t)) && d <= envelope(Math.min(t - headingT, GATE_DT_CAP_SEC));
    }

    /**
     * Still off its track: moving on smoothly from its jump for rebaseSec means it restarted: line it up. If nobody gave
     * the heading meanwhile (held), the turn it saw since is carried into the heading first.
     *
     * @return it restarted now
     */
    private boolean settle(Track s, boolean tracked, double raw, double t, boolean held) {
        if (!tracked || s.onTrack() || s.offSince == t) {
            return false;
        }
        double step = smoothStep(s, raw, t);
        if (Double.isNaN(step)) {
            s.levelSince = t;                                // jumped again
            s.levelTurn = 0;
            return false;
        }
        s.levelTurn = held ? s.levelTurn + step : 0;
        if (t - s.levelSince < rebaseSec) {
            return false;
        }
        if (held) {
            setHeading(MecanumMath.normalizeDegrees(heading + s.levelTurn), t);
        }
        lineUp(s, raw, t);
        addEvent(s.name + " heading restarted: carries on from the current heading");
        return true;
    }

    private double gate(double dt) {
        return maxRateDps * Math.min(dt, GATE_DT_CAP_SEC) + marginDeg;
    }

    private double predicted(double t) {
        return MecanumMath.normalizeDegrees(heading + rateDps * Math.min(t - headingT, GATE_DT_CAP_SEC));
    }

    /**
     * How far the robot can get from the predicted heading in dt, + margin: the predicted turn rate is the average over
     * its interval (it lags by half of it), and the rate changes at most MAX_ACCEL_DPS2, up to maxRate either way.
     */
    private double envelope(double dt) {
        return envelope(dt, rateDps, rateDt);
    }

    private double envelope(double dt, double rate, double overSec) {
        double swing = maxRateDps + Math.abs(rate);          // the most the turn rate can be off
        double lag = Math.min(MAX_ACCEL_DPS2 * overSec / 2, swing);
        double toSwing = (swing - lag) / MAX_ACCEL_DPS2;
        double dev = dt <= toSwing ? lag * dt + 0.5 * MAX_ACCEL_DPS2 * dt * dt
                : lag * toSwing + 0.5 * MAX_ACCEL_DPS2 * toSwing * toSwing + swing * (dt - toSwing);
        return dev + marginDeg;
    }

    /**
     * A lone sensor's reading out of reach of the prediction, but in reach of the one before its last reading and at
     * least twice as near it: that last reading was the bad one (it passed and moved the prediction): drop it.
     */
    private boolean lastWasOff(double driver, String name, double t) {
        if (!beforeRateKnown) {
            return false;
        }
        double dt = Math.min(t - beforeT, GATE_DT_CAP_SEC);
        double miss = dist(driver, MecanumMath.normalizeDegrees(beforeHeading + beforeRate * dt));
        if (miss > envelope(dt, beforeRate, beforeRateDt) || miss > 0.5 * dist(driver, predicted(t))) {
            return false;
        }
        heading = beforeHeading;
        headingT = beforeT;
        rateDps = beforeRate;
        rateDt = beforeRateDt;
        addEvent(name + " heading: the last reading was off: dropped");
        return true;
    }

    private void setHeading(double h, double t) {
        beforeHeading = heading;
        beforeT = headingT;
        beforeRate = rateDps;
        beforeRateDt = rateDt;
        beforeRateKnown = rateKnown;
        if (t - headingT > 1e-3) {
            double rate = MecanumMath.normalizeDegrees(h - heading) / (t - headingT);
            rateDps = Math.max(-maxRateDps, Math.min(maxRateDps, rate));
            rateDt = t - headingT;
            rateKnown = true;
        }
        heading = h;
        headingT = t;
    }

    private static void keep(Track s, double raw, double t) {
        s.good = raw;
        s.goodT = t;
        s.offSince = Double.NaN;
    }

    private void lineUp(Track s, double raw, double t) {
        s.off = MecanumMath.normalizeDegrees(heading - raw);
        keep(s, raw, t);
    }

    private static void offTrack(Track s, double t) {
        s.offSince = t;
        s.levelSince = t;
        s.levelTurn = 0;
    }

    private void glitch(Track s, double raw, double t) {
        glitches++;                                          // a jump the robot can't make: ignore it for now
        addEvent(String.format("%s heading jumped %.0f deg in %.0f ms: ignored", s.name, dist(raw, s.good),
                (t - s.goodT) * 1000));
        offTrack(s, t);
    }

    private void imuFlipped(double t) {
        imuFlips++;
        if (t - flipEventT >= FLIP_EVENT_SEC) {
            addEvent(String.format("IMU heading flipped 180 deg: corrected (%d so far)", imuFlips));
            flipEventT = t;
        }
    }

    private void use(Source s) {
        if (source != s && source != Source.NONE) {
            addEvent("heading from " + s + " (was " + source + ")");
        }
        source = s;
    }

    private void addEvent(String msg) {
        event = event == null ? msg : event + "; " + msg;
    }

    private static double dist(double a, double b) {
        return Math.abs(MecanumMath.normalizeDegrees(a - b));
    }
}

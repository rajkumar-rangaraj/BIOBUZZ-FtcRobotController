package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * Field-centric driving (TeleOp): the robot's heading relative to the DRIVERS, 0 = the intake points away from them,
 * counter-clockwise positive. The Pinpoint's heading leads; the Control Hub IMU carries the heading whenever the Pinpoint
 * has no reading or a reading off its track, and is kept lined up with it otherwise, so switching never jumps.
 * <ul>
 *   <li>A reading that jumps faster than the robot can turn is a sensor glitch (9 Nov logs: the IMU read -58, +89, -68
 *       deg within 0.5 s) and is ignored. If both sensors jump alike it is a real (hard) turn and is followed.</li>
 *   <li>A jump that stays REBASE_SEC is a sensor that restarted from zero: it carries on from the current heading.</li>
 *   <li>A sensor that comes back after a gap (no reading in the last update) rejoins at the current heading; a rate
 *       check over the gap would let a restart through (review, 10 Nov).</li>
 * </ul>
 * No FTC SDK dependency: unit-tested.
 */
public class FieldHeading {

    public enum Source { PINPOINT, IMU, NONE }

    private final double maxRateDps;
    private final double marginDeg;
    private final double rebaseSec;

    private double startDeg;                     // heading relative to the drivers when the first reading comes
    private boolean started = false;
    private double heading = Double.NaN;
    private Source source = Source.NONE;
    private double offPin = Double.NaN, offImu = Double.NaN;   // driver heading = sensor + offset
    private double lastPin = Double.NaN, lastImu = Double.NaN;  // last accepted sensor readings ...
    private double lastPinT = Double.NaN, lastImuT = Double.NaN;  // ... and when
    private double badSincePin = Double.NaN, badSinceImu = Double.NaN;   // a glitch began (NaN = none) ...
    private double glitchRawPin, glitchRawImu, glitchTPin, glitchTImu;     // ... and its latest reading
    private boolean pinBefore = false, imuBefore = false;                 // had a reading in the last update
    private double rawPin = Double.NaN, rawImu = Double.NaN, rawPinT, rawImuT;   // the latest readings, even rejected
    private int glitches = 0;
    private String event = null;
    static final double REJOIN_SEC = 0.5;            // a sensor silent this long rejoins from the current heading
    static final double AGREE_DEG = 20;              // both sensors jumped alike: a real (hard, fast) turn

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
     * @param imuDeg      the IMU heading (CCW +), NaN when not read (read it when the Pinpoint has none)
     * @return the heading relative to the drivers, degrees in (-180, 180]; NaN while there is no reading (drive
     *         robot-centric then)
     */
    public double update(double t, double pinpointDeg, double imuDeg) {
        event = null;
        boolean pinIn = !Double.isNaN(pinpointDeg), imuIn = !Double.isNaN(imuDeg);
        boolean pinJoins = pinIn && !pinBefore, imuJoins = imuIn && !imuBefore;   // back after a gap: rejoin
        pinBefore = pinIn;
        imuBefore = imuIn;
        rawPin = pinpointDeg;                                // the latest readings (for reset), accepted or not
        rawImu = imuDeg;
        rawPinT = t;
        rawImuT = t;
        if (!pinIn && !imuIn) {
            if (source != Source.NONE) {
                event = "no heading reading (Pinpoint and IMU): robot-centric until one comes back";
            }
            source = Source.NONE;
            return Double.NaN;                               // heading() keeps the last one for when a sensor returns
        }
        if (!started) {                                      // the very first reading: the start heading
            started = true;
            heading = MecanumMath.normalizeDegrees(startDeg);
        }
        double pinJump = pinIn && !Double.isNaN(lastPin) ? MecanumMath.normalizeDegrees(pinpointDeg - lastPin) : Double.NaN;
        double imuJump = imuIn && !Double.isNaN(lastImu) ? MecanumMath.normalizeDegrees(imuDeg - lastImu) : Double.NaN;
        double before = heading;
        Double hi = imuIn ? accept(t, imuDeg, false, imuJoins) : null;
        if (hi != null) {
            heading = hi;                                    // this loop's IMU heading: a Pinpoint that (re)joins or
        }                                                    // restarted lines up with it, not with the last loop's
        Double hp = pinIn ? accept(t, pinpointDeg, true, pinJoins) : null;
        if (hp != null) {
            use(Source.PINPOINT, hp);
            if (hi != null) {
                store(false, imuDeg, heading - imuDeg, t);   // keep the IMU lined up while the Pinpoint leads
            }
            return heading;
        }
        heading = before;
        if (hi != null) {                                    // the Pinpoint is missing or off its track: the IMU leads
            use(Source.IMU, hi);
            return heading;
        }
        if (pinIn && imuIn && Math.abs(MecanumMath.normalizeDegrees(pinJump - imuJump)) <= AGREE_DEG) {
            store(true, pinpointDeg, offPin, t);             // both jumped alike: a real (hard, fast) turn
            store(false, imuDeg, offImu, t);
            endGlitch(true);
            endGlitch(false);
            use(Source.PINPOINT, MecanumMath.normalizeDegrees(pinpointDeg + offPin));
            event = String.format("fast turn seen by both sensors (%.0f deg): followed", pinJump);
            return heading;
        }
        return heading;                                      // both off their tracks: keep the last good heading
    }

    /** The driver pointed the intake away from them and pressed reset: that is heading 0 from now on. */
    public void reset() {
        if (!started) {
            startDeg = 0;
        } else {
            heading = 0;                                     // based on each sensor's LATEST reading, accepted or not:
            if (Double.isNaN(rawPin)) {                      // the last accepted one may be stale inside a glitch
                offPin = Double.NaN;
            } else {
                store(true, rawPin, -rawPin, rawPinT);
            }
            if (Double.isNaN(rawImu)) {
                offImu = Double.NaN;
            } else {
                store(false, rawImu, -rawImu, rawImuT);
            }
            endGlitch(true);
            endGlitch(false);
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

    /** What happened in the last update (glitch, sensor restart, source change), or null. */
    public String event() {
        return event;
    }

    private void use(Source s, double h) {
        if (source != s && source != Source.NONE && s != Source.NONE) {
            String msg = "heading from " + s + " (was " + source + ")";
            event = event == null ? msg : event + "; " + msg;
        }
        source = s;
        heading = h;
    }

    /**
     * The driver heading from this sensor reading, or null to keep the last one (glitch).
     *
     * @param joins the sensor had no reading in the last update: it rejoins at the current heading (it may have restarted
     *              meanwhile, and a rate check over the gap would pass almost any jump)
     */
    private Double accept(double t, double raw, boolean pin, boolean joins) {
        double last = pin ? lastPin : lastImu;
        double lastSrcT = pin ? lastPinT : lastImuT;
        double off = pin ? offPin : offImu;
        if (joins || Double.isNaN(off) || t - lastSrcT > REJOIN_SEC) {   // (re)joins: carry on from the heading
            off = heading - raw;
            store(pin, raw, off, t);
            endGlitch(pin);
            return MecanumMath.normalizeDegrees(raw + off);
        }
        double fromGood = Math.abs(MecanumMath.normalizeDegrees(raw - last));
        double badSince = pin ? badSincePin : badSinceImu;
        if (!Double.isNaN(badSince)) {                       // inside a glitch: is this the new level or the old track?
            double glitchRaw = pin ? glitchRawPin : glitchRawImu;
            double glitchT = pin ? glitchTPin : glitchTImu;
            boolean newLevel = Math.abs(MecanumMath.normalizeDegrees(raw - glitchRaw)) <= maxRateDps * (t - glitchT) + marginDeg;
            if (newLevel && t - badSince >= rebaseSec) {     // it stayed: the sensor restarted; carry on from here
                off = heading - raw;
                store(pin, raw, off, t);
                endGlitch(pin);
                event = (pin ? "Pinpoint" : "IMU") + " heading restarted: carried on from the last good heading";
                return heading;
            }
            if (!newLevel && fromGood <= maxRateDps * (t - lastSrcT) + marginDeg) {
                store(pin, raw, off, t);                     // back on the old track: it was a one-off
                endGlitch(pin);
                return MecanumMath.normalizeDegrees(raw + off);
            }
            setGlitch(pin, raw, t, badSince);                // still off the track
            return null;
        }
        if (fromGood <= maxRateDps * (t - lastSrcT) + marginDeg) {
            store(pin, raw, off, t);
            return MecanumMath.normalizeDegrees(raw + off);
        }
        glitches++;                                          // a jump the robot can't make: ignore it for now
        event = String.format("%s heading jumped %.0f deg in %.0f ms: ignored", pin ? "Pinpoint" : "IMU", fromGood,
                (t - lastSrcT) * 1000);
        setGlitch(pin, raw, t, t);
        return null;
    }

    private void setGlitch(boolean pin, double raw, double t, double since) {
        if (pin) {
            badSincePin = since;
            glitchRawPin = raw;
            glitchTPin = t;
        } else {
            badSinceImu = since;
            glitchRawImu = raw;
            glitchTImu = t;
        }
    }

    private void endGlitch(boolean pin) {
        if (pin) {
            badSincePin = Double.NaN;
        } else {
            badSinceImu = Double.NaN;
        }
    }

    private void store(boolean pin, double raw, double off, double t) {
        if (pin) {
            lastPin = raw;
            offPin = off;
            lastPinT = t;
        } else {
            lastImu = raw;
            offImu = off;
            lastImuT = t;
        }
    }
}

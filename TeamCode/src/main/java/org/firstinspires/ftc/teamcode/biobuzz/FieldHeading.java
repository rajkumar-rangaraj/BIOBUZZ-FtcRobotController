package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * Field-centric driving (TeleOp): the robot's heading relative to the DRIVERS, 0 = the intake points away from them,
 * counter-clockwise positive. Two sensors, each with its own offset to that heading: the Pinpoint (leads) and the
 * Control Hub IMU (read right after the Pinpoint, so the two agree while turning).
 * <ul>
 *   <li>Both agree (within AGREE_DEG): that is the heading, however fast it moved (a hard hit, or a turn during a loop
 *       stall). The IMU is kept lined up with the Pinpoint.</li>
 *   <li>The IMU yaw flips by 180 deg, mostly where it crosses +-90 (diag logs 30 Oct - 9 Nov: ~1290 flips, each one
 *       the wheels' turn +-180). A jump of 180 deg plus a move the robot can make is such a flip: corrected.</li>
 *   <li>Otherwise a sensor's reading must be a move the robot can make since that sensor's last good reading, looking
 *       back at most GATE_DT_CAP_SEC (after a long loop stall the rate check would pass anything). A bigger jump is a
 *       glitch. If both moved plausibly but disagree, the one closer to the predicted heading (from the last turn
 *       rate) leads and the other is off its track.</li>
 *   <li>A sensor off its track is back when it agrees with the other one again or, with no other sensor to vouch,
 *       when it jumps back near the predicted heading. If it moves on smoothly from its jump for rebaseSec instead, it
 *       restarted: it carries on from the current heading.</li>
 *   <li>A sensor that comes back after a gap (no reading in the last update) rejoins at the current heading.</li>
 *   <li>Reset turns the whole frame: a glitch in progress is sorted out the same way afterwards.</li>
 * </ul>
 * Limit: a sensor on its own can't tell a restart from a turn made during a loop stall longer than ~0.4 s at full
 * speed; that turn is lost (Options fixes it). No FTC SDK dependency: unit-tested.
 */
public class FieldHeading {

    public enum Source { PINPOINT, IMU, NONE }

    static final double AGREE_DEG = 10;              // the two sensors agree (read back to back)
    static final double GATE_DT_CAP_SEC = 0.25;      // the rate check looks back at most this long
    static final double MAX_ACCEL_DPS2 = 2000;       // a sensor back from a glitch on its own: near the prediction

    private enum Verdict { NONE, ON_TRACK, OFF_TRACK, RESTARTED }

    /** One heading sensor. */
    private static final class Track {
        final String name;
        double off = Double.NaN;                     // driver heading = reading + off (NaN: joins at the next reading)
        double good, goodT;                          // its last reading on its track, and when
        double prev, prevT;                          // its last reading, on its track or not, and when
        double offSince = Double.NaN;                // off its track since (NaN: on it) ...
        double levelSince;                           // ... and moving on smoothly from its last jump since
        boolean had = false;                         // had a reading in the last update

        Track(String name) {
            this.name = name;
        }

        boolean onTrack() {
            return Double.isNaN(offSince);
        }

        double driver(double raw) {
            return MecanumMath.normalizeDegrees(raw + off);
        }
    }

    private final double maxRateDps;
    private final double marginDeg;
    private final double rebaseSec;
    private final Track pin = new Track("Pinpoint");
    private final Track imu = new Track("IMU");

    private double startDeg;                         // heading relative to the drivers when the first reading comes
    private boolean started = false;
    private double heading = Double.NaN;
    private double headingT;                         // when a sensor last vouched for the heading
    private double rateDps = 0;                      // the heading's last turn rate (predictions)
    private Source source = Source.NONE;
    private int glitches = 0;
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
        boolean pinTracked = pinIn && !pinJoins, imuTracked = imuIn && !imuJoins;
        boolean pinOk = false, imuOk = false;                // lined up with the heading after this update
        if (pinTracked && imuTracked
                && Math.abs(MecanumMath.normalizeDegrees(pin.driver(pinpointDeg) - imu.driver(imuDeg))) <= AGREE_DEG) {
            double h = pin.driver(pinpointDeg);              // both agree: that's the heading, however fast it moved
            if (pin.onTrack() && Math.abs(MecanumMath.normalizeDegrees(pinpointDeg - pin.good)) > gate(t - pin.goodT)) {
                addEvent(String.format("fast turn seen by both sensors (%.0f deg): followed",
                        MecanumMath.normalizeDegrees(h - heading)));
            }
            setHeading(h, t);
            keep(pin, pinpointDeg, t);
            imu.off = h - imuDeg;                            // keep the IMU lined up with the Pinpoint
            keep(imu, imuDeg, t);
            pinOk = true;
            imuOk = true;
        } else {
            Verdict vp = pinTracked ? judge(pin, pinpointDeg, t, imuTracked && imu.onTrack()) : Verdict.NONE;
            Verdict vi = imuTracked ? judge(imu, imuDeg, t, pinTracked && pin.onTrack()) : Verdict.NONE;
            if (vp == Verdict.ON_TRACK && vi == Verdict.ON_TRACK
                    && Math.abs(MecanumMath.normalizeDegrees(pin.driver(pinpointDeg) - imu.driver(imuDeg))) > AGREE_DEG) {
                double pred = predicted(t);                  // both moved plausibly but disagree
                boolean pinCloser = Math.abs(MecanumMath.normalizeDegrees(pin.driver(pinpointDeg) - pred))
                        <= Math.abs(MecanumMath.normalizeDegrees(imu.driver(imuDeg) - pred));
                Track loser = pinCloser ? imu : pin;
                glitches++;
                addEvent(String.format("%s heading %.0f deg off the %s: ignored", loser.name,
                        Math.abs(MecanumMath.normalizeDegrees(pin.driver(pinpointDeg) - imu.driver(imuDeg))),
                        pinCloser ? pin.name : imu.name));
                offTrack(loser, t);
                if (pinCloser) {
                    vi = Verdict.OFF_TRACK;
                } else {
                    vp = Verdict.OFF_TRACK;
                }
            }
            if (vp == Verdict.ON_TRACK) {
                setHeading(pin.driver(pinpointDeg), t);
                keep(pin, pinpointDeg, t);
                pinOk = true;
                if (vi == Verdict.ON_TRACK) {                // they agree after an IMU flip correction
                    imu.off = heading - imuDeg;
                    keep(imu, imuDeg, t);
                    imuOk = true;
                }
            } else if (vi == Verdict.ON_TRACK) {             // the Pinpoint is missing or off its track: the IMU leads
                setHeading(imu.driver(imuDeg), t);
                keep(imu, imuDeg, t);
                imuOk = true;
            }
            if (vp == Verdict.RESTARTED) {
                lineUp(pin, pinpointDeg, t);
                addEvent("Pinpoint heading restarted: carries on from the current heading");
                pinOk = true;
            }
            if (vi == Verdict.RESTARTED) {
                lineUp(imu, imuDeg, t);
                addEvent("IMU heading restarted: carries on from the current heading");
                imuOk = true;
            }
        }
        if (pinJoins) {                                      // after this update's heading, so it lines up with it
            lineUp(pin, pinpointDeg, t);
            pinOk = true;
        }
        if (imuJoins) {
            lineUp(imu, imuDeg, t);
            imuOk = true;
        }
        if (pinOk || imuOk) {
            headingT = t;
            use(pinOk ? Source.PINPOINT : Source.IMU);
        }                                                    // else both off their tracks: keep the last good heading
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
            pin.off = MecanumMath.normalizeDegrees(pin.off - heading);
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

    /** What happened in the last update (glitch, sensor restart, source change), or null. */
    public String event() {
        return event;
    }

    /**
     * This reading against the sensor's track (both sensors did NOT agree).
     *
     * @param vouched the other sensor is on its track: then this one, if off its track, is only back by agreeing with
     *                it, which was checked before
     */
    private Verdict judge(Track s, double raw, double t, boolean vouched) {
        if (s.onTrack()) {
            double jump = Math.abs(MecanumMath.normalizeDegrees(raw - s.good));
            if (jump <= gate(t - s.goodT)) {
                return Verdict.ON_TRACK;
            }
            if (s == imu && Math.abs(MecanumMath.normalizeDegrees(raw + 180 - s.good)) <= gate(t - s.goodT)) {
                s.off = MecanumMath.normalizeDegrees(s.off + 180);     // the IMU yaw flipped (see the class notes)
                addEvent("IMU heading flipped 180 deg: corrected");
                return Verdict.ON_TRACK;
            }
            glitches++;                                      // a jump the robot can't make: ignore it for now
            addEvent(String.format("%s heading jumped %.0f deg in %.0f ms: ignored", s.name, jump, (t - s.goodT) * 1000));
            offTrack(s, t);
            return Verdict.OFF_TRACK;
        }
        if (Math.abs(MecanumMath.normalizeDegrees(raw - s.prev)) <= gate(t - s.prevT)) {
            return t - s.levelSince >= rebaseSec ? Verdict.RESTARTED : Verdict.OFF_TRACK;   // moving on from its jump
        }
        if (!vouched) {                                      // jumped back near where the heading was going: a one-off
            double dt = Math.min(t - headingT, GATE_DT_CAP_SEC);
            double tolerance = Math.min(0.5 * MAX_ACCEL_DPS2 * dt * dt, maxRateDps * dt) + marginDeg;
            if (Math.abs(MecanumMath.normalizeDegrees(s.driver(raw) - predicted(t))) <= tolerance) {
                return Verdict.ON_TRACK;
            }
        }
        s.levelSince = t;                                    // jumped again
        return Verdict.OFF_TRACK;
    }

    private double gate(double dt) {
        return maxRateDps * Math.min(dt, GATE_DT_CAP_SEC) + marginDeg;
    }

    private double predicted(double t) {
        return MecanumMath.normalizeDegrees(heading + rateDps * Math.min(t - headingT, GATE_DT_CAP_SEC));
    }

    private void setHeading(double h, double t) {
        if (t - headingT > 1e-3) {
            double rate = MecanumMath.normalizeDegrees(h - heading) / (t - headingT);
            rateDps = Math.max(-maxRateDps, Math.min(maxRateDps, rate));
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
        s.off = heading - raw;
        keep(s, raw, t);
    }

    private static void offTrack(Track s, double t) {
        s.offSince = t;
        s.levelSince = t;
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
}

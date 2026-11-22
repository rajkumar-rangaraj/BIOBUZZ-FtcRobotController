package org.firstinspires.ftc.teamcode.v2;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Everything MEASURED on the v2 robot, saved on the Control Hub by "v2 Setup" so nobody edits code to calibrate.
 *
 * File: plain "key=value" lines (same style as the v1 arm calibration), e.g.
 * <pre>
 *   dir.launcher2=REVERSE
 *   gate.closed=0.2150
 *   queue.ticksPerBall=338
 * </pre>
 * Reading never throws and never crashes an OpMode: a missing file gives the defaults, a bad line keeps its
 * default and adds a warning, unknown keys are ignored (so older and newer files both load).
 * Values that must be measured start as "not calibrated" ({@link #missing()} lists them); code that depends on
 * them (e.g. firing needs the gate) must refuse to run until they are set.
 * The SETTERS are strict: a bad value throws IllegalArgumentException and changes nothing, so callers (v2 Setup)
 * must catch it and show the message. Only parse()/load() are forgiving.
 * The last line of a saved file is "end=ok"; a file without it was cut short and loads with a warning.
 */
public final class V2Calibration {
    public static final String FILE_NAME = "biobuzz_v2_calibration.txt";
    public static final int FORMAT_VERSION = 1;
    static final String END_KEY = "end";
    /** keys this version understands; an unknown key with one of the known prefixes is probably a typo */
    static final String[] KNOWN_PREFIXES = {"dir.", "gate.", "arm.", "beam", "queue.", "intake."};

    // limits for measured values (anything outside is a typo or a bad measurement)
    static final double MIN_GATE_TRAVEL = 0.05;            // servo units between gate CLOSED and OPEN
    static final double MIN_ARM_TRAVEL = 0.05;
    static final int MIN_TICKS_PER_BALL = 50, MAX_TICKS_PER_BALL = 5000;
    static final double MIN_JAM_AMPS = 0.5, MAX_JAM_AMPS = 10.0;

    // defaults
    static final int DEFAULT_TICKS_PER_BALL = 340;          // estimate (435 rpm motor, 16T-16T, 73 mm ball): MEASURE
    static final double DEFAULT_JAM_AMPS = 4.0;

    private final Map<String, Boolean> motorReversed = new LinkedHashMap<>();
    private final Map<String, Boolean> crServoReversed = new LinkedHashMap<>();
    private boolean directionsChecked = false;               // set by v2 Setup once every direction was confirmed
    private double gateClosed = Double.NaN, gateOpen = Double.NaN;
    private double armStowed = Double.NaN, armDeployed = Double.NaN;
    private boolean beamStagedBrokenWhenLow = true, beamFullBrokenWhenLow = true;   // Adafruit 2167: LOW = broken
    private boolean beamsChecked = false;
    private int queueTicksPerBall = DEFAULT_TICKS_PER_BALL;
    private boolean queueTicksMeasured = false;
    private double intakeJamAmps = DEFAULT_JAM_AMPS;
    private boolean intakeJamMeasured = false;
    private String savedAt = "";

    private final List<String> warnings = new ArrayList<>();
    private boolean fromFile = false;

    public V2Calibration() {
        for (String m : V2Config.MOTORS) {
            motorReversed.put(m, false);
        }
        for (String s : V2Config.CR_SERVOS) {
            crServoReversed.put(s, false);
        }
    }

    // ------------------------------------------------------------------ getters
    public boolean motorReversed(String name) {
        Boolean r = motorReversed.get(name);
        return r != null && r;
    }

    public boolean crServoReversed(String name) {
        Boolean r = crServoReversed.get(name);
        return r != null && r;
    }

    public boolean directionsChecked() { return directionsChecked; }
    public double gateClosed() { return gateClosed; }
    public double gateOpen() { return gateOpen; }
    public boolean gateCalibrated() { return !Double.isNaN(gateClosed) && !Double.isNaN(gateOpen); }
    public double armStowed() { return armStowed; }
    public double armDeployed() { return armDeployed; }
    public boolean armCalibrated() { return !Double.isNaN(armStowed) && !Double.isNaN(armDeployed); }
    public boolean beamStagedBrokenWhenLow() { return beamStagedBrokenWhenLow; }
    public boolean beamFullBrokenWhenLow() { return beamFullBrokenWhenLow; }
    public boolean beamsChecked() { return beamsChecked; }
    public int queueTicksPerBall() { return queueTicksPerBall; }
    public boolean queueTicksMeasured() { return queueTicksMeasured; }
    public double intakeJamAmps() { return intakeJamAmps; }
    public boolean intakeJamMeasured() { return intakeJamMeasured; }
    public String savedAt() { return savedAt; }
    /** problems found while reading the file (empty when it was clean or there was no file) */
    public List<String> warnings() { return Collections.unmodifiableList(warnings); }
    /** true if these values came from a saved file */
    public boolean fromFile() { return fromFile; }

    /** what still has to be measured with "v2 Setup" (empty = fully calibrated) */
    public List<String> missing() {
        List<String> out = new ArrayList<>();
        if (!directionsChecked) out.add("motor + servo directions");
        if (!beamsChecked) out.add("beam polarity");
        if (!gateCalibrated()) out.add("gate CLOSED / OPEN");
        if (!armCalibrated()) out.add("flower arm STOWED / DEPLOYED");
        if (!queueTicksMeasured) out.add("queue ticks per ball");
        if (!intakeJamMeasured) out.add("intake jam current");
        return out;
    }

    // ------------------------------------------------------------------ setters (v2 Setup); each validates
    public void setMotorReversed(String name, boolean reversed) {
        if (!motorReversed.containsKey(name)) throw new IllegalArgumentException("unknown motor " + name);
        motorReversed.put(name, reversed);
    }

    public void setCrServoReversed(String name, boolean reversed) {
        if (!crServoReversed.containsKey(name)) throw new IllegalArgumentException("unknown CR servo " + name);
        crServoReversed.put(name, reversed);
    }

    public void setDirectionsChecked(boolean v) { directionsChecked = v; }

    public void setGate(double closed, double open) {
        String err = checkPair("gate", closed, open, MIN_GATE_TRAVEL);
        if (err != null) throw new IllegalArgumentException(err);
        gateClosed = closed;
        gateOpen = open;
    }

    public void setArm(double stowed, double deployed) {
        String err = checkPair("arm", stowed, deployed, MIN_ARM_TRAVEL);
        if (err != null) throw new IllegalArgumentException(err);
        armStowed = stowed;
        armDeployed = deployed;
    }

    public void setBeams(boolean stagedBrokenWhenLow, boolean fullBrokenWhenLow) {
        beamStagedBrokenWhenLow = stagedBrokenWhenLow;
        beamFullBrokenWhenLow = fullBrokenWhenLow;
        beamsChecked = true;
    }

    public void setQueueTicksPerBall(int ticks) {
        if (ticks < MIN_TICKS_PER_BALL || ticks > MAX_TICKS_PER_BALL) {
            throw new IllegalArgumentException("queue ticks per ball " + ticks + " outside " + MIN_TICKS_PER_BALL + ".."
                    + MAX_TICKS_PER_BALL);
        }
        queueTicksPerBall = ticks;
        queueTicksMeasured = true;
    }

    public void setIntakeJamAmps(double amps) {
        if (!(amps >= MIN_JAM_AMPS && amps <= MAX_JAM_AMPS)) {
            throw new IllegalArgumentException("intake jam current " + amps + " A outside " + MIN_JAM_AMPS + ".."
                    + MAX_JAM_AMPS);
        }
        intakeJamAmps = amps;
        intakeJamMeasured = true;
    }

    /** null if a and b are valid servo positions far enough apart, else the reason */
    static String checkPair(String what, double a, double b, double minTravel) {
        if (!(a >= 0 && a <= 1) || !(b >= 0 && b <= 1)) return what + " positions must be 0..1";
        if (Math.abs(a - b) < minTravel) return what + " positions are only " + String.format(Locale.US, "%.3f", Math.abs(a - b))
                + " apart (need " + minTravel + ")";
        return null;
    }

    // ------------------------------------------------------------------ text format
    /** the file text for these values ({@code stamp} = when it was saved, free text) */
    public String format(String stamp) {
        StringBuilder sb = new StringBuilder();
        sb.append("# BIOBUZZ v2 calibration - written by \"v2 Setup\". Delete a line to fall back to its default.\n");
        sb.append("version=").append(FORMAT_VERSION).append('\n');
        sb.append("savedAt=").append(stamp == null ? "" : stamp.replace('\n', ' ')).append('\n');
        for (Map.Entry<String, Boolean> e : motorReversed.entrySet()) {
            sb.append("dir.").append(e.getKey()).append('=').append(e.getValue() ? "REVERSE" : "FORWARD").append('\n');
        }
        for (Map.Entry<String, Boolean> e : crServoReversed.entrySet()) {
            sb.append("dir.").append(e.getKey()).append('=').append(e.getValue() ? "REVERSE" : "FORWARD").append('\n');
        }
        sb.append("directions.checked=").append(directionsChecked).append('\n');
        if (gateCalibrated()) {
            sb.append(String.format(Locale.US, "gate.closed=%.4f\ngate.open=%.4f\n", gateClosed, gateOpen));
        }
        if (armCalibrated()) {
            sb.append(String.format(Locale.US, "arm.stowed=%.4f\narm.deployed=%.4f\n", armStowed, armDeployed));
        }
        sb.append("beam.staged.brokenWhenLow=").append(beamStagedBrokenWhenLow).append('\n');
        sb.append("beam.full.brokenWhenLow=").append(beamFullBrokenWhenLow).append('\n');
        sb.append("beams.checked=").append(beamsChecked).append('\n');
        if (queueTicksMeasured) {
            sb.append("queue.ticksPerBall=").append(queueTicksPerBall).append('\n');
        }
        if (intakeJamMeasured) {
            sb.append(String.format(Locale.US, "intake.jamAmps=%.2f\n", intakeJamAmps));
        }
        sb.append(END_KEY).append("=ok\n");                  // must stay the last line (completeness marker)
        return sb.toString();
    }

    /** reads the file text; never throws. Bad lines keep their defaults and add a warning. */
    public static V2Calibration parse(String text) {
        V2Calibration c = new V2Calibration();
        if (text == null) {
            return c;
        }
        Map<String, String> kv = new LinkedHashMap<>();
        int lineNo = 0;
        for (String raw : text.split("\\r?\\n")) {
            lineNo++;
            String line = raw.trim();
            if (line.isEmpty() || line.startsWith("#")) continue;
            int eq = line.indexOf('=');
            if (eq <= 0) {
                c.warnings.add("line " + lineNo + " has no key=value: '" + line + "'");
                continue;
            }
            kv.put(line.substring(0, eq).trim(), line.substring(eq + 1).trim());
        }
        if (!"ok".equals(kv.get(END_KEY))) {
            c.warnings.add("file looks incomplete (no 'end=ok' last line): check the values or run v2 Setup again");
        }
        java.util.Set<String> known = knownKeys();
        for (String k : kv.keySet()) {
            if (known.contains(k)) continue;
            for (String pre : KNOWN_PREFIXES) {
                if (k.startsWith(pre)) {
                    c.warnings.add("unknown key '" + k + "' (typo?): ignored");
                    break;
                }
            }
        }
        String ver = kv.get("version");
        if (ver != null && !ver.equals(String.valueOf(FORMAT_VERSION))) {
            c.warnings.add("file version " + ver + " (this code reads " + FORMAT_VERSION + "): read what is understood");
        }
        c.savedAt = kv.containsKey("savedAt") ? kv.get("savedAt") : "";
        for (String m : V2Config.MOTORS) {
            Boolean r = c.readDir(kv, "dir." + m);
            if (r != null) c.motorReversed.put(m, r);
        }
        for (String s : V2Config.CR_SERVOS) {
            Boolean r = c.readDir(kv, "dir." + s);
            if (r != null) c.crServoReversed.put(s, r);
        }
        Boolean b = c.readBool(kv, "directions.checked");
        if (b != null) c.directionsChecked = b;

        Double gc = c.readDouble(kv, "gate.closed"), go = c.readDouble(kv, "gate.open");
        if (gc != null || go != null) {
            String err = gc == null || go == null ? "gate needs both gate.closed and gate.open"
                    : checkPair("gate", gc, go, MIN_GATE_TRAVEL);
            if (err == null) {
                c.gateClosed = gc;
                c.gateOpen = go;
            } else {
                c.warnings.add(err + ": gate NOT calibrated");
            }
        }
        Double as = c.readDouble(kv, "arm.stowed"), ad = c.readDouble(kv, "arm.deployed");
        if (as != null || ad != null) {
            String err = as == null || ad == null ? "arm needs both arm.stowed and arm.deployed"
                    : checkPair("arm", as, ad, MIN_ARM_TRAVEL);
            if (err == null) {
                c.armStowed = as;
                c.armDeployed = ad;
            } else {
                c.warnings.add(err + ": arm NOT calibrated");
            }
        }
        b = c.readBool(kv, "beam.staged.brokenWhenLow");
        if (b != null) c.beamStagedBrokenWhenLow = b;
        b = c.readBool(kv, "beam.full.brokenWhenLow");
        if (b != null) c.beamFullBrokenWhenLow = b;
        b = c.readBool(kv, "beams.checked");
        if (b != null) c.beamsChecked = b;

        if (kv.containsKey("queue.ticksPerBall")) {
            try {
                int t = Integer.parseInt(kv.get("queue.ticksPerBall"));
                if (t >= MIN_TICKS_PER_BALL && t <= MAX_TICKS_PER_BALL) {
                    c.queueTicksPerBall = t;
                    c.queueTicksMeasured = true;
                } else {
                    c.warnings.add("queue.ticksPerBall " + t + " out of range: default " + DEFAULT_TICKS_PER_BALL + " used");
                }
            } catch (NumberFormatException e) {
                c.warnings.add("queue.ticksPerBall is not a whole number: default used");
            }
        }
        Double j = c.readDouble(kv, "intake.jamAmps");
        if (j != null) {
            if (j >= MIN_JAM_AMPS && j <= MAX_JAM_AMPS) {
                c.intakeJamAmps = j;
                c.intakeJamMeasured = true;
            } else {
                c.warnings.add("intake.jamAmps " + j + " out of range: default " + DEFAULT_JAM_AMPS + " used");
            }
        }
        return c;
    }

    /** every key this version reads or writes */
    static java.util.Set<String> knownKeys() {
        java.util.Set<String> k = new java.util.HashSet<>(Arrays.asList("version", "savedAt", END_KEY,
                "directions.checked", "gate.closed", "gate.open", "arm.stowed", "arm.deployed",
                "beam.staged.brokenWhenLow", "beam.full.brokenWhenLow", "beams.checked", "queue.ticksPerBall",
                "intake.jamAmps"));
        for (String m : V2Config.MOTORS) k.add("dir." + m);
        for (String sv : V2Config.CR_SERVOS) k.add("dir." + sv);
        return k;
    }

    private Boolean readDir(Map<String, String> kv, String key) {
        String v = kv.get(key);
        if (v == null) return null;
        if (v.equalsIgnoreCase("FORWARD")) return false;
        if (v.equalsIgnoreCase("REVERSE")) return true;
        warnings.add(key + "=" + v + " is not FORWARD/REVERSE: FORWARD used");
        return null;
    }

    private Boolean readBool(Map<String, String> kv, String key) {
        String v = kv.get(key);
        if (v == null) return null;
        if (v.equalsIgnoreCase("true")) return true;
        if (v.equalsIgnoreCase("false")) return false;
        warnings.add(key + "=" + v + " is not true/false: default used");
        return null;
    }

    private Double readDouble(Map<String, String> kv, String key) {
        String v = kv.get(key);
        if (v == null) return null;
        try {
            double d = Double.parseDouble(v);
            if (Double.isNaN(d) || Double.isInfinite(d)) throw new NumberFormatException();
            return d;
        } catch (NumberFormatException e) {
            warnings.add(key + "=" + v + " is not a number: default used");
            return null;
        }
    }

    // ------------------------------------------------------------------ files
    /** loads {@code f}; never throws: a missing or unreadable file gives the defaults (with a warning if unreadable) */
    public static V2Calibration load(File f) {
        if (f != null && !f.exists()) {
            File tmp = new File(f.getAbsoluteFile().getParentFile(), f.getName() + ".tmp");
            if (tmp.exists()) {                        // power lost between deleting the old file and the rename
                V2Calibration c = load(tmp);
                c.warnings.add(0, "main file missing: read the last save from " + tmp.getName());
                return c;
            }
        }
        if (f == null || !f.exists()) {
            return new V2Calibration();
        }
        StringBuilder sb = new StringBuilder();
        try (BufferedReader r = new BufferedReader(new InputStreamReader(new FileInputStream(f), StandardCharsets.UTF_8))) {
            String line;
            while ((line = r.readLine()) != null) {
                sb.append(line).append('\n');
            }
        } catch (IOException | RuntimeException e) {
            V2Calibration c = new V2Calibration();
            c.warnings.add("could not read " + f.getName() + " (" + e.getMessage() + "): defaults used");
            return c;
        }
        V2Calibration c = parse(sb.toString());
        c.fromFile = true;
        return c;
    }

    /**
     * Saves to {@code f} safely: writes a temporary file next to it and then replaces the old one, so a crash or a
     * pulled battery mid-save leaves the previous calibration intact.
     */
    public void save(File f, String stamp) throws IOException {
        File dir = f.getAbsoluteFile().getParentFile();
        if (dir != null && !dir.exists() && !dir.mkdirs()) {
            throw new IOException("cannot create " + dir);
        }
        File tmp = new File(dir, f.getName() + ".tmp");
        try (FileOutputStream fos = new FileOutputStream(tmp);
             Writer w = new OutputStreamWriter(fos, StandardCharsets.UTF_8)) {
            w.write(format(stamp));
            w.flush();
            fos.getFD().sync();                                // on the storage before the old file is replaced
        }
        if (f.exists() && !f.delete()) {
            throw new IOException("cannot replace " + f);
        }
        if (!tmp.renameTo(f)) {
            throw new IOException("cannot rename " + tmp + " to " + f);
        }
        savedAt = stamp == null ? "" : stamp;
        fromFile = true;
    }

    /** the file on the Control Hub (/sdcard/biobuzz_v2_calibration.txt) */
    public static File robotFile() {
        return new File(org.firstinspires.ftc.robotcore.internal.system.AppUtil.ROOT_FOLDER, FILE_NAME);
    }
}

package org.firstinspires.ftc.teamcode.biobuzz;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/**
 * Settings kept on the Control Hub (/sdcard/biobuzz_teleop.txt) so they survive app restarts and new installs:
 * the last alliance, and the SHOT SPOT saved in TeleOp (camera distance to the HIVE + flywheel speed that scored).
 */
public final class TeleOpSettings {
    public static final String FILE_NAME = "biobuzz_teleop.txt";
    /**
     * Spot distances are the camera's FLOOR distance to the CELL opening (HiveVision.Target.horizontalIn), saved with
     * the printed hood's speeds. Older saves (SDK camera range, polycarbonate-hood speeds) are dropped.
     */
    static final String SPOT_REF = "floor";

    public HiveVision.Alliance alliance = HiveVision.Alliance.RED;
    /** Camera floor distance to the HIVE CELL opening at the saved spot (NaN = no spot saved). */
    public double spotRangeIn = Double.NaN;
    public double spotSpeed = Double.NaN;
    /** Aim trim, degrees: + moves the shots LEFT, - moves them RIGHT (camera vs launcher). */
    public double aimTrimDeg = 0;
    /** Windmill power while turning one pocket. */
    public double feedPower = BioBuzzConstants.INDEXER_FEED_POWER;
    /** Feed pause after each pocket before the next one may turn. */
    public double feedPauseSec = BioBuzzConstants.SHOT_MIN_INTERVAL_SEC;
    /**
     * TeleOp: after a pocket that launched nothing, the next one turns after this instead of the pause (ShotFeeder
     * skipEmptyPockets). Shorter = faster, but a late ball is more often followed by the next one. At or above the pause
     * it is off.
     */
    public double feedEmptyWaitSec = BioBuzzConstants.SHOT_EMPTY_WAIT_SEC;
    /**
     * Where AUTO shoots from, taught in TeleOp (SHARE) when TeleOp was INIT-ed at the AUTO start position:
     * Pinpoint inches forward / left of the start and heading (CCW +). NaN = not taught.
     */
    public double shootFwdIn = Double.NaN, shootLeftIn = Double.NaN, shootHeadingDeg = Double.NaN;

    public boolean hasShootPose() {
        return !Double.isNaN(shootFwdIn) && !Double.isNaN(shootLeftIn) && !Double.isNaN(shootHeadingDeg);
    }

    public boolean hasSpot() {
        return !Double.isNaN(spotRangeIn) && !Double.isNaN(spotSpeed);
    }

    static TeleOpSettings parse(String text) {
        TeleOpSettings s = new TeleOpSettings();
        String spotRef = "";
        for (String line : text.split("\\r?\\n")) {
            String[] kv = line.trim().split("=", 2);
            if (kv.length != 2) {
                continue;
            }
            String k = kv[0].trim();
            String v = kv[1].trim();
            try {
                if (k.equals("alliance")) {
                    s.alliance = HiveVision.Alliance.valueOf(v);
                } else if (k.equals("spot_range_in")) {
                    s.spotRangeIn = Double.parseDouble(v);
                } else if (k.equals("spot_speed")) {
                    s.spotSpeed = Double.parseDouble(v);
                } else if (k.equals("spot_ref")) {
                    spotRef = v;
                } else if (k.equals("aim_trim_deg")) {
                    s.aimTrimDeg = clipOr(Double.parseDouble(v), -AIM_TRIM_MAX_DEG, AIM_TRIM_MAX_DEG, 0);
                } else if (k.equals("feed_power")) {
                    s.feedPower = clipOr(Double.parseDouble(v), FEED_POWER_MIN, FEED_POWER_MAX, s.feedPower);
                } else if (k.equals("feed_pause_sec")) {
                    s.feedPauseSec = clipOr(Double.parseDouble(v), FEED_PAUSE_MIN, FEED_PAUSE_MAX, s.feedPauseSec);
                } else if (k.equals("feed_empty_wait_sec")) {
                    s.feedEmptyWaitSec = clipOr(Double.parseDouble(v), FEED_EMPTY_WAIT_MIN, FEED_EMPTY_WAIT_MAX,
                            s.feedEmptyWaitSec);
                } else if (k.equals("shoot_fwd_in")) {
                    s.shootFwdIn = clipOr(Double.parseDouble(v), -SHOOT_MAX_IN, SHOOT_MAX_IN, Double.NaN);
                } else if (k.equals("shoot_left_in")) {
                    s.shootLeftIn = clipOr(Double.parseDouble(v), -SHOOT_MAX_IN, SHOOT_MAX_IN, Double.NaN);
                } else if (k.equals("shoot_heading_deg")) {
                    s.shootHeadingDeg = clipOr(Double.parseDouble(v), -180, 180, Double.NaN);
                }
            } catch (IllegalArgumentException ignored) {
                // keep the default for a bad line
            }
        }
        if (!SPOT_REF.equals(spotRef) || !(s.spotRangeIn > 6 && s.spotRangeIn < 200)
                || !(s.spotSpeed > 0 && s.spotSpeed <= BioBuzzConstants.LAUNCHER_MAX_VELOCITY)) {
            s.spotRangeIn = Double.NaN;
            s.spotSpeed = Double.NaN;
        }
        if (!SPOT_REF.equals(spotRef)) {
            // taught with the polycarbonate hood (and its speeds): AUTO must be taught again (LB + Share)
            s.shootFwdIn = Double.NaN;
            s.shootLeftIn = Double.NaN;
            s.shootHeadingDeg = Double.NaN;
        }
        return s;
    }

    static final double AIM_TRIM_MAX_DEG = 10;
    static final double FEED_POWER_MIN = 0.25, FEED_POWER_MAX = 0.8;
    static final double FEED_PAUSE_MIN = 0.1, FEED_PAUSE_MAX = 2.0;
    static final double FEED_EMPTY_WAIT_MIN = 0.05, FEED_EMPTY_WAIT_MAX = 0.5;
    /** AUTO never drives further than this from its start to a taught shooting position. */
    static final double SHOOT_MAX_IN = 72;

    /** v when it is inside [lo, hi], otherwise the fallback (a bad value in the file must not reach the robot). */
    private static double clipOr(double v, double lo, double hi, double fallback) {
        return v >= lo && v <= hi ? v : fallback;
    }

    String format() {
        // spot_ref is written every time: it marks the file as printed-hood era (see parse)
        StringBuilder sb = new StringBuilder("alliance=" + alliance + "\nspot_ref=" + SPOT_REF + "\n");
        if (hasSpot()) {
            sb.append(String.format(Locale.US, "spot_range_in=%.1f\nspot_speed=%.0f\n", spotRangeIn, spotSpeed));
        }
        sb.append(String.format(Locale.US, "aim_trim_deg=%.1f\nfeed_power=%.2f\nfeed_pause_sec=%.2f\nfeed_empty_wait_sec=%.2f\n",
                aimTrimDeg, feedPower, feedPauseSec, feedEmptyWaitSec));
        if (hasShootPose()) {
            sb.append(String.format(Locale.US, "shoot_fwd_in=%.1f\nshoot_left_in=%.1f\nshoot_heading_deg=%.1f\n",
                    shootFwdIn, shootLeftIn, shootHeadingDeg));
        }
        return sb.toString();
    }

    public static TeleOpSettings load() {
        try {
            File f = file();
            if (f.exists()) {
                StringBuilder sb = new StringBuilder();
                try (BufferedReader r = new BufferedReader(new FileReader(f))) {
                    String line;
                    while ((line = r.readLine()) != null) {
                        sb.append(line).append('\n');
                    }
                }
                return parse(sb.toString());
            }
        } catch (Throwable ignored) {
            // no robot storage (laptop tests)
        }
        return new TeleOpSettings();
    }

    public void save() throws IOException {
        try (FileWriter w = new FileWriter(file())) {
            w.write(format());
        }
    }

    /** Saves without throwing (a failed write must never stop an OpMode). */
    public boolean trySave() {
        try {
            save();
            return true;
        } catch (Throwable e) {
            return false;
        }
    }

    private static File file() {
        return new File(org.firstinspires.ftc.robotcore.internal.system.AppUtil.ROOT_FOLDER, FILE_NAME);
    }
}

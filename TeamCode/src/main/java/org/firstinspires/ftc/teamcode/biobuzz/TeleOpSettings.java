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

    public HiveVision.Alliance alliance = HiveVision.Alliance.RED;
    /** Camera range to the HIVE tags at the saved spot (NaN = no spot saved). */
    public double spotRangeIn = Double.NaN;
    public double spotSpeed = Double.NaN;

    public boolean hasSpot() {
        return !Double.isNaN(spotRangeIn) && !Double.isNaN(spotSpeed);
    }

    static TeleOpSettings parse(String text) {
        TeleOpSettings s = new TeleOpSettings();
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
                }
            } catch (IllegalArgumentException ignored) {
                // keep the default for a bad line
            }
        }
        if (!(s.spotRangeIn > 6 && s.spotRangeIn < 200) || !(s.spotSpeed > 0 && s.spotSpeed <= BioBuzzConstants.LAUNCHER_MAX_VELOCITY)) {
            s.spotRangeIn = Double.NaN;
            s.spotSpeed = Double.NaN;
        }
        return s;
    }

    String format() {
        StringBuilder sb = new StringBuilder("alliance=" + alliance + "\n");
        if (hasSpot()) {
            sb.append(String.format(Locale.US, "spot_range_in=%.1f\nspot_speed=%.0f\n", spotRangeIn, spotSpeed));
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

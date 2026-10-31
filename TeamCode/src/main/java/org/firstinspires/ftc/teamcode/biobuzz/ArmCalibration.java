package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/**
 * Flower-arm servo positions saved on the Control Hub by "BioBuzz Arm Setup" (/sdcard/biobuzz_arm.txt),
 * so the team never has to edit code to calibrate the arm. Falls back to ARM_DEPLOY_POSITION / ARM_STOW_DIRECTION
 * in BioBuzzConstants when no file has been saved.
 */
public final class ArmCalibration {
    public static final String FILE_NAME = "biobuzz_arm.txt";

    private static boolean loaded = false;
    private static double deploy = ARM_DEPLOY_POSITION;
    private static int direction = ARM_STOW_DIRECTION;
    private static boolean fromFile = false;

    private ArmCalibration() {
    }

    public static synchronized double deployPosition() {
        ensureLoaded();
        return deploy;
    }

    public static synchronized int stowDirection() {
        ensureLoaded();
        return direction;
    }

    public static synchronized boolean savedOnRobot() {
        ensureLoaded();
        return fromFile;
    }

    public static double stowPosition(double deployPos, int dir) {
        return deployPos + dir * ARM_STOW_OFFSET;
    }

    public static boolean valid(double deployPos, int dir) {
        double stow = stowPosition(deployPos, dir);
        return deployPos >= 0 && deployPos <= 1 && stow >= 0 && stow <= 1 && (dir == 1 || dir == -1);
    }

    /** Re-reads the file (called whenever a BioBuzzRobot is created, so every OpMode sees the latest save). */
    public static synchronized void reload() {
        loaded = false;
        ensureLoaded();
    }

    public static synchronized void save(double deployPos, int dir) throws IOException {
        File f = file();
        try (FileWriter w = new FileWriter(f)) {
            w.write(format(deployPos, dir));
        }
        deploy = deployPos;
        direction = dir;
        fromFile = true;
        loaded = true;
    }

    /**
     * Arm fitted level at servo 0.50, then jogged DOWN onto the pads to position {@code downPos}:
     * returns {deploy (0.002 into the pads), stow direction}, or null if the arm did not move far enough or
     * the up position would be out of the servo range.
     */
    static double[] fromDownPosition(double downPos) {
        double moved = downPos - SETUP_LEVEL_POSITION;
        if (Math.abs(moved) < 0.15) {
            return null;
        }
        int sign = moved > 0 ? 1 : -1;
        double d = Math.max(0, Math.min(1, downPos + sign * 0.002));
        int dir = -sign;
        return valid(d, dir) ? new double[]{d, dir} : null;
    }

    public static final double SETUP_LEVEL_POSITION = 0.5;

    /** "deploy=0.712\ndirection=-1" -> {0.712, -1}; null if missing or malformed. */
    static double[] parse(String text) {
        Double d = null;
        Integer dir = null;
        for (String line : text.split("\\r?\\n")) {
            String[] kv = line.trim().split("=", 2);
            if (kv.length != 2) {
                continue;
            }
            try {
                if (kv[0].trim().equals("deploy")) {
                    d = Double.parseDouble(kv[1].trim());
                } else if (kv[0].trim().equals("direction")) {
                    dir = Integer.parseInt(kv[1].trim());
                }
            } catch (NumberFormatException e) {
                return null;
            }
        }
        if (d == null || dir == null || !valid(d, dir)) {
            return null;
        }
        return new double[]{d, dir};
    }

    static String format(double deployPos, int dir) {
        return String.format(Locale.US, "deploy=%.4f\ndirection=%d\n", deployPos, dir);
    }

    private static void ensureLoaded() {
        if (loaded) {
            return;
        }
        loaded = true;
        deploy = ARM_DEPLOY_POSITION;
        direction = ARM_STOW_DIRECTION;
        fromFile = false;
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
                double[] v = parse(sb.toString());
                if (v != null) {
                    deploy = v[0];
                    direction = (int) v[1];
                    fromFile = true;
                }
            }
        } catch (Throwable ignored) {
            // no robot storage (e.g. laptop tests): use the constants
        }
    }

    private static File file() {
        return new File(org.firstinspires.ftc.robotcore.internal.system.AppUtil.ROOT_FOLDER, FILE_NAME);
    }
}

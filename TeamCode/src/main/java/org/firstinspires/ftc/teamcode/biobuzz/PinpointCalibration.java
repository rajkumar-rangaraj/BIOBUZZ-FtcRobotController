package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/**
 * Pinpoint pod directions and positions, measured on the robot by "BioBuzz Pinpoint Setup" and saved on the Control
 * Hub (/sdcard/biobuzz_pinpoint.txt), so the pods can sit wherever they fit. Falls back to the PINPOINT_* constants.
 * Offsets use goBILDA's convention: X (forward) pod LEFT of the turning centre = +, Y (strafe) pod FORWARD = +.
 */
public final class PinpointCalibration {
    public static final String FILE_NAME = "biobuzz_pinpoint.txt";

    /** goBILDA pod resolutions (ticks per mm). */
    public static final double TICKS_PER_MM_4_BAR = 19.89436789;
    public static final double TICKS_PER_MM_SWINGARM = 13.26291192;

    public final boolean xReversed, yReversed;
    public final double xPodLeftMm, yPodForwardMm;
    public final boolean fromFile;

    PinpointCalibration(boolean xReversed, boolean yReversed, double xPodLeftMm, double yPodForwardMm, boolean fromFile) {
        this.xReversed = xReversed;
        this.yReversed = yReversed;
        this.xPodLeftMm = xPodLeftMm;
        this.yPodForwardMm = yPodForwardMm;
        this.fromFile = fromFile;
    }

    public static double ticksPerMm() {
        return PINPOINT_SWINGARM_PODS ? TICKS_PER_MM_SWINGARM : TICKS_PER_MM_4_BAR;
    }

    /** Saved values, or the constants if nothing (valid) has been saved. */
    public static PinpointCalibration load() {
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
                PinpointCalibration c = parse(sb.toString());
                if (c != null) {
                    return c;
                }
            }
        } catch (Throwable ignored) {
            // no robot storage (laptop tests): use the constants
        }
        return new PinpointCalibration(PINPOINT_X_REVERSED, PINPOINT_Y_REVERSED, PINPOINT_X_POD_LEFT_MM,
                PINPOINT_Y_POD_FORWARD_MM, false);
    }

    public void save() throws IOException {
        try (FileWriter w = new FileWriter(file())) {
            w.write(format());
        }
    }

    String format() {
        return String.format(Locale.US, "x_reversed=%b\ny_reversed=%b\nx_pod_left_mm=%.1f\ny_pod_forward_mm=%.1f\n",
                xReversed, yReversed, xPodLeftMm, yPodForwardMm);
    }

    static PinpointCalibration parse(String text) {
        Boolean xr = null, yr = null;
        Double xl = null, yf = null;
        for (String line : text.split("\\r?\\n")) {
            String[] kv = line.trim().split("=", 2);
            if (kv.length != 2) {
                continue;
            }
            String k = kv[0].trim(), v = kv[1].trim();
            try {
                if (k.equals("x_reversed")) {
                    xr = Boolean.parseBoolean(v);
                } else if (k.equals("y_reversed")) {
                    yr = Boolean.parseBoolean(v);
                } else if (k.equals("x_pod_left_mm")) {
                    xl = Double.parseDouble(v);
                } else if (k.equals("y_pod_forward_mm")) {
                    yf = Double.parseDouble(v);
                }
            } catch (NumberFormatException e) {
                return null;
            }
        }
        if (xr == null || yr == null || xl == null || yf == null || Math.abs(xl) > 300 || Math.abs(yf) > 300) {
            return null;
        }
        return new PinpointCalibration(xr, yr, xl, yf, true);
    }

    // ---------------- measurement math (raw encoder ticks with both directions set FORWARD) ----------------

    /** Robot pushed straight FORWARD: the forward pod must move much more than the strafe pod. */
    public static String forwardCheck(int dX, int dY) {
        if (Math.abs(dX) < 2000 && Math.abs(dY) < 2000) {
            return "pods barely moved - check both pod wheels touch the floor and the pod cables, then try again";
        }
        if (Math.abs(dY) > Math.abs(dX)) {
            return "SWAP the two pod cables: the pod in port X must be the one that rolls when the robot goes FORWARD";
        }
        return null;
    }

    /** Robot pushed straight LEFT: the strafe pod must move much more than the forward pod. */
    public static String leftCheck(int dX, int dY) {
        if (Math.abs(dX) < 2000 && Math.abs(dY) < 2000) {
            return "pods barely moved - check both pod wheels touch the floor and the pod cables, then try again";
        }
        if (Math.abs(dX) > Math.abs(dY)) {
            return "SWAP the two pod cables: the pod in port Y must be the one that rolls when the robot slides SIDEWAYS";
        }
        return null;
    }

    /**
     * Robot spun in place by {@code turnedRad} (CCW +) with the encoder signs already corrected (forward + / left +).
     * A pod LEFT of the turning centre by L moves BACKWARD by L*theta on a CCW turn; a pod FORWARD by F moves LEFT by F*theta.
     * @return {xPodLeftMm, yPodForwardMm}
     */
    public static double[] offsetsFromSpin(double xTicks, double yTicks, double turnedRad, double ticksPerMm) {
        double xMm = xTicks / ticksPerMm, yMm = yTicks / ticksPerMm;
        return new double[]{-xMm / turnedRad, yMm / turnedRad};
    }

    private static File file() {
        return new File(org.firstinspires.ftc.robotcore.internal.system.AppUtil.ROOT_FOLDER, FILE_NAME);
    }
}

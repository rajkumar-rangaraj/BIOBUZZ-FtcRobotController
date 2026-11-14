package org.firstinspires.ftc.teamcode.auto;

import java.util.List;

/**
 * G417: never launch into a downward CELL. Is our CELL up? From the camera height of the CELL (HiveVision.Target
 * heightIn) over the last frames of the aim. No FTC SDK dependency: laptop-tested in CellCheckTest.
 */
public final class CellCheck {
    private CellCheck() {
    }

    /**
     * Camera heights at the S1 shot spot: UP read 46.1-46.3 in (9 Nov), DOWN 29.3-32.1 in (13 Nov). The threshold sits
     * half-way, ~7 in from both (the old 45 in left only 1 in for an UP CELL: one low reading and AUTO would keep all 4
     * pre-loads). CALIBRATE: every AUTO logs height_in.
     */
    public static final double UP_MIN_HEIGHT_IN = 39.0;
    /** The verdict uses the median of the last this-many frames of our CELL, so one odd frame can't flip it. */
    public static final int FRAMES = 5;
    /**
     * The CELL in front of the shot spot is at most this far (camera floor distance): the CELL in front read 36-39 in
     * (9 and 13 Nov), the far CELL of our HIVE 55-69 in (13-14 Nov). AUTO shoots only at the CELL in front.
     */
    public static final double FRONT_MAX_FLOOR_IN = 47.0;

    /** Is a CELL at this camera floor distance the one in front of the shot spot (NaN: no)? */
    public static boolean front(double floorIn) {
        return floorIn <= FRONT_MAX_FLOOR_IN;
    }

    /** Median of the last {@link #FRAMES} heights (NaN when there are none). */
    public static double median(List<Double> heightsIn) {
        int n = Math.min(FRAMES, heightsIn.size());
        if (n == 0) {
            return Double.NaN;
        }
        double[] h = new double[n];
        for (int i = 0; i < n; i++) {
            h[i] = heightsIn.get(heightsIn.size() - n + i);
        }
        java.util.Arrays.sort(h);
        return n % 2 == 1 ? h[n / 2] : (h[n / 2 - 1] + h[n / 2]) / 2;
    }

    /** True when the CELL is up (a NaN height, i.e. no frames, is not up). */
    public static boolean up(double medianHeightIn) {
        return medianHeightIn >= UP_MIN_HEIGHT_IN;
    }

    /**
     * The verdict over an aim's frames (oldest first): UP only when the median of the last {@link #FRAMES} AND the latest
     * frame are both up. A latest frame that reads DOWN vetoes the shot (the CELL may just have tipped): G417 fails safe.
     */
    public static boolean up(List<Double> heightsIn) {
        return !heightsIn.isEmpty() && up(median(heightsIn)) && up(heightsIn.get(heightsIn.size() - 1));
    }
}

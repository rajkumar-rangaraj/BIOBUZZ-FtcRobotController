package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Finds our ALLIANCE's HIVE CELL with the rear (launcher-side) camera.
 *
 * Each CELL has a 4-tag cluster on its downward-facing bottom. SDK 12 reports each cluster as ONE detection whose
 * pose is the centre of the CELL opening, even when only some of its tags are visible (averaging the visible tags
 * aimed at a corner: 31 Oct, always 2 of 4 tags seen). The cluster with the most visible tags is chosen.
 */
public class HiveVision {

    public enum Alliance { RED, BLUE }

    public enum Cell {
        RED_REAR(30, "RED SCORING"), RED_AUDIENCE(34, "RED AUDIENCE"), BLUE_AUDIENCE(38, "BLUE AUDIENCE"),
        BLUE_REAR(42, "BLUE SCORING");

        public final int firstId;
        /** Cluster name in the SDK 12 BIOBUZZ tag library. */
        public final String clusterName;

        Cell(int firstId, String clusterName) {
            this.firstId = firstId;
            this.clusterName = clusterName;
        }

        static Cell forCluster(String name) {
            for (Cell c : values()) {
                if (c.clusterName.equals(name)) {
                    return c;
                }
            }
            return null;
        }

        boolean belongsTo(Alliance alliance) {
            return (alliance == Alliance.RED) == (this == RED_REAR || this == RED_AUDIENCE);
        }

        /**
         * The same CELL on the other alliance's HIVE. The field is point-symmetric (the AUTO route is the same for both),
         * so RED AUDIENCE pairs with BLUE SCORING (the two CELLs that start upward, {@link #startingUpCell}) and RED
         * SCORING with BLUE AUDIENCE.
         */
        public Cell otherAlliance() {
            switch (this) {
                case RED_REAR:
                    return BLUE_AUDIENCE;
                case RED_AUDIENCE:
                    return BLUE_REAR;
                case BLUE_AUDIENCE:
                    return RED_REAR;
                default:
                    return RED_AUDIENCE;
            }
        }
    }

    /** The CELL that faces upward at the start of every MATCH (Competition Manual, Figure 10-2). */
    public static Cell startingUpCell(Alliance alliance) {
        return alliance == Alliance.RED ? Cell.RED_AUDIENCE : Cell.BLUE_REAR;
    }

    public static class Target {
        public final Cell cell;
        public final int tagCount;
        /**
         * Degrees to aim at, aim trim included; positive means turn counter-clockwise. Aiming drives it to 0, which
         * leaves the camera {@code -aimTrim} off the CELL: + trim turns the robot (and the shots) LEFT.
         */
        public final double bearingDeg;
        /** Camera bearing to the centre of the CELL opening, no trim. */
        public final double rawBearingDeg;
        /** SDK range (camera-frame horizontal distance, not straight-line). */
        public final double rangeIn;
        public final double elevationDeg;
        /** Mean tag position in the camera frame: x right, y along the lens axis, z up (tilted with the camera). */
        public final double xIn;
        public final double yIn;
        public final double zIn;

        Target(Cell cell, int tagCount, double rawBearingDeg, double aimTrimDeg, double rangeIn, double elevationDeg,
               double xIn, double yIn, double zIn) {
            this.cell = cell;
            this.tagCount = tagCount;
            this.rawBearingDeg = rawBearingDeg;
            this.bearingDeg = rawBearingDeg + aimTrimDeg;
            this.rangeIn = rangeIn;
            this.elevationDeg = elevationDeg;
            this.xIn = xIn;
            this.yIn = yIn;
            this.zIn = zIn;
        }

        /** Floor distance from the camera lens to the point under the tags. */
        public double horizontalIn() {
            double tilt = Math.toRadians(CAMERA_TILT_DEG);
            return Math.hypot(xIn, yIn * Math.cos(tilt) - zIn * Math.sin(tilt));
        }

        /** Height of the tags above the floor. */
        public double heightIn() {
            double tilt = Math.toRadians(CAMERA_TILT_DEG);
            return CAMERA_LENS_HEIGHT_IN + yIn * Math.sin(tilt) + zIn * Math.cos(tilt);
        }
    }

    private final AprilTagProcessor tags;
    private final VisionPortal portal;
    private final ElapsedTime sinceFrame = new ElapsedTime();
    private List<AprilTagDetection> latest = null;
    private long frames = 0;
    private double aimTrimDeg = 0;

    public HiveVision(HardwareMap hw) {
        tags = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())   // CELL clusters 30-45, origin = CELL opening
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, WEBCAM))
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .addProcessor(tags)
                .build();
    }

    /** Call once per loop. Returns true when a new camera frame was processed. */
    public boolean update() {
        List<AprilTagDetection> fresh = tags.getFreshDetections();
        if (fresh != null) {
            latest = fresh;
            sinceFrame.reset();
            frames++;
            return true;
        }
        return false;
    }

    /** All detections from the most recent frame (may be empty). */
    public List<AprilTagDetection> latestDetections() {
        return latest == null ? new java.util.ArrayList<AprilTagDetection>() : latest;
    }

    public long framesProcessed() {
        return frames;
    }

    public boolean cameraStreaming() {
        return portal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    /** Aim trim from "BioBuzz Tune" (+ = shots LEFT), applied to every {@link Target#bearingDeg}. */
    public void setAimTrimDeg(double deg) {
        aimTrimDeg = deg;
    }

    public double aimTrimDeg() {
        return aimTrimDeg;
    }

    /** Best visible CELL of our ALLIANCE (SDK 12 cluster = centre of the CELL opening), or null if nothing recent. */
    public Target target(Alliance alliance, Cell preferred) {
        if (latest == null || sinceFrame.seconds() > TARGET_MAX_AGE_SEC) {
            return null;
        }
        Target best = null;
        for (AprilTagDetection det : latest) {
            Target t = toTarget(det);
            if (t == null || !t.cell.belongsTo(alliance)) {
                continue;
            }
            if (best == null || t.tagCount > best.tagCount
                    || (t.tagCount == best.tagCount && t.cell == preferred)) {
                best = t;
            }
        }
        return best;
    }

    /**
     * Only this CELL of our ALLIANCE, or null when it is not in a recent frame. Unlike {@link #target}, another CELL with
     * more visible tags can't hide it (AUTO aims at one CELL only).
     */
    public Target targetCell(Alliance alliance, Cell cell) {
        if (latest == null || sinceFrame.seconds() > TARGET_MAX_AGE_SEC || !cell.belongsTo(alliance)) {
            return null;
        }
        Target best = null;
        for (AprilTagDetection det : latest) {
            Target t = toTarget(det);
            if (t != null && t.cell == cell && (best == null || t.tagCount > best.tagCount)) {
                best = t;
            }
        }
        return best;
    }

    /** A CELL cluster detection as a Target (null for single tags, unknown clusters or no pose). */
    private Target toTarget(AprilTagDetection det) {
        if (!(det instanceof AprilTagClusterDetection) || det.ftcPose == null) {
            return null;
        }
        AprilTagClusterDetection d = (AprilTagClusterDetection) det;
        Cell cell = d.metadata == null ? null : Cell.forCluster(d.metadata.name);
        if (cell == null) {
            return null;
        }
        int count = Math.max(1, (int) Math.round(d.percentClusterFound * 4 / 100.0));
        return new Target(cell, count, d.ftcPose.bearing, aimTrimDeg, d.ftcPose.range, d.ftcPose.elevation,
                d.ftcPose.x, d.ftcPose.y, d.ftcPose.z);
    }

    public int visibleTagCount() {
        return latest == null || sinceFrame.seconds() > TARGET_MAX_AGE_SEC ? 0 : latest.size();
    }

    /** Clockwise turn power that rotates the launcher toward a target at this bearing. */
    public static double aimTurnCw(double bearingDeg) {
        return MotionController.aimTurnCw(bearingDeg);
    }

    public void close() {
        portal.close();
    }
}

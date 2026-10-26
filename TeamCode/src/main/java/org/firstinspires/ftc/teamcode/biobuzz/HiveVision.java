package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Finds our ALLIANCE's HIVE CELL with the rear (launcher-side) camera.
 *
 * Each CELL has a 4-tag cluster on its downward-facing bottom. The upward-facing CELL is the one that
 * can be seen from the floor, so the cluster with the most visible tags is chosen.
 */
public class HiveVision {

    public enum Alliance { RED, BLUE }

    public enum Cell {
        RED_REAR(30), RED_AUDIENCE(34), BLUE_AUDIENCE(38), BLUE_REAR(42);

        public final int firstId;

        Cell(int firstId) {
            this.firstId = firstId;
        }

        boolean contains(int id) {
            return id >= firstId && id < firstId + 4;
        }

        boolean belongsTo(Alliance alliance) {
            return (alliance == Alliance.RED) == (this == RED_REAR || this == RED_AUDIENCE);
        }
    }

    /** The CELL that faces upward at the start of every MATCH (Competition Manual, Figure 10-2). */
    public static Cell startingUpCell(Alliance alliance) {
        return alliance == Alliance.RED ? Cell.RED_AUDIENCE : Cell.BLUE_REAR;
    }

    public static class Target {
        public final Cell cell;
        public final int tagCount;
        /** Degrees; positive means the CELL is left of the camera centre, so the robot must turn counter-clockwise. */
        public final double bearingDeg;
        /** SDK range (camera-frame horizontal distance, not straight-line). */
        public final double rangeIn;
        public final double elevationDeg;
        /** Mean tag position in the camera frame: x right, y along the lens axis, z up (tilted with the camera). */
        public final double xIn;
        public final double yIn;
        public final double zIn;

        Target(Cell cell, int tagCount, double bearingDeg, double rangeIn, double elevationDeg,
               double xIn, double yIn, double zIn) {
            this.cell = cell;
            this.tagCount = tagCount;
            this.bearingDeg = bearingDeg;
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

    public HiveVision(HardwareMap hw) {
        AprilTagLibrary.Builder library = new AprilTagLibrary.Builder();
        for (Cell cell : Cell.values()) {
            for (int i = 0; i < 4; i++) {
                int id = cell.firstId + i;
                library.addTag(id, cell.name() + "_" + id, TAG_SIZE_IN, DistanceUnit.INCH);
            }
        }
        tags = new AprilTagProcessor.Builder()
                .setTagLibrary(library.build())
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

    /** Best visible CELL of our ALLIANCE, or null if nothing recent was seen. */
    public Target target(Alliance alliance, Cell preferred) {
        if (latest == null || sinceFrame.seconds() > TARGET_MAX_AGE_SEC) {
            return null;
        }
        Target best = null;
        for (Cell cell : Cell.values()) {
            if (!cell.belongsTo(alliance)) {
                continue;
            }
            int count = 0;
            double bearing = 0;
            double range = 0;
            double elevation = 0;
            double x = 0;
            double y = 0;
            double z = 0;
            for (AprilTagDetection d : latest) {
                if (d.metadata != null && d.ftcPose != null && cell.contains(d.id)) {
                    count++;
                    bearing += d.ftcPose.bearing;
                    range += d.ftcPose.range;
                    elevation += d.ftcPose.elevation;
                    x += d.ftcPose.x;
                    y += d.ftcPose.y;
                    z += d.ftcPose.z;
                }
            }
            if (count == 0) {
                continue;
            }
            Target t = new Target(cell, count, bearing / count, range / count, elevation / count,
                    x / count, y / count, z / count);
            if (best == null || t.tagCount > best.tagCount
                    || (t.tagCount == best.tagCount && cell == preferred)) {
                best = t;
            }
        }
        return best;
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

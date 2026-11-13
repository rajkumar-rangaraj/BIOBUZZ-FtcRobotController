package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

/**
 * Both CELLs of our HIVE (HIVE AUTO, 12 Nov): {@link HiveVision#target} returns only the best one, the HIVE AUTO needs
 * each of them (did the HIVE tip? is the far CELL up?). Same cluster handling as HiveVision.target.
 */
public final class HiveCells {
    private HiveCells() {
    }

    /** Every CELL of our ALLIANCE in the latest frame. Call right after {@link HiveVision#update()} returned true. */
    public static List<HiveVision.Target> visible(HiveVision vision, HiveVision.Alliance alliance) {
        List<HiveVision.Target> out = new ArrayList<>();
        for (AprilTagDetection det : vision.latestDetections()) {
            if (!(det instanceof AprilTagClusterDetection) || det.ftcPose == null) {
                continue;
            }
            AprilTagClusterDetection d = (AprilTagClusterDetection) det;
            HiveVision.Cell cell = d.metadata == null ? null : HiveVision.Cell.forCluster(d.metadata.name);
            if (cell == null || !cell.belongsTo(alliance)) {
                continue;
            }
            int count = Math.max(1, (int) Math.round(d.percentClusterFound * 4 / 100.0));
            out.add(new HiveVision.Target(cell, count, d.ftcPose.bearing, vision.aimTrimDeg(), d.ftcPose.range,
                    d.ftcPose.elevation, d.ftcPose.x, d.ftcPose.y, d.ftcPose.z));
        }
        return out;
    }

    /** The CELL at the other end of the same HIVE (the one that comes up when this one tips down). */
    public static HiveVision.Cell otherEnd(HiveVision.Cell cell) {
        switch (cell) {
            case RED_AUDIENCE:
                return HiveVision.Cell.RED_REAR;
            case RED_REAR:
                return HiveVision.Cell.RED_AUDIENCE;
            case BLUE_AUDIENCE:
                return HiveVision.Cell.BLUE_REAR;
            default:
                return HiveVision.Cell.BLUE_AUDIENCE;
        }
    }
}

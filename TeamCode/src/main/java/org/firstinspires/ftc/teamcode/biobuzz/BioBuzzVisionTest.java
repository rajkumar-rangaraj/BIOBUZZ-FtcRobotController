package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/*
 * Vision test - measures how well the rear camera sees our HIVE CELL, against tape-measured truth.
 * Log: /sdcard/BioBuzzLogs/vision_*.csv
 *
 *   Sticks           drive at 30% to reposition
 *   X / Square(hold) closed-loop aim (same code as TeleOp/AUTO) - watch it settle
 *   D-pad up/down    TRUE floor distance, lens to the point under the CELL tags (+/- 6 in)
 *   D-pad right/left TRUE angle to the CELL, left = + (+/- 5 deg)
 *   A / Cross        record a 3 s capture with that truth
 *   B / Circle       switch alliance
 *
 * Each capture reports: detection rate, bearing mean and spread, distance error, tag height.
 */
@TeleOp(name = "BioBuzz Vision Test", group = "BioBuzz Test")
public class BioBuzzVisionTest extends OpMode {
    private static final double CAPTURE_SEC = 3.0;

    private BioBuzzRobot robot;
    private HiveVision vision;
    private CsvLog log;
    private HiveVision.Alliance alliance = BioBuzzMatchState.alliance;
    private double trueDistIn = 48;
    private double trueAngleDeg = 0;

    private int capture = 0;
    private boolean capturing = false;
    private final ElapsedTime captureTimer = new ElapsedTime();
    private int frames, hits;
    private double sumB, sumB2, sumH, sumZ;
    private String lastSummary = "none yet";

    @Override
    public void init() {
        robot = new BioBuzzRobot(hardwareMap);
        vision = new HiveVision(hardwareMap);
        log = new CsvLog("vision", "t", "capture", "true_dist_in", "true_angle_deg", "moving", "has_target",
                "cell", "tags", "bearing_deg", "range_in", "elevation_deg", "horiz_est_in", "height_est_in",
                "heading_deg", "all_ids");
    }

    @Override
    public void loop() {
        boolean newFrame = vision.update();

        if (gamepad1.dpadUpWasPressed()) {
            trueDistIn += 6;
        } else if (gamepad1.dpadDownWasPressed()) {
            trueDistIn = Math.max(0, trueDistIn - 6);
        } else if (gamepad1.dpadRightWasPressed()) {
            trueAngleDeg -= 5;
        } else if (gamepad1.dpadLeftWasPressed()) {
            trueAngleDeg += 5;
        }
        if (gamepad1.bWasPressed()) {
            alliance = alliance == HiveVision.Alliance.RED ? HiveVision.Alliance.BLUE : HiveVision.Alliance.RED;
        }
        if (gamepad1.aWasPressed() && !capturing) {
            capture++;
            capturing = true;
            captureTimer.reset();
            frames = hits = 0;
            sumB = sumB2 = sumH = sumZ = 0;
        }

        HiveVision.Target target = vision.target(alliance, null);
        double turn = gamepad1.right_stick_x;
        if (gamepad1.x && target != null) {
            turn = HiveVision.aimTurnCw(target.bearingDeg) / 0.3;
        }
        boolean moving = Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y) + Math.abs(turn) > 0.05;
        robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, turn, 0.3);

        if (capturing && newFrame) {
            frames++;
            StringBuilder ids = new StringBuilder();
            for (AprilTagDetection d : vision.latestDetections()) {
                ids.append(d.id).append(' ');
            }
            if (target != null) {
                hits++;
                sumB += target.bearingDeg;
                sumB2 += target.bearingDeg * target.bearingDeg;
                sumH += target.horizontalIn();
                sumZ += target.heightIn();
                log.row(getRuntime(), capture, trueDistIn, trueAngleDeg, moving, 1, target.cell, target.tagCount,
                        target.bearingDeg, target.rangeIn, target.elevationDeg, target.horizontalIn(),
                        target.heightIn(), robot.headingDeg(), ids.toString().trim());
            } else {
                log.row(getRuntime(), capture, trueDistIn, trueAngleDeg, moving, 0, "", 0, "", "", "", "", "",
                        robot.headingDeg(), ids.toString().trim());
            }
        }
        if (capturing && captureTimer.seconds() >= CAPTURE_SEC) {
            capturing = false;
            if (hits == 0) {
                lastSummary = String.format("#%d at %.0f in / %.0f deg: %d frames, NO detections", capture,
                        trueDistIn, trueAngleDeg, frames);
            } else {
                double meanB = sumB / hits;
                double sdB = Math.sqrt(Math.max(0, sumB2 / hits - meanB * meanB));
                lastSummary = String.format("#%d: seen %d/%d frames (%.0f%%) | bearing %.1f (true %.0f, spread %.1f)"
                                + " | dist %.1f (true %.0f, err %+.1f) | tag height %.1f in",
                        capture, hits, frames, 100.0 * hits / Math.max(1, frames), meanB, trueAngleDeg, sdB,
                        sumH / hits, trueDistIn, sumH / hits - trueDistIn, sumZ / hits);
            }
            log.row(getRuntime(), capture, trueDistIn, trueAngleDeg, "SUMMARY", lastSummary);
        }

        telemetry.addData("Alliance (B)", alliance);
        telemetry.addData("Truth", "%.0f in floor distance, %.0f deg (D-pad)", trueDistIn, trueAngleDeg);
        telemetry.addData("Capture (A)", capturing ? String.format("RECORDING %.1f s", captureTimer.seconds()) : "ready");
        if (target == null) {
            telemetry.addData("Now", "no %s CELL in view (%d tags of any kind)", alliance, vision.visibleTagCount());
        } else {
            telemetry.addData("Now", "%s %d tags | bearing %.1f | range %.1f | floor %.1f | height %.1f",
                    target.cell, target.tagCount, target.bearingDeg, target.rangeIn, target.horizontalIn(),
                    target.heightIn());
        }
        telemetry.addData("Last capture", lastSummary);
        telemetry.addData("Camera", vision.cameraStreaming() ? "streaming, %d frames" : "NOT streaming", vision.framesProcessed());
        telemetry.addData("Log", log.status());
    }

    @Override
    public void stop() {
        robot.stopAll();
        vision.close();
        log.close();
    }
}

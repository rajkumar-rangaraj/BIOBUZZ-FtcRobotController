package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/*
 * Calibration - finds the numbers marked CALIBRATE in BioBuzzConstants. Not for matches.
 *
 *   Sticks           drive at 30% (check every direction is correct)
 *   X / Square       zero drive encoders and IMU heading
 *   RB / LB (hold)   windmill slowly forward / backward (check FIRE direction, ticks per pocket)
 *   A / Cross        start arm jogging (arm moves to the saved DEPLOY value, or 0.50 if none) - hold the arm!
 *                    For a first-time arm setup use "BioBuzz Arm Setup" instead: it guides and saves.
 *   D-pad up/down    arm +/- 0.010       D-pad right/left   arm +/- 0.002
 *   Y / Triangle     arm: toggle between the jog value and the computed STOW position
 *   B / Circle       flip the STOW direction used by Y
 *
 * Drive: zero, push the robot exactly 48 in forward along a tape measure, read "Forward in".
 *   WHEEL_DIAMETER_IN *= 48 / reported. Then strafe 48 in: STRAFE_MULTIPLIER *= reported / 48.
 * IMU: turn the robot counter-clockwise by hand; heading must INCREASE. Otherwise fix HUB_LOGO / HUB_USB.
 * Arm: jog until both arms just touch the deploy pads, add 0.002 into the stop, check Y swings to STOW
 *   without stalling, then copy "Jog position" to ARM_DEPLOY_POSITION and the direction to ARM_STOW_DIRECTION.
 * Camera: park at the spot you shoot best from; note the CELL range and bearing.
 */
@TeleOp(name = "BioBuzz Calibration", group = "BioBuzz Test")
public class BioBuzzCalibration extends OpMode {

    private BioBuzzRobot robot;
    private HiveVision vision;
    private Pinpoint pinpoint;
    private boolean armActive = false;
    private boolean armAtStow = false;
    private double armJog = 0.5;
    private int stowDirection = ARM_STOW_DIRECTION;

    @Override
    public void init() {
        robot = new BioBuzzRobot(hardwareMap);
        try {
            vision = new HiveVision(hardwareMap);
        } catch (Exception e) {
            vision = null;
        }
        if (BioBuzzRobot.armCalibrated()) {
            armJog = ArmCalibration.deployPosition();
            stowDirection = ArmCalibration.stowDirection();
        }
        pinpoint = Pinpoint.tryCreate(hardwareMap);
        if (pinpoint != null) {
            pinpoint.reset();
        }
    }

    @Override
    public void loop() {
        if (vision != null) {
            vision.update();
        }
        robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 0.3);

        if (gamepad1.xWasPressed()) {
            robot.resetDriveEncoders();
            robot.imu.resetYaw();
            if (pinpoint != null) {
                pinpoint.reset();
            }
        }

        robot.setIndexerPower(gamepad1.right_bumper ? 0.25 : (gamepad1.left_bumper ? -0.25 : 0));

        if (gamepad1.aWasPressed()) {
            armActive = true;
            armAtStow = false;
        }
        if (gamepad1.bWasPressed()) {
            stowDirection = -stowDirection;
        }
        if (gamepad1.dpadUpWasPressed()) {
            armJog += 0.010;
        } else if (gamepad1.dpadDownWasPressed()) {
            armJog -= 0.010;
        } else if (gamepad1.dpadRightWasPressed()) {
            armJog += 0.002;
        } else if (gamepad1.dpadLeftWasPressed()) {
            armJog -= 0.002;
        }
        armJog = Range.clip(armJog, 0, 1);
        double stowTest = armJog + stowDirection * ARM_STOW_OFFSET;
        boolean stowValid = stowTest >= 0 && stowTest <= 1;
        if (gamepad1.yWasPressed() && armActive) {
            armAtStow = !armAtStow && stowValid;
        }
        if (armActive) {
            robot.flowerArm.setPosition(armAtStow ? stowTest : armJog);
        }

        telemetry.addLine("--- Drive (X zeroes) ---");
        telemetry.addData("Forward in", "%.2f", robot.forwardInches());
        telemetry.addData("Strafe right in", "%.2f", robot.strafeRightInches());
        telemetry.addData("Ticks LF RF LB RB", "%d %d %d %d", robot.leftFront.getCurrentPosition(),
                robot.rightFront.getCurrentPosition(), robot.leftBack.getCurrentPosition(),
                robot.rightBack.getCurrentPosition());
        telemetry.addData("Heading (CCW +)", "%.1f deg", robot.headingDeg());

        if (pinpoint == null) {
            telemetry.addData("Pinpoint", "not in the configuration");
        } else {
            pinpoint.update();
            telemetry.addData("Pinpoint", "%s   push fwd -> X+, slide LEFT -> Y+, turn CCW -> H+", pinpoint.status());
            telemetry.addData("Pinpoint X Y H", "%.2f in  %.2f in  %.1f deg", pinpoint.xIn(), pinpoint.yIn(), pinpoint.headingDeg());
        }

        telemetry.addLine("--- Windmill (RB fwd / LB back) ---");
        telemetry.addData("Ticks", "%d  (%.2f pockets)", robot.indexerTicks(),
                robot.indexerTicks() / INDEXER_TICKS_PER_POCKET);

        telemetry.addLine("--- Flower arm (A starts) ---");
        telemetry.addData("Arm", armActive ? (armAtStow ? "at STOW test" : "at jog position") : "not powered");
        telemetry.addData("Jog position", "%.3f", armJog);
        telemetry.addData("Stow test", stowValid ? String.format("%.3f (direction %+d)", stowTest, stowDirection)
                : String.format("out of range (direction %+d) - press B", stowDirection));

        telemetry.addLine("--- Camera ---");
        if (vision == null) {
            telemetry.addData("Camera", "not found");
        } else {
            for (HiveVision.Alliance a : HiveVision.Alliance.values()) {
                HiveVision.Target t = vision.target(a, null);
                telemetry.addData(a + " CELL", t == null ? "-" : String.format("%s %d tags bearing %.1f range %.1f in",
                        t.cell, t.tagCount, t.bearingDeg, t.rangeIn));
            }
        }
        telemetry.addData("Battery", "%.2f V", robot.battery.getVoltage());
    }

    @Override
    public void stop() {
        robot.stopAll();
        if (vision != null) {
            vision.close();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Test", group = "Test")
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor motor = hardwareMap.get(DcMotor.class, "test_motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y;
            motor.setPower(power);
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Position", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
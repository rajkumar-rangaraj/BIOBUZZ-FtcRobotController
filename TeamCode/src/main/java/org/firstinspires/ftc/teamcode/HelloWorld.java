package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled   // v1 (windmill robot): parked 21 Nov 2026, see v2/

@TeleOp(name = "v1 Hello World", group = "Test")
public class HelloWorld extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        ElapsedTime runtime = new ElapsedTime();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Hello from BIOBUZZ");
            telemetry.addData("Runtime", "%.1f s", runtime.seconds());
            telemetry.addData("Gamepad A", gamepad1.a);
            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;

/*
 * Wiring Check - finds wiring mistakes automatically. Robot ON A BOX (wheels free), no POLLEN, hands clear.
 * Spins each motor alone for a moment and reads ALL encoders and the motor current, then reports:
 *   - motor power cable missing / wrong port   (no current, nothing moves)
 *   - encoder cable missing                    (current flows but no encoder counts)
 *   - encoder cables swapped                   (a DIFFERENT motor's encoder counts)
 *   - encoder or motor direction reversed      (own encoder counts down for a forward command)
 *   - hubs found, IMU, camera, battery
 * Results: on the Driver Hub and in /sdcard/FIRST/BioBuzzLogs/wiring_*.csv
 */
@TeleOp(name = "BioBuzz Wiring Check", group = "BioBuzz Test")
public class BioBuzzWiringCheck extends LinearOpMode {
    private static final int MOVED = 30;                 // ticks: clearly turning
    private static final double TURNING_AMPS = 0.15;     // current that means the motor is powered

    private final List<String> lines = new ArrayList<>();
    private int failures = 0;
    private CsvLog log;
    private final ElapsedTime clock = new ElapsedTime();

    @Override
    public void runOpMode() {
        log = new CsvLog("wiring", "t", "item", "result", "detail");
        BioBuzzRobot robot;
        try {
            robot = new BioBuzzRobot(hardwareMap);
        } catch (Exception e) {
            report("CONFIG", "FAIL", e.getMessage());
            log.close();
            telemetry.addData("CONFIGURATION ERROR", e.getMessage());
            telemetry.update();
            waitForStart();
            return;
        }

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        StringBuilder hubText = new StringBuilder();
        for (LynxModule h : hubs) {
            hubText.append(h.getDeviceName()).append(h.isParent() ? " (Control Hub/parent)" : " (Expansion Hub)").append("; ");
        }
        report("HUBS", hubs.size() >= 2 ? "PASS" : "FAIL", hubs.size() + " hub(s): " + hubText
                + (hubs.size() < 2 ? " - Expansion Hub missing: check the RS485 cable, its power and the configuration" : ""));
        report("BATTERY", robot.battery.getVoltage() >= 12.0 ? "PASS" : "WARN", String.format("%.2f V", robot.battery.getVoltage()));

        telemetry.addLine("WIRING CHECK: robot ON A BOX (all wheels free), no POLLEN, hands clear of the flywheel.");
        telemetry.addLine("Press PLAY. Each motor spins for under a second.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) {
            log.close();
            return;
        }

        String[] names = {LEFT_FRONT_DRIVE, RIGHT_FRONT_DRIVE, LEFT_BACK_DRIVE, RIGHT_BACK_DRIVE, LAUNCHER, INDEXER, INTAKE};
        DcMotorEx[] motors = {robot.leftFront, robot.rightFront, robot.leftBack, robot.rightBack, robot.launcher, robot.indexer, robot.intake};
        double[] power = {0.25, 0.25, 0.25, 0.25, 0.3, 0.2, 0.4};
        double[] seconds = {0.6, 0.6, 0.6, 0.6, 1.0, 0.4, 0.6};
        boolean[] hasEncoder = {true, true, true, true, true, true, false};
        for (DcMotorEx m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        for (int i = 0; i < motors.length && opModeIsActive(); i++) {
            int[] before = positions(motors);
            double maxAmps = 0;
            ElapsedTime t = new ElapsedTime();
            if (motors[i] == robot.intake) {
                robot.setIntake(power[i]);                      // intake motor + both corner servos
            } else {
                motors[i].setPower(power[i]);
            }
            while (opModeIsActive() && t.seconds() < seconds[i]) {
                maxAmps = Math.max(maxAmps, motors[i].getCurrent(CurrentUnit.AMPS));
                telemetry.addLine("Spinning " + names[i] + " (" + port(motors[i]) + ")");
                telemetry.update();
            }
            if (motors[i] == robot.intake) {
                robot.setIntake(0);
            } else {
                motors[i].setPower(0);
            }
            sleep(motors[i] == robot.launcher ? 1500 : 300);   // let it stop before the next motor
            int[] after = positions(motors);
            judge(names, motors, i, hasEncoder[i], before, after, maxAmps);
        }

        try {
            double h1 = robot.headingDeg();
            report("IMU", Double.isNaN(h1) ? "FAIL" : "PASS", String.format("heading %.1f deg (turn the robot by hand in "
                    + "System Check part 2 to confirm the direction)", h1));
        } catch (Exception e) {
            report("IMU", "FAIL", "no IMU named '" + IMU_NAME + "': " + e.getMessage());
        }
        try {
            HiveVision v = new HiveVision(hardwareMap);
            ElapsedTime t = new ElapsedTime();
            while (opModeIsActive() && t.seconds() < 3 && !v.cameraStreaming()) {
                sleep(50);
            }
            report("CAMERA", v.cameraStreaming() ? "PASS" : "FAIL", v.cameraStreaming() ? "'" + WEBCAM + "' streaming"
                    : "'" + WEBCAM + "' found but not streaming: check the USB cable (USB 2.0 port)");
            v.close();
        } catch (Exception e) {
            report("CAMERA", "FAIL", "'" + WEBCAM + "' not found: " + e.getMessage());
        }
        report("SERVOS", "INFO", "corner intake servos ran with the intake test (both should have pulled inward); "
                + "flower arm is tested in System Check part 3");
        report("SUMMARY", failures == 0 ? "PASS" : "FAIL", failures == 0 ? "no wiring problems found" : failures + " problem(s)");
        log.close();

        while (opModeIsActive()) {
            telemetry.addLine(failures == 0 ? "WIRING OK" : failures + " WIRING PROBLEM(S)");
            for (String l : lines) {
                telemetry.addLine(l);
            }
            telemetry.addData("Saved", "/sdcard/FIRST/BioBuzzLogs/wiring_*.csv");
            telemetry.update();
            sleep(100);
        }
    }

    private void judge(String[] names, DcMotorEx[] motors, int i, boolean hasEncoder, int[] before, int[] after, double amps) {
        String who = names[i] + " (" + port(motors[i]) + ")";
        int own = after[i] - before[i];
        int other = -1, otherDelta = 0;
        for (int j = 0; j < motors.length; j++) {
            int d = after[j] - before[j];
            if (j != i && Math.abs(d) > MOVED && Math.abs(d) > Math.abs(otherDelta)) {
                other = j;
                otherDelta = d;
            }
        }
        String ampsText = String.format("%.2f A", amps);
        if (amps < TURNING_AMPS && Math.abs(own) <= MOVED && other < 0) {
            report(names[i], "FAIL", who + ": no current (" + ampsText + ") and nothing moved - motor power cable not "
                    + "plugged in, or plugged into a different port than the configuration says");
        } else if (other >= 0 && Math.abs(own) <= MOVED) {
            report(names[i], "FAIL", who + ": its encoder did not count, but " + names[other] + "'s encoder did ("
                    + otherDelta + " ticks) - encoder cables of " + names[i] + " and " + names[other] + " are swapped "
                    + "(each encoder cable must go in the encoder port with the same number as its motor)");
        } else if (!hasEncoder) {
            report(names[i], "PASS", who + ": runs (" + ampsText + "); no encoder needed");
        } else if (Math.abs(own) <= MOVED) {
            report(names[i], "FAIL", who + ": motor runs (" + ampsText + ") but its encoder did not count - encoder cable "
                    + "missing or loose");
        } else if (own < 0) {
            report(names[i], "FAIL", who + ": encoder counts DOWN (" + own + ") for a forward command - motor wires or "
                    + "encoder reversed; for drive motors also check the wheel turns the right way in System Check part 1");
        } else {
            report(names[i], "PASS", who + ": encoder +" + own + " ticks, " + ampsText
                    + (other >= 0 ? " (note: " + names[other] + " also moved " + otherDelta + ")" : ""));
        }
    }

    private static int[] positions(DcMotorEx[] motors) {
        int[] p = new int[motors.length];
        for (int i = 0; i < motors.length; i++) {
            p[i] = motors[i].getCurrentPosition();
        }
        return p;
    }

    private static String port(DcMotorEx m) {
        return m.getController().getConnectionInfo() + " motor " + m.getPortNumber();
    }

    private void report(String item, String result, String detail) {
        lines.add(result + " " + detail);
        log.row(clock.seconds(), item, result, detail);
        if (result.equals("FAIL")) {
            failures++;
        }
    }
}

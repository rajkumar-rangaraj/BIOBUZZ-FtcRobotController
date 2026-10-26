package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/*
 * On-robot integration test. Run it after every rebuild, wiring change or configuration change.
 * The robot measures what it can (encoders, IMU, flywheel speed); a person confirms what only eyes can see.
 * A / Cross = YES (or start a part), B / Circle = NO (or skip a part).
 *
 *   PART 1 Wheels    robot on a box: each drive motor alone -> right wheel? right direction? encoder counting?
 *   PART 2 Floor     forward, back, SLIDE RIGHT, SLIDE LEFT, turn clockwise, turn counter-clockwise
 *   PART 3 Mechanisms intake, windmill, launcher, flower arm (if calibrated)
 */
@TeleOp(name = "BioBuzz System Check", group = "BioBuzz Test")
public class BioBuzzSystemCheck extends LinearOpMode implements DriveIO {

    private static final String[] WHEEL_NAMES = {"LEFT FRONT", "RIGHT FRONT", "LEFT BACK", "RIGHT BACK"};

    private BioBuzzRobot robot;
    private final ElapsedTime clock = new ElapsedTime();
    private final List<String> results = new ArrayList<>();
    private int failures = 0;

    @Override
    public void runOpMode() {
        try {
            robot = new BioBuzzRobot(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("CONFIGURATION ERROR", e.getMessage());
            telemetry.addLine("A device name is missing or misspelled in the active robot configuration.");
            telemetry.update();
            waitForStart();
            return;
        }
        telemetry.addLine("BioBuzz System Check - press PLAY, then follow the prompts.");
        telemetry.update();
        waitForStart();

        if (ask("PART 1 - WHEELS: put the robot on a box, no wheel touching anything. A = start, B = skip")) {
            checkWheels();
        }
        if (ask("PART 2 - FLOOR: robot on tiles, 3 ft clear on every side. A = start, B = skip")) {
            checkFloorMoves();
        }
        if (ask("PART 3 - MECHANISMS: no POLLEN in the robot, hands clear. A = start, B = skip")) {
            checkMechanisms();
        }
        robot.stopAll();

        while (opModeIsActive()) {
            telemetry.addLine(failures == 0 ? "ALL CHECKS PASSED" : failures + " CHECK(S) FAILED");
            for (String r : results) {
                telemetry.addLine(r);
            }
            telemetry.update();
            sleep(50);
        }
    }

    // ---------------- PART 1 ----------------

    private void checkWheels() {
        DcMotorEx[] motors = robot.driveMotors();
        for (int i = 0; i < 4 && opModeIsActive(); i++) {
            int before = motors[i].getCurrentPosition();
            motors[i].setPower(0.3);
            pause(0.7, "Spinning " + WHEEL_NAMES[i] + " - watch which wheel turns");
            motors[i].setPower(0);
            int delta = motors[i].getCurrentPosition() - before;

            if (delta > 100) {
                pass(WHEEL_NAMES[i] + " encoder counts up (" + delta + ")");
            } else if (delta < -100) {
                fail(WHEEL_NAMES[i] + " encoder counts DOWN (" + delta + "): encoder or motor polarity reversed");
            } else {
                fail(WHEEL_NAMES[i] + " encoder ~0 (" + delta + "): encoder cable not in the matching port");
            }
            boolean ok = ask("Did ONLY the " + WHEEL_NAMES[i] + " wheel spin, rolling the robot toward the INTAKE?");
            record(ok, WHEEL_NAMES[i] + " is the right wheel and spins forward",
                    "fix the configuration name/port or the motor direction for " + WHEEL_NAMES[i]);
        }
    }

    // ---------------- PART 2 ----------------

    private void checkFloorMoves() {
        robot.imu.resetYaw();
        MotionController motion = new MotionController(this);
        String[] names = {"FORWARD", "BACK", "SLIDE RIGHT", "SLIDE LEFT"};
        double[][] moves = {{12, 0}, {-12, 0}, {0, 12}, {0, -12}};
        for (int i = 0; i < moves.length && opModeIsActive(); i++) {
            double headingBefore = robot.headingDeg();
            motion.driveRelative(moves[i][0], moves[i][1], 3.0);
            double turned = MecanumMath.normalizeDegrees(robot.headingDeg() - headingBefore);
            record(Math.abs(turned) < 5, names[i] + " kept its heading (" + fmt(turned) + " deg)",
                    names[i] + " turned " + fmt(turned) + " deg: check wheels, motor directions, weight balance");
            boolean ok = ask("Did the robot move " + names[i] + " about 1 ft WITHOUT turning?");
            String hint = i >= 2
                    ? "no slide: check LEFT FRONT/LEFT BACK and RIGHT FRONT/RIGHT BACK are not swapped, and the "
                    + "rollers form an X seen from ABOVE (each side needs one left-hand and one right-hand wheel)"
                    : "check the motor directions in PART 1";
            record(ok, names[i] + " confirmed by driver", hint);
        }

        double[] targets = {-90, 0};
        String[] turnNames = {"TURN CLOCKWISE 90", "TURN BACK COUNTER-CLOCKWISE"};
        for (int i = 0; i < targets.length && opModeIsActive(); i++) {
            boolean reached = motion.turnToHeading(targets[i], 4.0);
            record(reached, turnNames[i] + ": IMU reached " + fmt(targets[i]) + " deg",
                    turnNames[i] + ": IMU never reached the heading - check HUB_LOGO / HUB_USB");
            boolean ok = ask("Did the robot " + turnNames[i] + " (a quarter turn)?");
            record(ok, turnNames[i] + " confirmed by driver", "turn direction wrong: check HUB_LOGO / HUB_USB");
        }
    }

    // ---------------- PART 3 ----------------

    private void checkMechanisms() {
        robot.setIntake(0.6);
        pause(1.0, "Intake running IN");
        robot.setIntake(0);
        record(ask("Did the intake roller and BOTH corner wheels pull INWARD?"), "intake pulls in",
                "intake: flip the motor or a corner-servo direction");

        int start = robot.indexerTicks();
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < 2.0
                && Math.abs(robot.indexerTicks() - start) < INDEXER_TICKS_PER_POCKET) {
            robot.setIndexerPower(0.3);
            telemetry.addLine("Windmill turning one pocket");
            telemetry.update();
        }
        robot.setIndexerPower(0);
        int moved = robot.indexerTicks() - start;
        record(Math.abs(moved) >= INDEXER_TICKS_PER_POCKET * 0.9, "windmill encoder: one pocket = " + moved + " ticks",
                "windmill encoder only moved " + moved + " ticks: check cable / jam");
        record(ask("Did the windmill turn about a quarter turn TOWARD the launcher?"), "windmill FIRE direction",
                "windmill direction: change INDEXER_DIRECTION");

        robot.setLauncherVelocity(LAUNCHER_NEAR_VELOCITY);
        t.reset();
        while (opModeIsActive() && t.seconds() < 3.0 && !robot.launcherReady()) {
            telemetry.addData("Launcher", "%.0f / %.0f t/s", robot.launcherVelocity(), LAUNCHER_NEAR_VELOCITY);
            telemetry.update();
        }
        boolean ready = robot.launcherReady();
        double spinUp = t.seconds();
        pause(0.5, "Holding launcher speed");
        String warning = robot.launcherWarning();
        double velocity = robot.launcherVelocity();
        robot.setLauncherVelocity(0);
        record(ready && warning == null,
                "launcher reached " + fmt(velocity) + " t/s in " + fmt(spinUp) + " s",
                "launcher " + (warning != null ? warning : "did not reach speed in 3 s (" + fmt(velocity) + " t/s)"));

        if (!BioBuzzRobot.armCalibrated()) {
            results.add("SKIP flower arm: not calibrated (run BioBuzz Calibration)");
        } else if (ask("Flower arm test: keep hands clear of the arm swing. A = run, B = skip")) {
            robot.stowArm();
            pause(1.0, "Arm STOW");
            robot.deployArm();
            pause(1.5, "Arm DEPLOY");
            record(ask("Did the arm DEPLOY onto both pads without buzzing?"), "arm deploys", "re-calibrate ARM_DEPLOY_POSITION");
            robot.stowArm();
            pause(1.5, "Arm STOW");
            record(ask("Did the arm STOW onto its posts without buzzing?"), "arm stows", "check ARM_STOW_DIRECTION");
        }
    }

    // ---------------- helpers ----------------

    /** Waits for A (true) or B (false). */
    private boolean ask(String question) {
        robot.stopDrive();
        gamepad1.aWasPressed();
        gamepad1.bWasPressed();
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                return true;
            }
            if (gamepad1.bWasPressed()) {
                return false;
            }
            telemetry.addLine(question);
            telemetry.addLine("A / Cross = YES     B / Circle = NO");
            telemetry.addData("Failures so far", failures);
            telemetry.update();
            sleep(20);
        }
        return false;
    }

    private void pause(double seconds, String message) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < seconds) {
            telemetry.addLine(message);
            telemetry.update();
            sleep(20);
        }
    }

    private void record(boolean ok, String passText, String failText) {
        if (ok) {
            pass(passText);
        } else {
            fail(failText);
        }
    }

    private void pass(String text) {
        results.add("PASS " + text);
    }

    private void fail(String text) {
        results.add("FAIL " + text);
        failures++;
    }

    private static String fmt(double v) {
        return String.format("%.1f", v);
    }

    // ---------------- DriveIO (PART 2 uses the same controller as AUTO) ----------------

    @Override
    public boolean active() {
        return opModeIsActive();
    }

    @Override
    public double seconds() {
        return clock.seconds();
    }

    @Override
    public double forwardInches() {
        return robot.forwardInches();
    }

    @Override
    public double strafeRightInches() {
        return robot.strafeRightInches();
    }

    @Override
    public double headingDeg() {
        return robot.headingDeg();
    }

    @Override
    public void drive(double forward, double strafeRight, double turnCw) {
        robot.drive(forward, strafeRight, turnCw, 1.0);
    }

    @Override
    public void loopTick() {
        telemetry.addData("Moving", "fwd %.1f  right %.1f in  heading %.1f",
                robot.forwardInches(), robot.strafeRightInches(), robot.headingDeg());
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * TEST OpMode: indexer (motor + gecko wheels, 2 chained layers) feeding the flywheel.
 * No driving. Use this to find a flywheel speed and indexer power that shoot well,
 * then copy those numbers into the main teleop.
 *
 * ============================ CONTROLS (gamepad1) ============================
 *   Y               Flywheel ON/OFF
 *   D-pad up/down   Flywheel target speed +/- 50 ticks/s
 *   D-pad right/left Indexer power +/- 0.1
 *   Right bumper    (hold) Run indexer forward = FEED
 *   Left bumper     (hold) Run indexer backward = UNJAM
 *   A               Toggle "wait for speed" (feed only when flywheel is at speed)
 *   B               Reset shot stats
 *   Right trigger   Intake in     Left trigger  Intake out
 * ============================================================================
 *
 * HOW TO TEST
 *   1. Turn flywheel on (Y), wait for rumble.
 *   2. Load balls, hold RB. Watch "Last dip" and "Recovery" after each shot.
 *   3. Big dip or slow recovery = balls fired too close together -> lower indexer power,
 *      or raise flywheel speed. Shots falling short -> raise speed. Overshooting -> lower.
 *   4. Write down the Target speed and Indexer power that work best.
 *
 * Robot configuration: add a motor named "indexer" (and remove "windmillServo").
 */

@TeleOp(name = "TEST Indexer + Flywheel", group = "Test")
public class IndexerFlywheelTest extends OpMode {

    private DcMotorEx launcher;
    private DcMotor indexer;
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private VoltageSensor battery;

    private final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(40, 0, 0, 12.5);
    private static final double TICKS_PER_REV = 28.0;     // goBILDA 6000 RPM 1:1
    private static final double MAX_VELOCITY = 2800;      // ~6000 RPM
    private static final double VELOCITY_STEP = 50;
    private static final double INDEXER_STEP = 0.1;
    private static final double READY_MARGIN = 100;       // ready = above (target - 100)
    private static final double SHOT_DIP = 150;           // drop this far below target = a shot

    // Values you are tuning (starting points)
    double targetVelocity = 1250;
    double indexerPower = 0.6;

    boolean launcherOn = false;
    boolean requireReady = true;
    boolean ready = false;
    boolean lastReady = false;
    double velocity = 0;
    double indexerOutput = 0;

    // Button edge detection
    boolean lastY, lastA, lastB, lastUp, lastDown, lastLeft, lastRight;

    // Shot stats (detected from flywheel speed dips)
    boolean armed = false;          // only detect shots after the flywheel reached speed
    boolean inDip = false;
    double dipLowest = 0;
    int shotCount = 0;
    double lastDipDepth = 0;
    double lastRecoverySec = 0;
    double worstDipDepth = 0;
    ElapsedTime dipTimer = new ElapsedTime();

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        indexer = hardwareMap.get(DcMotor.class, "indexer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        battery = hardwareMap.voltageSensor.iterator().next();

        launcher.setZeroPowerBehavior(FLOAT);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setPower(0);

        // If RB pushes balls AWAY from the flywheel, change this to REVERSE.
        indexer.setDirection(DcMotor.Direction.FORWARD);
        indexer.setZeroPowerBehavior(BRAKE);   // holds balls in place when stopped
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setZeroPowerBehavior(BRAKE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized - TEST mode, robot will not drive");
    }

    @Override
    public void loop() {
        handleButtons();

        // --- Flywheel ---
        if (launcherOn) {
            launcher.setVelocity(targetVelocity);
        }
        velocity = launcher.getVelocity();
        double minVelocity = targetVelocity - READY_MARGIN;
        ready = launcherOn && velocity > minVelocity;

        if (ready && !lastReady) {
            gamepad1.rumble(250);
            armed = true;
        }
        lastReady = ready;

        // --- Indexer ---
        if (gamepad1.right_bumper && (ready || !requireReady)) {
            indexerOutput = indexerPower;
        } else if (gamepad1.left_bumper) {
            indexerOutput = -indexerPower;
        } else {
            indexerOutput = 0;
        }
        indexer.setPower(indexerOutput);

        // --- Intake ---
        double intakePower = Range.clip(gamepad1.right_trigger - gamepad1.left_trigger, -1, 1);
        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        updateShotStats(minVelocity);
        updateTelemetry(minVelocity);
    }

    @Override
    public void stop() {
        launcher.setPower(0);
        indexer.setPower(0);
        intake.setPower(0);
    }

    void handleButtons() {
        if (gamepad1.y && !lastY) {
            launcherOn = !launcherOn;
            if (launcherOn) {
                launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);
            } else {
                launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                launcher.setPower(0);
            }
            armed = false;
            inDip = false;
        }
        if (gamepad1.dpad_up && !lastUp) {
            targetVelocity = Range.clip(targetVelocity + VELOCITY_STEP, 0, MAX_VELOCITY);
            armed = false;   // wait until it reaches the new speed before counting shots
            inDip = false;
        }
        if (gamepad1.dpad_down && !lastDown) {
            targetVelocity = Range.clip(targetVelocity - VELOCITY_STEP, 0, MAX_VELOCITY);
            armed = false;
            inDip = false;
        }
        if (gamepad1.dpad_right && !lastRight) {
            indexerPower = Range.clip(indexerPower + INDEXER_STEP, 0.1, 1.0);
        }
        if (gamepad1.dpad_left && !lastLeft) {
            indexerPower = Range.clip(indexerPower - INDEXER_STEP, 0.1, 1.0);
        }
        if (gamepad1.a && !lastA) {
            requireReady = !requireReady;
        }
        if (gamepad1.b && !lastB) {
            shotCount = 0;
            lastDipDepth = 0;
            lastRecoverySec = 0;
            worstDipDepth = 0;
        }

        lastY = gamepad1.y;
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
    }

    /*
     * Each ball pulls the flywheel speed down. We count a "shot" when speed drops
     * SHOT_DIP below target, and record how deep the drop was and how long it took
     * to climb back above the ready speed.
     */
    void updateShotStats(double minVelocity) {
        if (!launcherOn || !armed) return;

        if (!inDip && velocity < targetVelocity - SHOT_DIP) {
            inDip = true;
            dipLowest = velocity;
            dipTimer.reset();
        } else if (inDip) {
            dipLowest = Math.min(dipLowest, velocity);
            if (velocity > minVelocity) {
                inDip = false;
                shotCount++;
                lastDipDepth = targetVelocity - dipLowest;
                lastRecoverySec = dipTimer.seconds();
                worstDipDepth = Math.max(worstDipDepth, lastDipDepth);
            }
        }
    }

    void updateTelemetry(double minVelocity) {
        telemetry.addData("Battery", "%.2f V", battery.getVoltage());

        telemetry.addLine("--- Flywheel ---");
        telemetry.addData("State", !launcherOn ? "OFF" : (ready ? "READY" : "spinning up..."));
        telemetry.addData("Target", "%.0f t/s (~%.0f RPM)", targetVelocity, targetVelocity / TICKS_PER_REV * 60);
        telemetry.addData("Actual", "%.0f t/s (~%.0f RPM)", velocity, velocity / TICKS_PER_REV * 60);
        telemetry.addData("Ready above", "%.0f t/s", minVelocity);

        telemetry.addLine("--- Indexer ---");
        telemetry.addData("Power setting", "%.1f", indexerPower);
        telemetry.addData("Output now", "%.1f", indexerOutput);
        telemetry.addData("Wait for speed (A)", requireReady ? "ON" : "OFF - feeds any time");

        telemetry.addLine("--- Shots ---");
        telemetry.addData("Shots counted", shotCount);
        telemetry.addData("Last dip", "%.0f t/s", lastDipDepth);
        telemetry.addData("Worst dip", "%.0f t/s", worstDipDepth);
        telemetry.addData("Last recovery", "%.2f s", lastRecoverySec);

        telemetry.addLine("Y fly | Dpad U/D speed | Dpad L/R index pwr | RB feed | LB unjam | A gate | B reset");
    }
}
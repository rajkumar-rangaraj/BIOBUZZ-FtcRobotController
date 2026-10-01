/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * Teleop (driver-controlled) OpMode for the goBILDA® StarterBot with Mecanum wheels,
 * modified by team BioBuzz.
 *
 * ============================ CONTROLS (gamepad1) ============================
 *   Left stick          Drive forward/back and strafe left/right
 *   Right stick X       Rotate
 *   Left bumper (hold)  SLOW / precision drive (40% speed)
 *   Right trigger       Intake in
 *   Left trigger        Intake out (reverse)
 *   Y / Triangle        Toggle launcher ON/OFF (pre-spin while driving)
 *   Right bumper (hold) FIRE: feeds with the windmill once the launcher is at speed.
 *                       If the launcher is OFF, holding this also spins it up
 *                       (same as the old behavior).
 *   Controller rumbles once when the launcher reaches firing speed.
 * ============================================================================
 *
 * CHANGES IN THIS VERSION:
 *   Bug fixes
 *   - Launcher now COASTS down when turned off instead of braking hard.
 *   - "Encoder" warning no longer flashes at the start of every spin-up; it only
 *     appears if the launcher is still not reading after 1 second.
 *   - Left trigger (reverse) can no longer fight the feeder while firing.
 *   - "Ready to Fire" only shows YES while the launcher is actually running.
 *   Enhancements
 *   - Launcher on/off toggle (Y) separate from firing (right bumper).
 *   - Controller rumble when the launcher is ready.
 *   - Slow/precision drive mode (left bumper).
 *   - Bulk reads from the hubs for a faster, more responsive loop.
 */

@TeleOp(name = "Mec BioBuzz StarterBot Teleop", group = "StarterBot")
//@Disabled
public class BioBuzzStarterbotTeleopMecanum extends OpMode {

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    /*
     * Launcher velocities, in encoder ticks per second.
     * With a 28 tick/rev encoder (goBILDA 6000 RPM 1:1 motor):
     *   RPM = ticks per second / 28 * 60
     * TARGET is the speed we ask the launcher to hold.
     * MIN is the speed it must be above before the windmill feeds a ball.
     * Keep MIN about 50-100 below TARGET. Tune TARGET by test shots.
     */
    public final int LAUNCHER_TARGET_VELOCITY = 1250; // ~2678 RPM
    public final int LAUNCHER_MIN_VELOCITY = 1150;    // ~2464 RPM

    // PIDF values for launcher velocity control.
    private final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(40, 0, 0, 12.5);

    // Drive speed multiplier while the left bumper is held (0.4 = 40%).
    public final double SLOW_DRIVE_SCALE = 0.4;

    // Extra intake power while firing, to push stuck elements toward the launcher.
    public final double FEED_INTAKE_BOOST = 0.5;

    // How long the launcher may run before we warn about a missing/reversed encoder.
    public final double ENCODER_WARNING_DELAY_SEC = 1.0;
    public final double ENCODER_NOT_MOVING_THRESHOLD = 50; // ticks per second

    // Drive power for each wheel, stored here so telemetry can display them.
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    boolean slowMode = false;

    // Power sent to the intake motor and intake servos.
    double intakePower;

    // Launcher / windmill state.
    boolean launcherToggledOn = false;   // set by the Y button
    boolean launcherActive = false;      // true if toggled on OR right bumper held
    boolean lastLauncherActive = false;  // launcherActive from the previous loop
    boolean lastYButton = false;         // Y button from the previous loop (for toggle)
    boolean launcherReady = false;       // launcher running AND above min velocity
    boolean lastLauncherReady = false;   // for rumbling only once when ready
    boolean windmillRunning = false;
    double launcherTargetVelocity = 0;
    double launcherVelocity = 0;
    ElapsedTime launcherRunTimer = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /*
         * Bulk reads: the hub reads all motor/encoder data in one message per loop
         * instead of one message per value. This makes the loop faster.
         */
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        /*
         * The names here must match the robot configuration on the Control Hub exactly.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        /*
         * Pushing the left stick forward MUST make the robot go forward.
         * Flip these if your first test drive goes the wrong way.
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * BRAKE mode makes the robot stop quickly when the sticks are released.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        /*
         * The launcher starts OFF and set to FLOAT so it coasts when stopped.
         * When it turns on, launch() switches it to RUN_USING_ENCODER (velocity control).
         * If the launcher jumps to full speed and ignores the set point, check that the
         * encoder cable is plugged in beside the launcher motor's port, and that motor
         * polarity is consistent through any wiring.
         */
        launcher.setZeroPowerBehavior(FLOAT);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setPower(0);

        /*
         * Reverse the right intake servo so both sides pull elements inward.
         */
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Set servos to 0 to initialize the servo controller.
         */
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after INIT, before START.
     * Shows the launcher reading so you can confirm the encoder works before the match.
     */
    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized - waiting for START");
        telemetry.addData("Launcher Vel", "%.0f (should be 0 when stopped)", launcher.getVelocity());
        telemetry.addLine("Y = launcher on/off | RB = fire | LB = slow drive");
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after START, before STOP
     */
    @Override
    public void loop() {
        /*
         * Left stick: forward/back and strafe. Right stick X: rotate.
         * Stick forward gives a negative value, so we invert left_stick_y.
         * Holding the left bumper drives at reduced speed for precise lining up.
         */
        slowMode = gamepad1.left_bumper;
        double driveScale = slowMode ? SLOW_DRIVE_SCALE : 1.0;
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, driveScale);

        /*
         * Right trigger = intake in, left trigger = intake out (reverse).
         */
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        // launch() may change intake power while firing, so call it before setting the intake.
        launch();

        // Keep intake power in the valid range so telemetry shows the real value.
        intakePower = Range.clip(intakePower, -1.0, 1.0);

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        updateTelemetry();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate, double scale) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        // Scale all wheels down together if any is above 1.0, to keep the direction correct.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Apply slow-mode scaling (1.0 = full speed).
        leftFrontPower *= scale;
        rightFrontPower *= scale;
        leftBackPower *= scale;
        rightBackPower *= scale;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    void launch() {
        /*
         * Y toggles the launcher on/off. We only react when Y goes from
         * "not pressed" to "pressed", so holding Y does not flip it every loop.
         */
        boolean yButton = gamepad1.y;
        if (yButton && !lastYButton) {
            launcherToggledOn = !launcherToggledOn;
        }
        lastYButton = yButton;

        // The launcher runs if it was toggled on, or while the fire button is held.
        launcherActive = launcherToggledOn || gamepad1.right_bumper;

        /*
         * Switch motor modes only when the launcher turns on or off:
         *  - Turning ON: velocity control (RUN_USING_ENCODER) with our PIDF values.
         *  - Turning OFF: plain power mode with power 0, so the flywheel coasts down
         *    instead of braking hard (easier on the motor, gears, and battery).
         */
        if (launcherActive && !lastLauncherActive) {
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);
            launcherRunTimer.reset();
        } else if (!launcherActive && lastLauncherActive) {
            launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcher.setPower(0);
        }
        lastLauncherActive = launcherActive;

        if (launcherActive) {
            launcherTargetVelocity = LAUNCHER_TARGET_VELOCITY;
            launcher.setVelocity(launcherTargetVelocity);
        } else {
            launcherTargetVelocity = 0;
        }

        // Read the velocity once per loop and reuse it.
        launcherVelocity = launcher.getVelocity();

        // "Ready" only counts while the launcher is actually running.
        launcherReady = launcherActive && launcherVelocity > LAUNCHER_MIN_VELOCITY;

        // Rumble once each time the launcher becomes ready.
        if (launcherReady && !lastLauncherReady) {
            gamepad1.rumble(250);
        }
        lastLauncherReady = launcherReady;

        /*
         * Feed with the windmill only when the fire button is held AND the launcher is
         * fast enough for a good shot. While firing, ignore reverse on the intake so the
         * left trigger can't fight the feeder, then add a boost to push elements forward.
         */
        if (gamepad1.right_bumper && launcherReady) {
            windmillServo.setPower(1);
            windmillRunning = true;
            intakePower = Math.max(intakePower, 0) + FEED_INTAKE_BOOST;
        } else {
            windmillServo.setPower(0);
            windmillRunning = false;
        }
    }

    void updateTelemetry() {
        // --- Drive ---
        telemetry.addLine("--- Drive ---");
        telemetry.addData("Drive Mode", slowMode ? "SLOW (precision)" : "normal");
        telemetry.addData("Front", "FL (%.2f), FR (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("Back", "BL (%.2f), BR (%.2f)", leftBackPower, rightBackPower);

        // --- Intake ---
        telemetry.addLine("--- Intake ---");
        telemetry.addData("Triggers", "left (%.2f), right (%.2f)",
                gamepad1.left_trigger, gamepad1.right_trigger);
        telemetry.addData("Intake Power", "%.2f", intakePower);

        // --- Launcher ---
        String launcherState;
        if (!launcherActive) {
            launcherState = "OFF";
        } else if (launcherReady) {
            launcherState = "READY";
        } else {
            launcherState = "spinning up...";
        }

        telemetry.addLine("--- Launcher ---");
        telemetry.addData("Launcher", "%s (%s)", launcherState,
                launcherToggledOn ? "toggled on with Y" : (launcherActive ? "held RB" : "-"));
        telemetry.addData("Launcher Target", "%.0f", launcherTargetVelocity);
        telemetry.addData("Launcher Vel", "%.0f (~%.0f RPM)",
                launcherVelocity, launcherVelocity / 28.0 * 60.0);
        telemetry.addData("Launcher Min", "%d", LAUNCHER_MIN_VELOCITY);
        telemetry.addData("Ready to Fire", launcherReady ? "YES" : "no");
        telemetry.addData("Fire Button (RB)", gamepad1.right_bumper ? "HELD" : "released");
        telemetry.addData("Windmill", windmillRunning ? "RUNNING" : "off");

        /*
         * Warnings, only after the launcher has had time to spin up,
         * so they don't flash at the start of every spin-up.
         */
        if (launcherActive && launcherRunTimer.seconds() > ENCODER_WARNING_DELAY_SEC) {
            if (launcherVelocity < -ENCODER_NOT_MOVING_THRESHOLD) {
                telemetry.addData("WARNING", "Launcher velocity is negative - check motor/encoder polarity");
            } else if (Math.abs(launcherVelocity) < ENCODER_NOT_MOVING_THRESHOLD) {
                telemetry.addData("WARNING", "Launcher reads ~0 - check encoder cable");
            }
        }
    }
}

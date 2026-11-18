package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

/**
 * All BIOBUZZ hardware and the low-level commands shared by TeleOp, Autonomous and Calibration.
 * The constructor only configures hardware; it never moves a motor or servo.
 * <p>
 * Indexer v2 (Nov 2026): a second flywheel motor (launcher2), the queue gate servo and two IR break beams (the
 * centring wheels at the lane mouth were dropped on 17 Nov: the passive funnel walls centre the balls). Bulk caching: TeleOp, Tune and the Pedro AUTO make the robot with
 * {@code manualBulkCache = true} and call {@link #clearBulkCache()} once per loop, so every encoder, velocity and
 * beam read in a loop comes from ONE bulk read per hub (and all of them agree with each other). The test op modes
 * keep AUTO caching (a fresh bulk read whenever a value is read twice).
 */
public class BioBuzzRobot {
    public final DcMotorEx leftFront;
    public final DcMotorEx rightFront;
    public final DcMotorEx leftBack;
    public final DcMotorEx rightBack;
    public final DcMotorEx intake;
    public final DcMotorEx launcher;
    public final DcMotorEx launcher2;
    public final DcMotorEx indexer;
    public final CRServo leftIntakeServo;
    public final CRServo rightIntakeServo;
    public final Servo flowerArm;
    public final Servo gate;
    public final DigitalChannel beamStaged;
    public final DigitalChannel beamFull;
    public final IMU imu;
    public final VoltageSensor battery;
    private final List<LynxModule> hubs;
    private final boolean manualBulkCache;

    private boolean launcherOn = false;
    /** Last commanded outputs, kept for diagnostics (reading them back from the hub would cost loop time). */
    public final double[] lastDrivePower = new double[4];
    public double lastIntakePower = 0;
    public double lastIndexerPower = 0;
    /** Last gate command (null = never commanded since INIT) and the last beam readings (true = broken). */
    public Boolean lastGateOpen = null;
    public boolean lastStagedBroken = false;
    public boolean lastFullBroken = false;
    /** Launcher current from the last launcherCurrentAmps() this loop (NaN = not read; TeleOp reads it only while firing). */
    public double lastLauncherAmps = Double.NaN;
    /** TeleOp field-centric driving, for the diagnostics log: heading relative to the drivers, and whether it was used. */
    public double lastDriveHeadingDeg = Double.NaN;
    public boolean lastFieldDrive = false;
    private double launcherTarget = 0;
    private double writtenTarget = -1;                          // last velocity sent to the two motors (-1 = none)
    private final ElapsedTime launcherTimer = new ElapsedTime();
    private final LauncherGate launcherGate = LauncherGate.standard();
    private final FlywheelPairCheck pairCheck = FlywheelPairCheck.standard();
    private String launcherFault = null;                        // latched pair fault: both motors stopped
    private final ElapsedTime clock = new ElapsedTime();
    private boolean readyCached = false;
    private double readyAt = -1;
    /** launcherReady() reuses its last answer for this long: one decision per loop, the same one logs and checks see. */
    static final double READY_CACHE_SEC = 0.03;

    public enum ArmState { UNKNOWN, STOWED, DEPLOYED }
    private ArmState armState = ArmState.UNKNOWN;

    /** Test and setup op modes: AUTO bulk caching (no clearBulkCache() needed). */
    public BioBuzzRobot(HardwareMap hw) {
        this(hw, false);
    }

    /**
     * @param manualBulkCache MANUAL bulk caching: the op mode MUST call {@link #clearBulkCache()} once per loop (and per
     *                        INIT loop), or every encoder / velocity / beam reading stays frozen
     */
    public BioBuzzRobot(HardwareMap hw, boolean manualBulkCache) {
        this.manualBulkCache = manualBulkCache;
        hubs = hw.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(manualBulkCache ? LynxModule.BulkCachingMode.MANUAL : LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hw.get(DcMotorEx.class, LEFT_FRONT_DRIVE);
        rightFront = hw.get(DcMotorEx.class, RIGHT_FRONT_DRIVE);
        leftBack = hw.get(DcMotorEx.class, LEFT_BACK_DRIVE);
        rightBack = hw.get(DcMotorEx.class, RIGHT_BACK_DRIVE);
        intake = hw.get(DcMotorEx.class, INTAKE);
        launcher = hw.get(DcMotorEx.class, LAUNCHER);
        launcher2 = hw.get(DcMotorEx.class, LAUNCHER2);
        indexer = hw.get(DcMotorEx.class, INDEXER);
        leftIntakeServo = hw.get(CRServo.class, LEFT_INTAKE_SERVO);
        rightIntakeServo = hw.get(CRServo.class, RIGHT_INTAKE_SERVO);
        flowerArm = hw.get(Servo.class, FLOWER_ARM);
        gate = hw.get(Servo.class, GATE);
        beamStaged = hw.get(DigitalChannel.class, BEAM_STAGED);
        beamFull = hw.get(DigitalChannel.class, BEAM_FULL);
        imu = hw.get(IMU.class, IMU_NAME);
        battery = hw.voltageSensor.iterator().next();

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx m : driveMotors()) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        resetDriveEncoders();

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher2.setDirection(LAUNCHER2_DIRECTION);
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        indexer.setDirection(INDEXER_DIRECTION);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (flowerArm instanceof ServoImplEx) {
            ((ServoImplEx) flowerArm).setPwmRange(new PwmControl.PwmRange(ARM_PWM_MIN_US, ARM_PWM_MAX_US));
        }
        if (gate instanceof ServoImplEx) {
            ((ServoImplEx) gate).setPwmRange(new PwmControl.PwmRange(GATE_PWM_MIN_US, GATE_PWM_MAX_US));
        }
        beamStaged.setMode(DigitalChannel.Mode.INPUT);
        beamFull.setMode(DigitalChannel.Mode.INPUT);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
        ArmCalibration.reload();
    }

    public DcMotorEx[] driveMotors() {
        return new DcMotorEx[]{leftFront, rightFront, leftBack, rightBack};
    }

    /**
     * MANUAL bulk caching: the next read of any motor encoder / velocity / digital input does a fresh bulk read. Call
     * ONCE per loop (start or end, always the same place). Harmless with AUTO caching.
     */
    public void clearBulkCache() {
        if (!manualBulkCache) {
            return;
        }
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    /** Sets this robot's caching mode again (after a library such as Pedro Pathing may have changed it). */
    public void reapplyBulkCaching() {
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(manualBulkCache ? LynxModule.BulkCachingMode.MANUAL : LynxModule.BulkCachingMode.AUTO);
        }
    }

    // ---------------- Drive ----------------

    /** forward + = intake side, strafeRight + = robot right, turnCw + = clockwise (seen from above). */
    public void drive(double forward, double strafeRight, double turnCw, double scale) {
        double[] p = MecanumMath.wheelPowers(forward, strafeRight, turnCw, scale);
        System.arraycopy(p, 0, lastDrivePower, 0, 4);
        leftFront.setPower(p[MecanumMath.LF]);
        rightFront.setPower(p[MecanumMath.RF]);
        leftBack.setPower(p[MecanumMath.LB]);
        rightBack.setPower(p[MecanumMath.RB]);
    }

    public void stopDrive() {
        drive(0, 0, 0, 0);
    }

    public void resetDriveEncoders() {
        for (DcMotorEx m : driveMotors()) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public double[] driveTicks() {
        return new double[]{leftFront.getCurrentPosition(), rightFront.getCurrentPosition(),
                leftBack.getCurrentPosition(), rightBack.getCurrentPosition()};
    }

    public double forwardInches() {
        return MecanumMath.forward(driveTicks()) / DRIVE_TICKS_PER_INCH;
    }

    public double strafeRightInches() {
        return MecanumMath.strafeRight(driveTicks()) / DRIVE_TICKS_PER_INCH / STRAFE_MULTIPLIER;
    }

    /** Counter-clockwise positive, degrees. */
    public double headingDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    // ---------------- Intake ----------------

    /** Intake roller and both corner wheels (they reverse with it). */
    public void setIntake(double power) {
        power = Range.clip(power, -1, 1);
        lastIntakePower = power;
        intake.setPower(power);
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
    }

    // ---------------- Launcher (two motors on one flywheel) ----------------

    /**
     * velocity <= 0 lets the flywheel coast down. Both motors get the same target (each in velocity mode on its own
     * encoder, LAUNCHER_PIDF); a new target is only sent when it changes. After a pair fault (see
     * {@link #updateLauncherReady()}) both stay off until the op mode commands 0 once (FIRE / pre-spin let go).
     */
    public void setLauncherVelocity(double ticksPerSecond) {
        if (ticksPerSecond <= 0) {
            launcherFault = null;                               // let go: the next spin-up is a fresh try
        }
        if (ticksPerSecond <= 0 || launcherFault != null) {
            if (launcherOn) {
                for (DcMotorEx m : launchers()) {
                    m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    m.setPower(0);
                }
            }
            launcherOn = false;
            launcherTarget = 0;
            writtenTarget = -1;
            launcherGate.reset();
            pairCheck.reset();
            readyCached = false;
            readyAt = -1;
            return;
        }
        if (!launcherOn) {
            for (DcMotorEx m : launchers()) {
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                m.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);
            }
            launcherTimer.reset();
            launcherOn = true;
        }
        double newTarget = Range.clip(ticksPerSecond, 0, LAUNCHER_MAX_VELOCITY);
        if (newTarget != launcherTarget) {
            readyAt = -1;                                       // a new target must qualify on its own
        }
        launcherTarget = newTarget;
        if (launcherTarget != writtenTarget) {                  // each write is a hub transaction per motor
            launcher.setVelocity(launcherTarget);
            launcher2.setVelocity(launcherTarget);
            writtenTarget = launcherTarget;
        }
    }

    private DcMotorEx[] launchers() {
        return new DcMotorEx[]{launcher, launcher2};
    }

    public boolean isLauncherOn() {
        return launcherOn;
    }

    public double launcherTarget() {
        return launcherTarget;
    }

    /** Flywheel speed: the average of both motors' encoders (ticks/s at the motor = flywheel, 1:1). */
    public double launcherVelocity() {
        return (launcher.getVelocity() + launcher2.getVelocity()) / 2;
    }

    public double launcher1Velocity() {
        return launcher.getVelocity();
    }

    public double launcher2Velocity() {
        return launcher2.getVelocity();
    }

    /**
     * Indexer v2: the queue must HOLD - the flywheel is off, or more than QUEUE_HOLD_DIP_TPS below its target (a dip
     * deeper than the normal one between balls).
     */
    public boolean launcherDipTooDeep() {
        return !launcherOn || launcherVelocity() < launcherTarget - QUEUE_HOLD_DIP_TPS;
    }

    /** Launcher motor current (A). Not in the hub's bulk read: each call costs a hub transaction (~2-3 ms). */
    public double launcherCurrentAmps() {
        lastLauncherAmps = launcher.getCurrent(CurrentUnit.AMPS);
        return lastLauncherAmps;
    }

    /**
     * Ready to fire the next ball: speed within LAUNCHER_READY_TOLERANCE of the target for LAUNCHER_READY_HOLD_SEC
     * (see {@link LauncherGate}). Op modes decide once per loop with {@link #updateLauncherReady()} (right after setting
     * the speed); this returns that answer for READY_CACHE_SEC, so logs and checks in the same loop see the same one.
     * A new target speed always forces a fresh decision.
     */
    public boolean launcherReady() {
        if (readyAt >= 0 && clock.seconds() - readyAt < READY_CACHE_SEC) {
            return readyCached;
        }
        return updateLauncherReady();
    }

    /**
     * Re-decides now (with a fresh speed reading); launcherReady() calls this when its last answer is too old. Also
     * checks the two flywheel encoders against each other (FlywheelPairCheck): a fault stops both motors at once.
     */
    public boolean updateLauncherReady() {
        readyAt = clock.seconds();
        if (launcherOn) {
            String fault = pairCheck.update(readyAt, launcher1Velocity(), launcher2Velocity());
            if (fault != null) {
                setLauncherVelocity(-1);                        // both off (clears the latch: set it after)
                launcherFault = fault;
                readyAt = clock.seconds();
            }
        }
        if (!launcherOn) {
            launcherGate.reset();
            readyCached = false;
        } else {
            readyCached = launcherGate.update(readyAt, launcherTarget, launcherVelocity());
        }
        return readyCached;
    }

    /** Non-null after the two flywheel encoders disagreed: both motors are stopped until the launcher is turned off. */
    public String launcherFault() {
        return launcherFault;
    }

    /** True when the last {@link #launcherReady()} was only allowed by the 1 s fallback (speed not settled). */
    public boolean launcherReadyByFallback() {
        return launcherOn && launcherGate.usedFallback();
    }

    /** Non-null when the launcher has had time to spin up but its encoder reading makes no sense. */
    public String launcherWarning() {
        if (launcherFault != null) {
            return "Launcher STOPPED: " + launcherFault;
        }
        if (!launcherOn || launcherTimer.seconds() < LAUNCHER_ENCODER_WARNING_SEC) {
            return null;
        }
        double v = launcherVelocity();
        if (v < -50) {
            return "Launcher velocity negative: check motor/encoder polarity (both launcher motors)";
        }
        if (Math.abs(v) < 50) {
            return "Launcher reads ~0: check the launcher / launcher2 encoder cables and motor power";
        }
        return null;
    }

    // ---------------- Indexer v2: queue, gate, beams ----------------

    public void setIndexerPower(double power) {
        lastIndexerPower = Range.clip(power, -1, 1);
        indexer.setPower(lastIndexerPower);
    }

    public int indexerTicks() {
        return indexer.getCurrentPosition();
    }

    /** GATE_OPEN and GATE_CLOSED set: otherwise the gate is never moved and FIRE is refused. */
    public static boolean gateCalibrated() {
        return QueueFeeder.gateCalibrated(GATE_OPEN, GATE_CLOSED);
    }

    /** Sends the gate position only when it changes (a servo write is a hub transaction). Uncalibrated: nothing. */
    public void setGateOpen(boolean open) {
        if (!gateCalibrated() || (lastGateOpen != null && lastGateOpen == open)) {
            return;
        }
        gate.setPosition(open ? GATE_OPEN : GATE_CLOSED);
        lastGateOpen = open;
    }

    /** True = a ball breaks beam_staged (ball 1 at the gate). From the bulk read (see clearBulkCache). */
    public boolean stagedBroken() {
        lastStagedBroken = beamStaged.getState() != BEAM_BROKEN_IS_LOW;
        return lastStagedBroken;
    }

    /** True = a ball breaks beam_full (ball 4's spot just behind the intake). */
    public boolean fullBroken() {
        lastFullBroken = beamFull.getState() != BEAM_BROKEN_IS_LOW;
        return lastFullBroken;
    }

    // ---------------- Flower arm ----------------

    public static boolean armCalibrated() {
        return ArmCalibration.valid(ArmCalibration.deployPosition(), ArmCalibration.stowDirection());
    }

    public static double armStowPosition() {
        return ArmCalibration.stowPosition(ArmCalibration.deployPosition(), ArmCalibration.stowDirection());
    }

    public boolean deployArm() {
        if (!armCalibrated()) {
            return false;
        }
        flowerArm.setPosition(ArmCalibration.deployPosition());
        armState = ArmState.DEPLOYED;
        return true;
    }

    public boolean stowArm() {
        if (!armCalibrated()) {
            return false;
        }
        flowerArm.setPosition(armStowPosition());
        armState = ArmState.STOWED;
        return true;
    }

    public ArmState armState() {
        return armState;
    }

    public void stopMechanisms() {
        setIntake(0);
        setIndexerPower(0);
        setLauncherVelocity(0);
    }

    public void stopAll() {
        stopDrive();
        stopMechanisms();
    }
}

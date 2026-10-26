package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * All BIOBUZZ hardware and the low-level commands shared by TeleOp, Autonomous and Calibration.
 * The constructor only configures hardware; it never moves a motor or servo.
 */
public class BioBuzzRobot {
    public final DcMotorEx leftFront;
    public final DcMotorEx rightFront;
    public final DcMotorEx leftBack;
    public final DcMotorEx rightBack;
    public final DcMotorEx intake;
    public final DcMotorEx launcher;
    public final DcMotorEx indexer;
    public final CRServo leftIntakeServo;
    public final CRServo rightIntakeServo;
    public final Servo flowerArm;
    public final IMU imu;
    public final VoltageSensor battery;

    private boolean launcherOn = false;
    private double launcherTarget = 0;
    private final ElapsedTime launcherTimer = new ElapsedTime();

    public enum ArmState { UNKNOWN, STOWED, DEPLOYED }
    private ArmState armState = ArmState.UNKNOWN;

    public BioBuzzRobot(HardwareMap hw) {
        for (LynxModule hub : hw.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hw.get(DcMotorEx.class, LEFT_FRONT_DRIVE);
        rightFront = hw.get(DcMotorEx.class, RIGHT_FRONT_DRIVE);
        leftBack = hw.get(DcMotorEx.class, LEFT_BACK_DRIVE);
        rightBack = hw.get(DcMotorEx.class, RIGHT_BACK_DRIVE);
        intake = hw.get(DcMotorEx.class, INTAKE);
        launcher = hw.get(DcMotorEx.class, LAUNCHER);
        indexer = hw.get(DcMotorEx.class, INDEXER);
        leftIntakeServo = hw.get(CRServo.class, LEFT_INTAKE_SERVO);
        rightIntakeServo = hw.get(CRServo.class, RIGHT_INTAKE_SERVO);
        flowerArm = hw.get(Servo.class, FLOWER_ARM);
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

        indexer.setDirection(INDEXER_DIRECTION);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (flowerArm instanceof ServoImplEx) {
            ((ServoImplEx) flowerArm).setPwmRange(new PwmControl.PwmRange(ARM_PWM_MIN_US, ARM_PWM_MAX_US));
        }

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
    }

    public DcMotorEx[] driveMotors() {
        return new DcMotorEx[]{leftFront, rightFront, leftBack, rightBack};
    }

    // ---------------- Drive ----------------

    /** forward + = intake side, strafeRight + = robot right, turnCw + = clockwise (seen from above). */
    public void drive(double forward, double strafeRight, double turnCw, double scale) {
        double[] p = MecanumMath.wheelPowers(forward, strafeRight, turnCw, scale);
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

    public void setIntake(double power) {
        power = Range.clip(power, -1, 1);
        intake.setPower(power);
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
    }

    // ---------------- Launcher ----------------

    /** velocity <= 0 lets the flywheel coast down. */
    public void setLauncherVelocity(double ticksPerSecond) {
        if (ticksPerSecond <= 0) {
            if (launcherOn) {
                launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                launcher.setPower(0);
            }
            launcherOn = false;
            launcherTarget = 0;
            return;
        }
        if (!launcherOn) {
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);
            launcherTimer.reset();
            launcherOn = true;
        }
        launcherTarget = Range.clip(ticksPerSecond, 0, LAUNCHER_MAX_VELOCITY);
        launcher.setVelocity(launcherTarget);
    }

    public boolean isLauncherOn() {
        return launcherOn;
    }

    public double launcherTarget() {
        return launcherTarget;
    }

    public double launcherVelocity() {
        return launcher.getVelocity();
    }

    public boolean launcherReady() {
        return launcherOn && launcherVelocity() >= launcherTarget - LAUNCHER_READY_MARGIN;
    }

    /** Non-null when the launcher has had time to spin up but its encoder reading makes no sense. */
    public String launcherWarning() {
        if (!launcherOn || launcherTimer.seconds() < LAUNCHER_ENCODER_WARNING_SEC) {
            return null;
        }
        double v = launcherVelocity();
        if (v < -50) {
            return "Launcher velocity negative: check motor/encoder polarity";
        }
        if (Math.abs(v) < 50) {
            return "Launcher reads ~0: check launcher encoder cable";
        }
        return null;
    }

    // ---------------- Indexer ----------------

    public void setIndexerPower(double power) {
        indexer.setPower(Range.clip(power, -1, 1));
    }

    public int indexerTicks() {
        return indexer.getCurrentPosition();
    }

    // ---------------- Flower arm ----------------

    public static boolean armCalibrated() {
        double stow = armStowPosition();
        return ARM_DEPLOY_POSITION >= 0 && ARM_DEPLOY_POSITION <= 1 && stow >= 0 && stow <= 1;
    }

    public static double armStowPosition() {
        return ARM_DEPLOY_POSITION + ARM_STOW_DIRECTION * ARM_STOW_OFFSET;
    }

    public boolean deployArm() {
        if (!armCalibrated()) {
            return false;
        }
        flowerArm.setPosition(ARM_DEPLOY_POSITION);
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

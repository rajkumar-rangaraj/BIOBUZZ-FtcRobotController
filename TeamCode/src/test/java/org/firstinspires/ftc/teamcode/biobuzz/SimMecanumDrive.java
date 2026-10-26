package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

/**
 * Laptop simulation of the BIOBUZZ mecanum drivetrain: motor lag, X-layout mecanum kinematics,
 * encoders and IMU. Wiring faults can be injected to see what the robot would really do.
 *
 * World frame: X = the robot's starting FORWARD (towards the alliance wall), Y = starting LEFT,
 * heading counter-clockwise positive. The robot starts at (0, 0) touching the wall.
 */
class SimMecanumDrive implements DriveIO {
    static final double DT = 0.01;
    static final double MAX_WHEEL_SPEED_IPS = 312.0 / 60.0 * Math.PI * WHEEL_DIAMETER_IN;
    static final double MOTOR_LAG_SEC = 0.08;
    static final double TURN_RADIUS_IN = 13.5;     // (track width + wheelbase) / 2

    /** physicalWheel[i] = which real wheel the motor configured as wheel i drives (identity = correct wiring). */
    private final int[] physicalWheel;
    private final double timeLimitSec;

    private final double[] commanded = new double[4];
    private final double[] wheelSpeed = new double[4];
    private final double[] wheelDistance = new double[4];
    double t = 0;
    double x = 0;
    double y = 0;
    double headingRad = 0;
    double maxX = 0;

    SimMecanumDrive() {
        this(new int[]{0, 1, 2, 3}, 30.0);
    }

    SimMecanumDrive(int[] physicalWheel, double timeLimitSec) {
        this.physicalWheel = physicalWheel.clone();
        this.timeLimitSec = timeLimitSec;
    }

    double rightOfStart() {
        return -y;
    }

    @Override
    public boolean active() {
        return t < timeLimitSec;
    }

    @Override
    public double seconds() {
        return t;
    }

    private double[] encoderInches() {
        double[] e = new double[4];
        for (int i = 0; i < 4; i++) {
            e[i] = wheelDistance[physicalWheel[i]];
        }
        return e;
    }

    @Override
    public double forwardInches() {
        return MecanumMath.forward(encoderInches());
    }

    @Override
    public double strafeRightInches() {
        return MecanumMath.strafeRight(encoderInches()) / STRAFE_MULTIPLIER;
    }

    @Override
    public double headingDeg() {
        return MecanumMath.normalizeDegrees(Math.toDegrees(headingRad));
    }

    @Override
    public void drive(double forward, double strafeRight, double turnCw) {
        double[] p = MecanumMath.wheelPowers(forward, strafeRight, turnCw, 1.0);
        for (int i = 0; i < 4; i++) {
            commanded[physicalWheel[i]] = p[i];
        }
    }

    @Override
    public void loopTick() {
        double a = DT / (MOTOR_LAG_SEC + DT);
        for (int w = 0; w < 4; w++) {
            wheelSpeed[w] += (commanded[w] * MAX_WHEEL_SPEED_IPS - wheelSpeed[w]) * a;
            wheelDistance[w] += wheelSpeed[w] * DT;
        }
        double vForward = MecanumMath.forward(wheelSpeed);
        double vRight = MecanumMath.strafeRight(wheelSpeed) / STRAFE_MULTIPLIER;
        double omegaCw = MecanumMath.turnCw(wheelSpeed) / TURN_RADIUS_IN;
        x += (vForward * Math.cos(headingRad) + vRight * Math.sin(headingRad)) * DT;
        y += (vForward * Math.sin(headingRad) - vRight * Math.cos(headingRad)) * DT;
        headingRad -= omegaCw * DT;
        maxX = Math.max(maxX, x);
        t += DT;
    }

    /** Open-loop: hold a stick command for some time, then stop and let the robot settle. */
    void hold(double forward, double strafeRight, double turnCw, double seconds) {
        double end = t + seconds;
        while (t < end) {
            drive(forward, strafeRight, turnCw);
            loopTick();
        }
        drive(0, 0, 0);
        for (int i = 0; i < 50; i++) {
            loopTick();
        }
    }
}

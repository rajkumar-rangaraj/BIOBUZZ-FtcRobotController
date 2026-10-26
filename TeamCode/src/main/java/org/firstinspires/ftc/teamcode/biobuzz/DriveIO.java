package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * What the motion controller needs from a drivetrain. Implemented by the real robot (BioBuzzAutoBase)
 * and by the laptop simulation in the unit tests, so both run exactly the same route code.
 */
public interface DriveIO {
    boolean active();

    /** Monotonic time in seconds. */
    double seconds();

    double forwardInches();

    double strafeRightInches();

    /** Counter-clockwise positive, degrees. */
    double headingDeg();

    void drive(double forward, double strafeRight, double turnCw);

    /** Called once per control loop (telemetry on the robot, physics step in the simulation). */
    void loopTick();
}

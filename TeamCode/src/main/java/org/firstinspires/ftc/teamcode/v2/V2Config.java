package org.firstinspires.ftc.teamcode.v2;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * v2 robot configuration: every device name (must match the Driver Hub configuration) and fixed mounting facts.
 * Values that are MEASURED on the robot (directions, servo positions, ticks per ball...) are not here: they live in
 * {@link V2Calibration}, saved on the hub by "v2 Setup".
 */
public final class V2Config {
    private V2Config() {
    }

    // ---------------- motors (Control Hub: drive left + launchers; Expansion Hub: drive right + intake + queue)
    public static final String LEFT_FRONT = "left_front_drive";
    public static final String LEFT_BACK = "left_back_drive";
    public static final String RIGHT_FRONT = "right_front_drive";
    public static final String RIGHT_BACK = "right_back_drive";
    public static final String LAUNCHER = "launcher";            // flywheel motor 1
    public static final String LAUNCHER2 = "launcher2";          // flywheel motor 2, belted to the same disc
    public static final String QUEUE = "indexer";                // queue motor: bottom belts + top Gecko row
    public static final String INTAKE = "intake";                // intake roller

    /** every motor, in a fixed order (directions are saved per name) */
    public static final List<String> MOTORS = Collections.unmodifiableList(Arrays.asList(
            LEFT_FRONT, LEFT_BACK, RIGHT_FRONT, RIGHT_BACK, LAUNCHER, LAUNCHER2, QUEUE, INTAKE));

    // ---------------- servos
    public static final String LEFT_INTAKE_SERVO = "left_intake_servo";    // continuous: corner intake wheel
    public static final String RIGHT_INTAKE_SERVO = "right_intake_servo";  // continuous: corner intake wheel
    public static final String GATE = "gate";                              // position (servo mode): queue drop gate
    public static final String FLOWER_ARM = "flower_arm";                  // position (servo mode)

    public static final List<String> CR_SERVOS = Collections.unmodifiableList(Arrays.asList(
            LEFT_INTAKE_SERVO, RIGHT_INTAKE_SERVO));
    public static final List<String> POSITION_SERVOS = Collections.unmodifiableList(Arrays.asList(GATE, FLOWER_ARM));

    // ---------------- sensors
    public static final String BEAM_STAGED = "beam_staged";      // ball 1 at the gate
    public static final String BEAM_FULL = "beam_full";          // ball 4 at the lane foot
    public static final List<String> BEAMS = Collections.unmodifiableList(Arrays.asList(BEAM_STAGED, BEAM_FULL));
    public static final String IMU = "imu";
    public static final String PINPOINT = "pinpoint";            // optional
    public static final String WEBCAM = "Webcam 1";

    /** Control Hub mounted on its edge: logo LEFT, USB FORWARD (unchanged from v1). */
    public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // ---------------- mechanism facts used by calibration limits
    public static final double BALL_PITCH_MM = 73.0;              // one ball length in the queue
}

package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

/*
 * BioBuzz Arm Setup - guided flower-arm setup, one step at a time on the Driver Hub screen.
 * Saves the result on the Control Hub (ArmCalibration), so no code editing or re-deploy is needed.
 *
 *   1. arm off the servo hub (servo not powered)      A = next
 *   2. servo to the middle; fit the arm LEVEL          A = next
 *   3. D-pad: move the arm DOWN onto the pads          A = next
 *   4. Y = test up / down                              X = SAVE
 *   B goes back one step at any time.
 */
@TeleOp(name = "BioBuzz Arm Setup", group = "BioBuzz Test")
public class BioBuzzArmSetup extends OpMode {

    private enum Step { REMOVE, FIT_LEVEL, FIND_DOWN, TEST_SAVE, SAVED }

    private BioBuzzRobot robot;
    private Step step = Step.REMOVE;
    private double jog = ArmCalibration.SETUP_LEVEL_POSITION;
    private double[] result;
    private boolean atUp = false;
    private String message = "";

    @Override
    public void init() {
        robot = new BioBuzzRobot(hardwareMap);
        telemetry.addLine("BioBuzz Arm Setup");
        telemetry.addLine("Robot on a box. Press PLAY (the triangle) to start.");
        if (ArmCalibration.savedOnRobot()) {
            telemetry.addData("Saved now", "down %.3f, direction %+d",
                    ArmCalibration.deployPosition(), ArmCalibration.stowDirection());
        }
    }

    @Override
    public void loop() {
        boolean a = gamepad1.aWasPressed(), b = gamepad1.bWasPressed();
        boolean x = gamepad1.xWasPressed(), y = gamepad1.yWasPressed();

        switch (step) {
            case REMOVE:
                show("STEP 1 of 4: TAKE THE ARM OFF THE SERVO",
                        "Remove the 4 screws that hold the arm to the round servo hub.",
                        "Swing the arm by hand: it must move freely.",
                        "",
                        "Then press A.");
                if (a) {
                    jog = ArmCalibration.SETUP_LEVEL_POSITION;
                    robot.flowerArm.setPosition(jog);
                    step = Step.FIT_LEVEL;
                }
                break;

            case FIT_LEVEL:
                robot.flowerArm.setPosition(jog);
                show("STEP 2 of 4: FIT THE ARM LEVEL",
                        "The servo hub has turned to its middle and holds there.",
                        "Hold the arm STRAIGHT OUT FORWARD, level with the floor",
                        "(half-way between down on the pads and up on the posts).",
                        "Screw it to the hub with the 4 screws.",
                        "If the holes do not line up: pull the hub off and turn it 1 tooth.",
                        "",
                        "Then press A.     (B = back)");
                if (a) {
                    step = Step.FIND_DOWN;
                    message = "";
                } else if (b) {
                    step = Step.REMOVE;
                }
                break;

            case FIND_DOWN:
                if (gamepad1.dpadUpWasPressed()) {
                    jog += 0.010;
                } else if (gamepad1.dpadDownWasPressed()) {
                    jog -= 0.010;
                } else if (gamepad1.dpadRightWasPressed()) {
                    jog += 0.002;
                } else if (gamepad1.dpadLeftWasPressed()) {
                    jog -= 0.002;
                }
                jog = Range.clip(jog, 0.02, 0.98);
                robot.flowerArm.setPosition(jog);
                show("STEP 3 of 4: MOVE THE ARM DOWN ONTO THE PADS",
                        "D-pad = the + shaped arrow buttons on the LEFT of the controller.",
                        "  D-pad UP / DOWN    = big steps",
                        "  D-pad LEFT / RIGHT = small steps",
                        "One of them moves the arm DOWN. Use that one.",
                        "Stop when BOTH arms just touch the pads (wedge near the floor).",
                        "",
                        String.format(Locale.US, "Position: %.3f", jog),
                        message,
                        "Then press A.     (B = back)");
                if (a) {
                    result = ArmCalibration.fromDownPosition(jog);
                    if (result == null) {
                        message = Math.abs(jog - ArmCalibration.SETUP_LEVEL_POSITION) < 0.15
                                ? "!! The arm has not moved far enough to be on the pads yet."
                                : "!! Out of range: go back (B) and fit the arm 1 tooth closer to level.";
                    } else {
                        robot.flowerArm.setPosition(result[0]);
                        atUp = false;
                        step = Step.TEST_SAVE;
                    }
                } else if (b) {
                    step = Step.FIT_LEVEL;
                    jog = ArmCalibration.SETUP_LEVEL_POSITION;
                }
                break;

            case TEST_SAVE:
                if (y) {
                    atUp = !atUp;
                }
                double up = ArmCalibration.stowPosition(result[0], (int) result[1]);
                robot.flowerArm.setPosition(atUp ? up : result[0]);
                show("STEP 4 of 4: TEST, THEN SAVE",
                        "Press Y: the arm swings UP to the posts. Y again: back DOWN.",
                        "Check: it reaches the posts and the pads, and does NOT buzz or push hard.",
                        "",
                        String.format(Locale.US, "Down %.3f   Up %.3f   Direction %+d", result[0], up, (int) result[1]),
                        message,
                        "Good? Press X to SAVE.     (B = back to step 3)");
                if (x) {
                    try {
                        ArmCalibration.save(result[0], (int) result[1]);
                        robot.flowerArm.setPosition(result[0]);
                        step = Step.SAVED;
                    } catch (Exception e) {
                        message = "!! Could not save: " + e.getMessage();
                    }
                } else if (b) {
                    step = Step.FIND_DOWN;
                    jog = result[0];
                }
                break;

            case SAVED:
                show("SAVED - THE ARM IS SET UP",
                        String.format(Locale.US, "Down %.3f, direction %+d (stored on the robot).",
                                ArmCalibration.deployPosition(), ArmCalibration.stowDirection()),
                        "Press STOP. TeleOp now shows 'Flower arm: calibrated' and A works.",
                        "No code change is needed.");
                break;
        }
    }

    private void show(String... lines) {
        for (String l : lines) {
            telemetry.addLine(l);
        }
    }
}

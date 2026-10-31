package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

/*
 * BioBuzz Pinpoint Setup - measures where the two odometry pods are and which way they count, on the robot itself,
 * and saves it on the Control Hub (PinpointCalibration). Pods can be mounted wherever they fit.
 * Robot on the FLOOR (tiles), pods touching. One step per screen - the robot drives itself:
 *   1. keep still            A = start (zeroes the Pinpoint)
 *   2. A = drives FORWARD ~1.5 ft   (finds the forward pod and its direction)
 *   3. A = slides LEFT ~1 ft        (finds the strafe pod and its direction)
 *   4. A = spins 2 turns in place   (finds both pod positions)
 *   5. X = SAVE              B = back / stop at any time
 */
@TeleOp(name = "BioBuzz Pinpoint Setup", group = "BioBuzz Test")
public class BioBuzzPinpointSetup extends OpMode {

    private enum Step { STILL, SETTLE, FORWARD, FORWARD_MOVING, LEFT, LEFT_MOVING, SPIN_READY, SPINNING, RESULT, SAVED }

    private static final double SPIN_DEG = 720;
    private static final double SPIN_POWER = 0.35;
    private static final double MOVE_SEC = 1.0;
    private static final double SETTLE_SEC = 0.5;
    private double holdHeading;

    private BioBuzzRobot robot;
    private Pinpoint pp;
    private Step step = Step.STILL;
    private final ElapsedTime timer = new ElapsedTime();
    private int x0, y0;
    private boolean xRev, yRev;
    private double turned, lastHeading;
    private double xLeftMm, yFwdMm;
    private String message = "";

    @Override
    public void init() {
        robot = new BioBuzzRobot(hardwareMap);
        pp = Pinpoint.tryCreate(hardwareMap, true);
        telemetry.addLine("BioBuzz Pinpoint Setup");
        if (pp == null) {
            telemetry.addLine("!! No \"pinpoint\" in the robot configuration (Control Hub, I2C port, goBILDA Pinpoint).");
        } else {
            PinpointCalibration c = PinpointCalibration.load();
            telemetry.addData("Saved now", c.fromFile ? String.format(Locale.US, "X pod left %.0f mm, Y pod forward %.0f mm, rev %b/%b",
                    c.xPodLeftMm, c.yPodForwardMm, c.xReversed, c.yReversed) : "nothing saved yet");
            telemetry.addLine("Robot on the floor. Press PLAY.");
        }
    }

    @Override
    public void loop() {
        if (pp == null) {
            telemetry.addLine("!! No \"pinpoint\" in the robot configuration.");
            return;
        }
        pp.update();
        boolean a = gamepad1.aWasPressed(), b = gamepad1.bWasPressed(), x = gamepad1.xWasPressed();
        int dX = pp.rawX() - x0, dY = pp.rawY() - y0;
        double tpm = PinpointCalibration.ticksPerMm();

        switch (step) {
            case STILL:
                show("STEP 1 of 4: KEEP THE ROBOT STILL",
                        "Robot on the floor, both pod wheels touching. Hands off.",
                        "Pinpoint status: " + pp.status(),
                        "", "Press A.");
                if (a) {
                    pp.reset();
                    timer.reset();
                    step = Step.SETTLE;
                }
                break;

            case SETTLE:
                show("Zeroing the Pinpoint - keep still...", "Status: " + pp.status());
                if (timer.seconds() > 0.6 && pp.ok()) {
                    x0 = pp.rawX();
                    y0 = pp.rawY();
                    message = "";
                    step = Step.FORWARD;
                } else if (timer.seconds() > 3) {
                    show("!! Pinpoint not READY: " + pp.status(), "Check both pod cables (X and Y ports) and the I2C cable.");
                }
                break;

            case FORWARD:
                show("STEP 2 of 4: ROBOT DRIVES FORWARD BY ITSELF",
                        "Clear about 2 ft in FRONT of the intake. Press A: it drives forward ~1.5 ft slowly and stops.",
                        message, "(B = back)");
                if (a) {
                    x0 = pp.rawX();
                    y0 = pp.rawY();
                    timer.reset();
                    message = "";
                    step = Step.FORWARD_MOVING;
                } else if (b) {
                    step = Step.STILL;
                }
                break;

            case FORWARD_MOVING:
                if (move(0.3, 0, dX, dY)) {
                    String err = PinpointCalibration.forwardCheck(dX, dY);
                    if (err != null) {
                        message = "!! " + err;
                        step = Step.FORWARD;
                    } else {
                        xRev = dX < 0;
                        message = String.format(Locale.US, "Forward OK: pod X moved %.0f mm (%s)", Math.abs(dX) / tpm,
                                xRev ? "counts backwards - handled" : "normal");
                        step = Step.LEFT;
                    }
                }
                break;

            case LEFT:
                show("STEP 3 of 4: ROBOT SLIDES LEFT BY ITSELF",
                        "Clear about 2 ft on the robot's LEFT (left as seen from behind, looking at the intake).",
                        "Press A: it slides left ~1 ft slowly and stops.",
                        message, "(B = back)");
                if (a) {
                    x0 = pp.rawX();
                    y0 = pp.rawY();
                    timer.reset();
                    message = "";
                    step = Step.LEFT_MOVING;
                } else if (b) {
                    step = Step.FORWARD;
                }
                break;

            case LEFT_MOVING:
                if (move(0, -0.35, dX, dY)) {
                    String err = PinpointCalibration.leftCheck(dX, dY);
                    if (err != null) {
                        message = "!! " + err;
                        step = Step.LEFT;
                    } else {
                        yRev = dY < 0;
                        message = "";
                        step = Step.SPIN_READY;
                    }
                }
                break;

            case SPIN_READY:
                show("STEP 4 of 4: THE ROBOT WILL SPIN 2 TURNS IN PLACE",
                        "Clear 3 ft around it. It turns slowly by itself and stops.",
                        "", "Press A to start.   (B = back)");
                if (a) {
                    x0 = pp.rawX();
                    y0 = pp.rawY();
                    turned = 0;
                    lastHeading = pp.headingDeg();
                    step = Step.SPINNING;
                } else if (b) {
                    step = Step.LEFT;
                    x0 = pp.rawX();
                    y0 = pp.rawY();
                }
                break;

            case SPINNING:
                turned += MecanumMath.normalizeDegrees(pp.headingDeg() - lastHeading);
                lastHeading = pp.headingDeg();
                if (Math.abs(turned) < SPIN_DEG) {
                    robot.drive(0, 0, SPIN_POWER, 1.0);
                    timer.reset();
                } else {
                    robot.drive(0, 0, 0, 1.0);
                    if (timer.seconds() > 0.5) {                      // let it stop, then read the final counts
                        double[] off = PinpointCalibration.offsetsFromSpin((xRev ? -1 : 1) * dX, (yRev ? -1 : 1) * dY,
                                Math.toRadians(turned), tpm);
                        xLeftMm = off[0];
                        yFwdMm = off[1];
                        step = Step.RESULT;
                    }
                }
                if (b) {
                    robot.drive(0, 0, 0, 1.0);
                    step = Step.SPIN_READY;
                }
                show(String.format(Locale.US, "Spinning... %.0f of %.0f deg", Math.abs(turned), SPIN_DEG), "B = stop");
                break;

            case RESULT:
                robot.drive(0, 0, 0, 1.0);
                show("RESULT",
                        String.format(Locale.US, "Forward pod (port X): %.0f mm %s of centre, counts %s",
                                Math.abs(xLeftMm), xLeftMm >= 0 ? "LEFT" : "RIGHT", xRev ? "reversed" : "normal"),
                        String.format(Locale.US, "Strafe pod (port Y): %.0f mm %s of centre, counts %s",
                                Math.abs(yFwdMm), yFwdMm >= 0 ? "FORWARD" : "BEHIND", yRev ? "reversed" : "normal"),
                        Math.abs(xLeftMm) > 250 || Math.abs(yFwdMm) > 250
                                ? "!! Offsets look too big - redo the spin (B) with the wheels on the floor" : "",
                        message, "Press X to SAVE.   (B = spin again)");
                if (x) {
                    try {
                        PinpointCalibration c = new PinpointCalibration(xRev, yRev, xLeftMm, yFwdMm, true);
                        c.save();
                        pp.apply(c);
                        step = Step.SAVED;
                    } catch (Exception e) {
                        message = "!! Could not save: " + e.getMessage();
                    }
                } else if (b) {
                    step = Step.SPIN_READY;
                }
                break;

            case SAVED:
                show("SAVED on the robot.",
                        "Check: press nothing, push the robot forward 24 in -> X goes UP ~24 in; slide left -> Y goes UP.",
                        String.format(Locale.US, "Now: X %.1f in  Y %.1f in  heading %.1f deg", pp.xIn(), pp.yIn(), pp.headingDeg()),
                        "Press STOP. AUTO uses the Pinpoint from now on.");
                break;
        }
    }

    @Override
    public void stop() {
        robot.drive(0, 0, 0, 1.0);
    }

    /**
     * Drive for MOVE_SEC (holding the start heading with the Pinpoint gyro), then stop and wait SETTLE_SEC.
     * @return true once the move is over and the robot has stopped
     */
    private boolean move(double forward, double strafeRight, int dX, int dY) {
        double t = timer.seconds();
        if (t < MOVE_SEC) {
            if (t < 0.05) {
                holdHeading = pp.headingDeg();
            }
            double turn = MotionController.clip(-MecanumMath.normalizeDegrees(holdHeading - pp.headingDeg()) * 0.02, -0.15, 0.15);
            robot.drive(forward, strafeRight, turn, 1.0);
            show(forward != 0 ? "Driving forward..." : "Sliding left...",
                    String.format(Locale.US, "pod X %+d  pod Y %+d ticks", dX, dY), "B = stop");
            if (gamepad1.b) {
                robot.drive(0, 0, 0, 1.0);
                timer.reset();
                step = forward != 0 ? Step.FORWARD : Step.LEFT;
            }
            return false;
        }
        robot.drive(0, 0, 0, 1.0);
        show("Stopping...");
        return t > MOVE_SEC + SETTLE_SEC;
    }

    private void show(String... lines) {
        for (String l : lines) {
            if (!l.isEmpty()) {
                telemetry.addLine(l);
            }
        }
    }
}

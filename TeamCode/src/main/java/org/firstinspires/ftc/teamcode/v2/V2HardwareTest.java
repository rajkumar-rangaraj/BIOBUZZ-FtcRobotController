package org.firstinspires.ftc.teamcode.v2;

import android.util.Size;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;

/*
 * v2 Hardware Test - tests every motor, servo, sensor and the camera ONE AT A TIME.
 * Stand-alone on purpose: it uses only the hardware map (no v1 or v2 robot classes), so it works while everything
 * else is being built. A device missing from the configuration shows MISSING; the rest still work.
 *
 * Robot ON A BOX (wheels free), no balls, hands clear of the flywheel and the gate.
 *
 *   D-pad LEFT / RIGHT   page: MOTORS - SERVOS - SENSORS - CAMERA
 *   D-pad UP / DOWN      pick a device on the page
 *   MOTORS   hold RIGHT trigger = forward, LEFT trigger = reverse (power = trigger x the motor's limit).
 *            Releasing the trigger cuts the power at once (flywheels coast down on their own; they only RAMP UP).
 *            Power is also cut when the page changes or another device is picked;
 *            after a change the triggers must be released once before the new device responds.
 *            B = zero its encoder (only when its power is 0).
 *   SERVOS   continuous servos (corner intake wheels): triggers like a motor.
 *            position servos (gate, flower arm): RB / LB = target +-0.01 (hold Y: +-0.05),
 *            A = SEND the target (the servo only moves after A), X = let go (PWM off).
 *   SENSORS  beams, IMU (A = zero the yaw), Pinpoint (Y = zero position + its IMU: keep the robot STILL), hubs.
 *            Pinpoint shows RAW pod counts (no offsets / directions set): it checks the pods count, not calibration.
 *   CAMERA   starts the camera + AprilTags the first time the page opens; it streams only while this page is open.
 */
@TeleOp(name = "v2 Hardware Test", group = "v2")
public class V2HardwareTest extends LinearOpMode {

    // ---------------- device names (v2 configuration; keep in step with the Driver Hub config sheet)
    static final String[] MOTOR_NAMES = {"left_front_drive", "left_back_drive", "right_front_drive", "right_back_drive",
            "launcher", "launcher2", "indexer", "intake"};
    static final String[] MOTOR_LABEL = {"front-left wheel", "back-left wheel", "front-right wheel", "back-right wheel",
            "flywheel motor 1", "flywheel motor 2", "queue motor (belts + Geckos)", "intake roller"};
    // power limits: drive and intake gentle, the queue slow (balls / fingers), flywheels ramp up (steel disc)
    static final double[] MOTOR_LIMIT = {0.5, 0.5, 0.5, 0.5, 1.0, 1.0, 0.4, 0.8};
    static final boolean[] MOTOR_RAMP = {false, false, false, false, true, true, false, false};
    static final String[] CR_NAMES = {"left_intake_servo", "right_intake_servo"};
    static final String[] POS_NAMES = {"gate", "flower_arm"};
    static final String[] POS_LABEL = {"gate paddle (opens 90 deg TOWARD THE INTAKE)", "flower arm"};
    static final String[] BEAM_NAMES = {"beam_staged", "beam_full"};
    static final String[] BEAM_LABEL = {"ball 1 at the gate", "ball 4 at the lane foot"};
    static final String IMU_NAME = "imu", PINPOINT_NAME = "pinpoint", WEBCAM_NAME = "Webcam 1";
    // Control Hub mounted on its edge: logo LEFT, USB FORWARD (same as the v1 robot)
    static final RevHubOrientationOnRobot HUB_ORIENTATION = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    static final double SPIN_TPS = 100;               // flywheel 'still turning' threshold (ticks/s)
    static final double RAMP_PER_SEC = 0.8;            // flywheel power slew limit (power per second)

    enum Page { MOTORS, SERVOS, SENSORS, CAMERA }

    private final DcMotorEx[] motors = new DcMotorEx[MOTOR_NAMES.length];
    private final CRServo[] crServos = new CRServo[CR_NAMES.length];
    private final Servo[] posServos = new Servo[POS_NAMES.length];
    private final double[] posTarget = new double[POS_NAMES.length];
    private final boolean[] posSent = new boolean[POS_NAMES.length];
    private final DigitalChannel[] beams = new DigitalChannel[BEAM_NAMES.length];
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;
    private WebcamName webcam;
    private VisionPortal portal;
    private AprilTagProcessor tags;
    private List<LynxModule> hubs;
    private final List<String> missing = new ArrayList<>();

    private Page page = Page.MOTORS;
    private int sel = 0;
    private double rampPower = 0;
    private boolean armed = false;                     // triggers must be released after picking a device
    private boolean cameraFailed = false;
    private String cameraError = "";
    private String bNote = "";
    private boolean reverseWait = false;
    private double pairDir = 0;                        // flywheel pair: last driven direction, -1 / +1, 0 = unknown
    private int pairMotor = -1;                        // which launcher motor drove it (index), -1 = none
    private int loopCount = 0;
    private final double[] ampsCache = new double[MOTOR_NAMES.length];
    private final List<String> tableErrors = new ArrayList<>();
    private double battery = Double.NaN;
    private final Buttons pad = new Buttons();

    @Override
    public void runOpMode() {
        findDevices();
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule h : hubs) {
            h.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        while (opModeInInit()) {
            telemetry.addLine("v2 HARDWARE TEST - robot ON A BOX, no balls, hands clear.");
            telemetry.addData("Hubs", hubs.size() + (hubs.size() < 2 ? "  (Expansion Hub MISSING?)" : "  (Control + Expansion)"));
            telemetry.addData("Missing from the configuration", missing.isEmpty() ? "none" : String.join(", ", missing));
            for (String e : tableErrors) {
                telemetry.addLine("CODE ERROR: " + e);
            }
            telemetry.addLine("Press PLAY. Nothing moves until you hold a trigger or press A.");
            telemetry.update();
            sleep(50);
        }
        ElapsedTime loop = new ElapsedTime();
        while (opModeIsActive()) {
            double dt = Math.min(loop.seconds(), 0.1);
            loop.reset();
            pad.update(gamepad1);
            if (gamepad1.right_trigger < 0.05 && gamepad1.left_trigger < 0.05) {
                armed = true;
            }
            if (pad.dpadRight || pad.dpadLeft) {
                stopAllOutputs();
                int k = Page.values().length;
                page = Page.values()[(page.ordinal() + (pad.dpadRight ? 1 : k - 1)) % k];
                sel = 0;
            }
            int n = pageSize();
            if (n > 0 && (pad.dpadDown || pad.dpadUp)) {
                stopAllOutputs();
                sel = (sel + (pad.dpadDown ? 1 : n - 1)) % n;
            }
            syncCamera();
            telemetry.addLine("< " + page + " >   (D-pad L/R page, U/D device)");
            switch (page) {
                case MOTORS: motorPage(dt); break;
                case SERVOS: servoPage(); break;
                case SENSORS: sensorPage(); break;
                default: cameraPage(); break;
            }
            if (loopCount++ % 10 == 0) {
                battery = battery();
            }
            telemetry.addData("Battery", "%.2f V", battery);
            telemetry.update();
        }
        stopAllOutputs();
        if (portal != null) {
            portal.close();
        }
    }

    // ---------------- discovery: nothing here throws; a missing device is listed and skipped
    private void findDevices() {
        java.util.Arrays.fill(posTarget, 0.5);
        if (MOTOR_LABEL.length != MOTOR_NAMES.length || MOTOR_LIMIT.length != MOTOR_NAMES.length
                || MOTOR_RAMP.length != MOTOR_NAMES.length) {
            tableErrors.add("MOTOR_NAMES / MOTOR_LABEL / MOTOR_LIMIT / MOTOR_RAMP lengths differ");
        }
        if (POS_LABEL.length != POS_NAMES.length || posTarget.length != POS_NAMES.length || posSent.length != POS_NAMES.length) {
            tableErrors.add("POS_NAMES / POS_LABEL / posTarget / posSent lengths differ");
        }
        if (BEAM_LABEL.length != BEAM_NAMES.length) {
            tableErrors.add("BEAM_NAMES / BEAM_LABEL lengths differ");
        }
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            motors[i] = hardwareMap.tryGet(DcMotorEx.class, MOTOR_NAMES[i]);
            if (motors[i] == null) {
                missing.add(MOTOR_NAMES[i]);
            } else {
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motors[i].setDirection(DcMotor.Direction.FORWARD);   // raw: shows what FORWARD does on each motor
                motors[i].setPower(0);
            }
        }
        for (int i = 0; i < CR_NAMES.length; i++) {
            crServos[i] = hardwareMap.tryGet(CRServo.class, CR_NAMES[i]);
            if (crServos[i] == null) {
                missing.add(CR_NAMES[i]);
            } else {
                crServos[i].setPower(0);
            }
        }
        for (int i = 0; i < POS_NAMES.length; i++) {
            posServos[i] = hardwareMap.tryGet(Servo.class, POS_NAMES[i]);
            if (posServos[i] == null) {
                missing.add(POS_NAMES[i]);
            } else if (posServos[i] instanceof ServoImplEx) {
                ((ServoImplEx) posServos[i]).setPwmDisable();     // must not jump anywhere on INIT
            }
        }
        for (int i = 0; i < BEAM_NAMES.length; i++) {
            beams[i] = hardwareMap.tryGet(DigitalChannel.class, BEAM_NAMES[i]);
            if (beams[i] == null) {
                missing.add(BEAM_NAMES[i]);
            } else {
                beams[i].setMode(DigitalChannel.Mode.INPUT);
            }
        }
        imu = hardwareMap.tryGet(IMU.class, IMU_NAME);
        if (imu == null) {
            missing.add(IMU_NAME);
        } else {
            imu.initialize(new IMU.Parameters(HUB_ORIENTATION));
        }
        pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, PINPOINT_NAME);
        if (pinpoint == null) {
            missing.add(PINPOINT_NAME + " (optional)");
        }
        webcam = hardwareMap.tryGet(WebcamName.class, WEBCAM_NAME);
        if (webcam == null) {
            missing.add(WEBCAM_NAME);
        }
    }

    private int pageSize() {
        switch (page) {
            case MOTORS: return MOTOR_NAMES.length;
            case SERVOS: return CR_NAMES.length + POS_NAMES.length;
            default: return 0;
        }
    }

    // ---------------- MOTORS
    private void motorPage(double dt) {
        DcMotorEx m = motors[sel];
        reverseWait = false;
        double want = armed ? (gamepad1.right_trigger - gamepad1.left_trigger) * MOTOR_LIMIT[sel] : 0;
        if (!MOTOR_RAMP[sel]) {
            rampPower = want;
        } else if (want == 0 || (rampPower != 0 && Math.signum(want) != Math.signum(rampPower))) {
            rampPower = 0;                                     // release or direction change: cut at once (it coasts)
        } else if (m != null && Math.abs(m.getVelocity()) > SPIN_TPS
                && (pairMotor != sel || Math.signum(want) != pairDir)) {
            // the flywheel pair shares one disc: while it still turns, only the motor that last drove it may add power,
            // and only the same way (unknown / the other motor's direction = unsafe: the motors may be mirrored)
            rampPower = 0;                                     // disc still turning: wait
            reverseWait = true;
        } else if (Math.abs(want) > Math.abs(rampPower)) {
            rampPower += Math.copySign(Math.min(RAMP_PER_SEC * dt, Math.abs(want) - Math.abs(rampPower)), want);  // ramp UP
        } else {
            rampPower = want;                                  // less power: at once
        }
        if (MOTOR_RAMP[sel] && rampPower != 0) {
            pairDir = Math.signum(rampPower);           // direction the disc was really driven (not the encoder sign)
            pairMotor = sel;
        } else if (pairMotor >= 0 && pairStopped()) {
            pairDir = 0;                                // forget it only once the disc has stopped
            pairMotor = -1;
        }
        if (m != null) {
            m.setPower(rampPower);
            if (pad.b) {
                bNote = rampPower == 0 ? "" : "B ignored: release the trigger first";
            }
            if (pad.b && rampPower == 0) {
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            if (motors[i] != null && (i == sel || loopCount % 10 == 0)) {
                ampsCache[i] = amps(motors[i]);           // current is not in the bulk read: one hub call each
            }
            String mark = i == sel ? ">> " : "   ";
            if (motors[i] == null) {
                telemetry.addLine(mark + MOTOR_NAMES[i] + ": MISSING");
                continue;
            }
            telemetry.addLine(String.format("%s%-17s port %d  enc %7d  %6.0f t/s  %4.2f A", mark, MOTOR_NAMES[i],
                    motors[i].getPortNumber(), motors[i].getCurrentPosition(), motors[i].getVelocity(), ampsCache[i]));
        }
        telemetry.addLine("");
        telemetry.addData("Selected", "%s = %s", MOTOR_NAMES[sel], MOTOR_LABEL[sel]);
        if (!bNote.isEmpty()) {
            telemetry.addLine(bNote);
        }
        telemetry.addData("Power", "%.2f (limit %.1f)%s%s", rampPower, MOTOR_LIMIT[sel], MOTOR_RAMP[sel] ? ", ramped" : "",
                armed ? "" : "   RELEASE the triggers first");
        if (reverseWait) {
            telemetry.addLine("WAIT for the flywheel to stop before reversing it.");
        }
        telemetry.addLine("CHECK: the right part moves; its encoder counts UP for + power (if DOWN: note 'reverse'); "
                + "current stays low when it turns freely.");
        if (MOTOR_NAMES[sel].startsWith("launcher")) {
            telemetry.addLine("Flywheel pair: both encoders move (one motor back-drives the other through the belts).");
        }
    }

    // ---------------- SERVOS
    private void servoPage() {
        double step = gamepad1.y ? 0.05 : 0.01;
        for (int i = 0; i < CR_NAMES.length; i++) {
            if (crServos[i] != null) {
                crServos[i].setPower(sel == i && armed ? gamepad1.right_trigger - gamepad1.left_trigger : 0);
            }
        }
        int p = sel - CR_NAMES.length;
        if (p >= 0 && posServos[p] != null) {
            boolean moved = false;
            if (pad.rb) { posTarget[p] = Math.min(1, posTarget[p] + step); moved = true; }
            if (pad.lb) { posTarget[p] = Math.max(0, posTarget[p] - step); moved = true; }
            if (pad.a) {
                posServos[p].setPosition(posTarget[p]);          // pulse width first, then switch PWM on
                if (posServos[p] instanceof ServoImplEx) {
                    ((ServoImplEx) posServos[p]).setPwmEnable();
                }
                posSent[p] = true;
                moved = true;
            }
            if (pad.x && posServos[p] instanceof ServoImplEx) {
                ((ServoImplEx) posServos[p]).setPwmDisable();
                posSent[p] = false;
            }
            if (posSent[p] && moved) {
                posServos[p].setPosition(posTarget[p]);          // once sent, it follows the bumpers live
            }
        }
        for (int i = 0; i < CR_NAMES.length; i++) {
            telemetry.addLine((sel == i ? ">> " : "   ") + CR_NAMES[i] + " (continuous): "
                    + (crServos[i] == null ? "MISSING" : String.format("power %.2f", crServos[i].getPower())));
        }
        for (int i = 0; i < POS_NAMES.length; i++) {
            telemetry.addLine((sel == CR_NAMES.length + i ? ">> " : "   ") + POS_NAMES[i] + " (position): "
                    + (posServos[i] == null ? "MISSING" : String.format("target %.2f  %s", posTarget[i],
                    posSent[i] ? "SENT (follows RB/LB)" : "not sent - press A")));
        }
        telemetry.addLine("");
        if (p < 0) {
            telemetry.addLine("Triggers spin it. CHECK: the corner wheel pulls a ball IN for + power.");
        } else {
            telemetry.addData("Selected", POS_LABEL[p]);
            telemetry.addLine("RB/LB target, Y = bigger steps, A = SEND, X = let go. Start near 0.5, move in small steps,");
            telemetry.addLine("write down the CLOSED and OPEN numbers (gate) / STOWED and DEPLOYED (arm).");
        }
    }

    // ---------------- SENSORS
    private void sensorPage() {
        for (int i = 0; i < BEAM_NAMES.length; i++) {
            if (beams[i] == null) {
                telemetry.addLine(BEAM_NAMES[i] + ": MISSING");
            } else {
                telemetry.addLine(String.format("%s (%s): %s", BEAM_NAMES[i], BEAM_LABEL[i], beams[i].getState() ? "HIGH" : "LOW"));
            }
        }
        telemetry.addLine("   CHECK: put a ball in each beam - the reading must flip (Adafruit 2167: LOW = beam broken).");
        telemetry.addLine("   Always LOW with no ball: check the emitter power and the receiver wiring first.");
        if (imu == null) {
            telemetry.addLine("IMU: MISSING");
        } else {
            if (pad.a) imu.resetYaw();
            YawPitchRollAngles a = imu.getRobotYawPitchRollAngles();
            telemetry.addLine(String.format("IMU: yaw %6.1f  pitch %5.1f  roll %5.1f deg   (A = zero yaw)",
                    a.getYaw(AngleUnit.DEGREES), a.getPitch(AngleUnit.DEGREES), a.getRoll(AngleUnit.DEGREES)));
            telemetry.addLine("   CHECK: turn the robot LEFT by hand - yaw goes UP. Robot flat: pitch and roll near 0.");
        }
        if (pinpoint == null) {
            telemetry.addLine("Pinpoint: not configured (optional)");
        } else {
            if (pad.y) pinpoint.resetPosAndIMU();             // robot must be still for ~0.25 s
            pinpoint.update();
            telemetry.addLine(String.format("Pinpoint %s: encX %d  encY %d  heading %.1f deg", pinpoint.getDeviceStatus(),
                    pinpoint.getEncoderX(), pinpoint.getEncoderY(), pinpoint.getHeading(AngleUnit.DEGREES)));
            telemetry.addLine("   CHECK: push the robot forward - one pod counts; sideways - the other. (Y = zero, keep still)");
            telemetry.addLine("   RAW counts: no pod offsets / directions here - do not read position or heading as calibrated.");
        }
        StringBuilder h = new StringBuilder();
        for (LynxModule m : hubs) {
            h.append(m.getDeviceName()).append(m.isParent() ? " (parent)  " : "  ");
        }
        telemetry.addData("Hubs", h.toString());
    }

    // ---------------- CAMERA
    private void cameraPage() {
        if (webcam == null) {
            telemetry.addLine(WEBCAM_NAME + ": MISSING from the configuration (check the USB cable and the config).");
            return;
        }
        if (cameraFailed) {
            telemetry.addData("Camera start FAILED (restart the OpMode to retry)", cameraError);
            return;
        }
        if (portal == null) {
            try {
                AprilTagProcessor p = new AprilTagProcessor.Builder()
                        .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()).build();
                portal = new VisionPortal.Builder().setCamera(webcam).setCameraResolution(new Size(640, 480))
                        .addProcessor(p).build();
                tags = p;
            } catch (Exception e) {
                cameraFailed = true;                            // do not retry every loop
                cameraError = String.valueOf(e.getMessage());
                tags = null;
                portal = null;
                telemetry.addData("Camera start FAILED", cameraError);
                return;
            }
        }
        telemetry.addData("Camera", "%s  %.0f fps", portal.getCameraState(), portal.getFps());
        List<AprilTagDetection> d = tags.getDetections();
        telemetry.addData("AprilTags seen", d.size());
        for (AprilTagDetection t : d) {
            String id = t instanceof AprilTagSingleDetection ? String.valueOf(((AprilTagSingleDetection) t).id)
                    : t instanceof AprilTagClusterDetection && ((AprilTagClusterDetection) t).metadata != null
                    ? ((AprilTagClusterDetection) t).metadata.name : "cluster";
            if (t.ftcPose != null) {
                telemetry.addLine(String.format("  tag %s  range %.1f in  bearing %.1f deg  elevation %.1f deg", id,
                        t.ftcPose.range, t.ftcPose.bearing, t.ftcPose.elevation));
            } else {
                telemetry.addLine("  tag " + id + " (not in the tag library)");
            }
        }
        telemetry.addLine("CHECK: STREAMING at about 30 fps; hold a tag in front of the lens - its id shows.");
    }

    // ---------------- helpers
    /** both flywheel motors (the ramped ones) turn slower than SPIN_TPS; a missing motor counts as stopped */
    private boolean pairStopped() {
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            if (MOTOR_RAMP[i] && motors[i] != null && Math.abs(motors[i].getVelocity()) > SPIN_TPS) {
                return false;
            }
        }
        return true;
    }

    /** the camera streams only while the CAMERA page is open; checked against its real state every loop */
    private void syncCamera() {
        if (portal == null) {
            return;
        }
        VisionPortal.CameraState st = portal.getCameraState();
        try {
            if (page != Page.CAMERA && st == VisionPortal.CameraState.STREAMING) {
                portal.stopStreaming();
            } else if (page == Page.CAMERA && st == VisionPortal.CameraState.CAMERA_DEVICE_READY) {
                portal.resumeStreaming();
            }
        } catch (Exception e) {
            // a transition was still in progress: try again next loop
        }
    }

    private void stopAllOutputs() {
        rampPower = 0;
        armed = false;
        for (DcMotorEx m : motors) {
            if (m != null) m.setPower(0);
        }
        for (CRServo s : crServos) {
            if (s != null) s.setPower(0);
        }
    }

    private static double amps(DcMotorEx m) {
        try {
            return m.getCurrent(CurrentUnit.AMPS);
        } catch (Exception e) {
            return Double.NaN;
        }
    }

    private double battery() {
        double v = Double.NaN;
        for (VoltageSensor s : hardwareMap.voltageSensor) {
            double x = s.getVoltage();
            if (x > 0 && (Double.isNaN(v) || x < v)) v = x;
        }
        return v;
    }

    /** rising edges of the gamepad buttons used here */
    static final class Buttons {
        boolean dpadUp, dpadDown, dpadLeft, dpadRight, a, b, x, y, rb, lb;
        private boolean pu, pd, pl, pr, pa, pb, px, py, prb, plb;

        void update(Gamepad g) {
            dpadUp = g.dpad_up && !pu;
            dpadDown = g.dpad_down && !pd;
            dpadLeft = g.dpad_left && !pl;
            dpadRight = g.dpad_right && !pr;
            a = g.a && !pa;
            b = g.b && !pb;
            x = g.x && !px;
            y = g.y && !py;
            rb = g.right_bumper && !prb;
            lb = g.left_bumper && !plb;
            pu = g.dpad_up; pd = g.dpad_down; pl = g.dpad_left; pr = g.dpad_right;
            pa = g.a; pb = g.b; px = g.x; py = g.y; prb = g.right_bumper; plb = g.left_bumper;
        }
    }
}

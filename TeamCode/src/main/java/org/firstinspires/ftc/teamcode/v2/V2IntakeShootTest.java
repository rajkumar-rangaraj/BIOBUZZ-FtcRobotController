package org.firstinspires.ftc.teamcode.v2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * v2 Intake-Queue-Shoot Test: the first ball-through test with only 4 motors connected (intake, queue, 2 flywheel
 * motors). Nothing else needs to be in the configuration. Robot on a box, a net or a wall in front of the flywheel.
 *
 * Controls (gamepad 1):
 *   A        flywheel ON (ramps up to the set power)        B  flywheel OFF (coasts)
 *   BACK     (flywheel off) which flywheel motor runs: BOTH / ONLY launcher / ONLY launcher2. Run each ALONE first:
 *            the wheel must turn the same (throwing) way for both; then use BOTH. Two motors pushing against each other
 *            stall and get hot (current > ~4 A each at low speed = fighting: stop at once).
 *   D-pad L / R   flywheel power -/+ 0.05 (0.30..0.90)
 *   RB (hold)     intake IN (auto-unjams a side-by-side pair)  LB (hold)  intake OUT
 *   RT (hold)     queue toward the flywheel (power = trigger, up to full)
 *   LT (hold)     queue back toward the intake (max 0.4)
 *   with the flywheel OFF only:
 *     X  flip the intake direction   Y  flip the queue direction
 *     D-pad UP  flip launcher2 against launcher   D-pad DOWN  flip both flywheel motors
 * The screen shows both flywheel speeds: they must have the SAME sign and about the same size. If launcher2 shows the
 * opposite sign, the motors fight each other: press B, then D-pad UP.
 * Write down the final directions (shown on screen) for v2 Setup.
 */
@TeleOp(name = "v2 Intake Queue Shoot Test", group = "v2")
public class V2IntakeShootTest extends LinearOpMode {

    static final double RAMP_PER_SEC = 0.6;          // flywheel power ramp up (steel disc: no step to full power)
    static final double QUEUE_MAX = 1.0, QUEUE_BACK_MAX = 0.5, INTAKE_POWER = 0.6;   // queue faster than the intake:
                                                                                     // it pulls balls apart (single file)
    // pair-jam unjam: two balls side by side wedge at the lane mouth. Intake current above JAM_A for JAM_SEC while
    // pulling in -> reverse UNJAM_SEC, then pull in again. CALIBRATE JAM_A from the V2INT log (free vs wedged).
    static final double JAM_A = 3.0, JAM_SEC = 0.3, UNJAM_SEC = 0.25, UNJAM_POWER = 0.6;

    @Override
    public void runOpMode() {
        DcMotorEx fly1 = hardwareMap.tryGet(DcMotorEx.class, V2Config.LAUNCHER);
        DcMotorEx fly2 = hardwareMap.tryGet(DcMotorEx.class, V2Config.LAUNCHER2);
        DcMotorEx queue = hardwareMap.tryGet(DcMotorEx.class, V2Config.QUEUE);
        DcMotorEx intake = hardwareMap.tryGet(DcMotorEx.class, V2Config.INTAKE);
        for (DcMotorEx m : new DcMotorEx[]{fly1, fly2, queue, intake}) {
            if (m != null) {
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setPower(0);
            }
        }
        for (DcMotorEx m : new DcMotorEx[]{fly1, fly2}) {
            if (m != null) {
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);       // coast, never brake the disc
            }
        }
        for (DcMotorEx m : new DcMotorEx[]{queue, intake}) {
            if (m != null) {
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
        boolean flyRev = false, fly2Flip = true, queueRev = false, intakeRev = false;   // launcher2 is mounted mirrored
        double setPower = 0.55, flyPower = 0;
        boolean flyOn = false;
        int which = 1;                                   // 0 BOTH, 1 ONLY launcher, 2 ONLY launcher2 (start safe)
        boolean pBack = false;
        double stallSince = -1;
        int stallTrips = 0;
        double jamSince = -1, unjamUntil = -1;
        int unjams = 0;
        final String[] WHICH = {"BOTH", "ONLY launcher", "ONLY launcher2"};
        boolean pA = false, pB = false, pX = false, pY = false, pU = false, pD = false, pL = false, pR = false;

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("v2 Intake Queue Shoot Test - robot on a box, hands clear of the flywheel");
            telemetry.addData("found", "launcher %s, launcher2 %s, indexer %s, intake %s",
                    fly1 != null ? "OK" : "MISSING", fly2 != null ? "OK" : "MISSING",
                    queue != null ? "OK" : "MISSING", intake != null ? "OK" : "MISSING");
            telemetry.update();
            sleep(50);
        }
        ElapsedTime loop = new ElapsedTime();
        while (opModeIsActive()) {
            double dt = Math.min(loop.seconds(), 0.1);
            loop.reset();
            boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x, y = gamepad1.y;
            boolean up = gamepad1.dpad_up, dn = gamepad1.dpad_down, lf = gamepad1.dpad_left, rt = gamepad1.dpad_right;
            if (a && !pA) flyOn = true;
            if (b && !pB) {
                flyOn = false;
                flyPower = 0;
            }
            if (lf && !pL) setPower = Math.max(0.30, setPower - 0.05);
            if (rt && !pR) setPower = Math.min(0.90, setPower + 0.05);
            boolean idle = !flyOn && flyPower == 0;
            if (idle) {
                if (x && !pX) intakeRev = !intakeRev;
                if (y && !pY) queueRev = !queueRev;
                if (up && !pU) fly2Flip = !fly2Flip;
                if (dn && !pD) flyRev = !flyRev;
                if (gamepad1.back && !pBack) which = (which + 1) % 3;
            }
            pBack = gamepad1.back;
            pA = a; pB = b; pX = x; pY = y; pU = up; pD = dn; pL = lf; pR = rt;

            // flywheel: ramp up only, cut at once
            double want = flyOn ? setPower : 0;
            flyPower = want > flyPower ? Math.min(want, flyPower + RAMP_PER_SEC * dt) : want;
            double p1 = flyRev ? -flyPower : flyPower;
            double p2 = fly2Flip ? -p1 : p1;
            if (fly1 != null) fly1.setPower(which == 2 ? 0 : p1);
            if (fly2 != null) fly2.setPower(which == 1 ? 0 : p2);
            // stall guard: high current with the wheel hardly turning for 0.4 s = cut the flywheel (protects the motors)
            double c1 = fly1 != null ? fly1.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS) : 0;
            double c2 = fly2 != null ? fly2.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS) : 0;
            double w1 = fly1 != null ? fly1.getVelocity() : 0, w2 = fly2 != null ? fly2.getVelocity() : 0;
            if (flyPower > 0.25 && Math.max(c1, c2) > 3.5 && Math.max(Math.abs(w1), Math.abs(w2)) < 400) {
                if (stallSince < 0) stallSince = getRuntime();
                if (getRuntime() - stallSince > 0.4) {
                    flyOn = false;
                    flyPower = 0;
                    if (fly1 != null) fly1.setPower(0);
                    if (fly2 != null) fly2.setPower(0);
                    stallTrips++;
                    stallSince = -1;
                }
            } else {
                stallSince = -1;
            }
            if (flyPower > 0 || Math.max(c1, c2) > 0.3) {
                com.qualcomm.robotcore.util.RobotLog.ii("V2FLY", "mode=%s power=%.2f p1=%.2f p2=%.2f v1=%.0f v2=%.0f a1=%.2f a2=%.2f trips=%d",
                        WHICH[which], flyPower, which == 2 ? 0 : p1, which == 1 ? 0 : p2, w1, w2, c1, c2, stallTrips);
            }

            double qp = 0;
            if (gamepad1.right_trigger > 0.05) qp = gamepad1.right_trigger * QUEUE_MAX;
            else if (gamepad1.left_trigger > 0.05) qp = -gamepad1.left_trigger * QUEUE_BACK_MAX;
            if (queue != null) queue.setPower(queueRev ? -qp : qp);
            double ip = gamepad1.right_bumper ? INTAKE_POWER : gamepad1.left_bumper ? -INTAKE_POWER : 0;
            double ia = intake != null ? intake.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS) : 0;
            double now = getRuntime();
            if (unjamUntil > now) {
                ip = -UNJAM_POWER;                                   // knocking one ball of the pair back
            } else if (ip > 0 && ia > JAM_A) {
                if (jamSince < 0) jamSince = now;
                if (now - jamSince > JAM_SEC) {
                    unjamUntil = now + UNJAM_SEC;
                    unjams++;
                    jamSince = -1;
                }
            } else {
                jamSince = -1;
            }
            if (ip != 0 || ia > 0.3) {
                com.qualcomm.robotcore.util.RobotLog.ii("V2INT", "intake power=%.2f amps=%.2f queue=%.2f unjams=%d", ip, ia, qp, unjams);
            }
            if (intake != null) intake.setPower(intakeRev ? -ip : ip);

            double v1 = fly1 != null ? fly1.getVelocity() : 0, v2 = fly2 != null ? fly2.getVelocity() : 0;
            // seen from the disc: launcher2's encoder counts the other way when it is mounted mirrored
            double v2disc = fly2Flip ? -v2 : v2;
            telemetry.addData("Flywheel", "%s  power %.2f (set %.2f, D-pad L/R)  motors: %s (BACK)", flyOn ? "ON" : "OFF", flyPower, setPower, WHICH[which]);
            double a1 = fly1 != null ? fly1.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS) : 0;
            double a2 = fly2 != null ? fly2.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS) : 0;
            telemetry.addData("Current A", "launcher %.1f   launcher2 %.1f", a1, a2);
            if (stallTrips > 0) {
                telemetry.addLine("!! STALL CUT-OFF tripped " + stallTrips + "x: the flywheel was stopped to protect the motors");
            }
            if (flyPower > 0.2 && Math.max(a1, a2) > 4.0 && Math.abs(v1) < 600) {
                telemetry.addLine("!! HIGH CURRENT, LOW SPEED: motors fighting or the wheel is stuck - press B NOW");
            }
            telemetry.addData("Speed t/s", "launcher %.0f   launcher2 %.0f (as the disc: %.0f)", v1, v2, v2disc);
            if (flyPower > 0.2 && Math.abs(v1) > 100 && Math.signum(v1) != Math.signum(v2disc)) {
                telemetry.addLine("!! launcher2 FIGHTS launcher: press B, then D-pad UP");
            }
            telemetry.addData("Queue", "power %.2f%s (RT forward, LT back)", qp, queueRev ? " [flipped]" : "");
            telemetry.addData("Intake", "power %.2f  %.1f A%s (RB in, LB out)  auto-unjams %d", ip, ia, intakeRev ? " [flipped]" : "", unjams);
            telemetry.addData("Directions to note", "flywheel %s, launcher2 %s, queue %s, intake %s",
                    flyRev ? "REVERSE" : "FORWARD", fly2Flip ? "opposite to launcher" : "same as launcher",
                    queueRev ? "REVERSE" : "FORWARD", intakeRev ? "REVERSE" : "FORWARD");
            telemetry.addLine(idle ? "X / Y / D-pad UP / DOWN flip directions" : "B = stop the flywheel before flipping");
            telemetry.update();
        }
        for (DcMotorEx m : new DcMotorEx[]{fly1, fly2, queue, intake}) {
            if (m != null) m.setPower(0);
        }
    }
}

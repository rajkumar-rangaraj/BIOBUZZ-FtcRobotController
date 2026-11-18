package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * BioBuzz Tune - tune AIM and the QUEUE FIRE POWER on the robot, no code change. SHARE saves; TeleOp and AUTO use them.
 *
 *   Sticks, LB                 drive (LB = slow)
 *   Square / X (hold)          aim at our CELL (+ drive to the SHOT SPOT distance if one is saved)
 *   Triangle / Y               flywheel ON / OFF (SHOT SPOT speed, or NEAR)
 *   R1 / RB (hold)             FIRE a volley with the values below (QueueFeeder: gate opens once the flywheel is
 *                              ready, the queue runs at the fire power, holds only on a deep dip)
 *   Triggers                   intake in / out (the queue loads while the intake pulls in)
 *   Circle / B (hold)          unjam (queue backwards)
 *   D-pad LEFT / RIGHT         AIM TRIM: shots go more LEFT / RIGHT (0.5 deg per press)
 *   D-pad UP / DOWN            QUEUE FIRE POWER +/- 0.05 (faster = balls closer together; the flywheel must keep up)
 *   Cross / A                  reset the ball counters
 *   SHARE / Back               SAVE trim and fire power on the robot
 *
 * The windmill's pause / empty wait (ShotFeeder) are gone with indexer v2: a volley has no per-ball pause.
 */
@TeleOp(name = "BioBuzz Tune", group = "BioBuzz Test")
public class BioBuzzTune extends OpMode {

    private BioBuzzRobot robot;
    private HiveVision vision;
    private TeleOpSettings settings;
    private QueueFeeder queue;
    private final IntakeGuard intakeGuard = IntakeGuard.standard();
    private double trim, firePower;
    private boolean launcherOn = false;
    private int volleys = 0, jams = 0, beamBalls = 0;
    private final BallCounter ballCounter = BallCounter.forQueue();   // flywheel dips: cross-check of the beam count
    private String note = "";
    private final ElapsedTime noteTimer = new ElapsedTime();
    private final ElapsedTime clock = new ElapsedTime();

    @Override
    public void init() {
        robot = new BioBuzzRobot(hardwareMap, true);            // MANUAL bulk cache: cleared at the top of every loop
        settings = TeleOpSettings.load();
        trim = settings.aimTrimDeg;
        firePower = settings.queueFirePower;
        try {
            vision = new HiveVision(hardwareMap);
        } catch (Exception e) {
            vision = null;
        }
        rebuildQueue();
    }

    @Override
    public void init_loop() {
        robot.clearBulkCache();
        telemetry.addData("Alliance", "%s (from the last TeleOp / AUTO)", settings.alliance);
        telemetry.addData("Saved", "trim %+.1f deg, queue fire power %.2f", trim, firePower);
        telemetry.addData("Gate", BioBuzzRobot.gateCalibrated() ? "calibrated" : "NOT CALIBRATED - FIRE refused");
        telemetry.addData("Camera", vision == null ? "NOT FOUND" : "ok");
    }

    /** A new fire power (Tune's D-pad) applies from the next loop. */
    private void rebuildQueue() {
        settings.queueFirePower = firePower;
        if (queue == null) {
            queue = QueueFeeder.fromSettings(settings);
        }
        queue.setFirePower(firePower);
        if (vision != null) {
            vision.setAimTrimDeg(trim);
        }
    }

    private double speed() {
        return settings.hasSpot() ? settings.spotSpeed : LAUNCHER_NEAR_VELOCITY;
    }

    private void say(String s) {
        note = s;
        noteTimer.reset();
    }

    @Override
    public void loop() {
        robot.clearBulkCache();                                 // one bulk read per hub per loop
        if (vision != null) {
            vision.update();
        }
        HiveVision.Target target = vision == null ? null : vision.target(settings.alliance, null);

        // ---------------- Settings ----------------
        boolean changed = false;
        if (gamepad1.dpadLeftWasPressed()) {
            trim = Range.clip(trim + 0.5, -TeleOpSettings.AIM_TRIM_MAX_DEG, TeleOpSettings.AIM_TRIM_MAX_DEG);
            changed = true;
        } else if (gamepad1.dpadRightWasPressed()) {
            trim = Range.clip(trim - 0.5, -TeleOpSettings.AIM_TRIM_MAX_DEG, TeleOpSettings.AIM_TRIM_MAX_DEG);
            changed = true;
        } else if (gamepad1.dpadUpWasPressed()) {
            firePower = Range.clip(round2(firePower + 0.05), TeleOpSettings.QUEUE_FIRE_POWER_MIN,
                    TeleOpSettings.QUEUE_FIRE_POWER_MAX);
            changed = true;
        } else if (gamepad1.dpadDownWasPressed()) {
            firePower = Range.clip(round2(firePower - 0.05), TeleOpSettings.QUEUE_FIRE_POWER_MIN,
                    TeleOpSettings.QUEUE_FIRE_POWER_MAX);
            changed = true;
        }
        if (changed) {
            rebuildQueue();
        }
        if (gamepad1.aWasPressed()) {
            volleys = 0;
            beamBalls = 0;
            ballCounter.reset();
            jams = 0;
            say("Counters reset");
        }
        if (gamepad1.backWasPressed()) {
            TeleOpSettings saved = TeleOpSettings.load();          // keep alliance + SHOT SPOT as they are on the robot
            saved.aimTrimDeg = trim;
            saved.queueFirePower = firePower;
            boolean ok = saved.trySave();
            say(ok ? String.format("SAVED: trim %+.1f, queue fire power %.2f", trim, firePower) : "SAVE FAILED");
            gamepad1.rumble(ok ? 600 : 200);
        }

        // ---------------- Launcher ----------------
        if (gamepad1.yWasPressed()) {
            launcherOn = !launcherOn;
        }
        boolean fire = gamepad1.right_bumper;
        robot.setLauncherVelocity(launcherOn || fire ? speed() : 0);
        boolean ready = robot.updateLauncherReady();            // one fresh decision per loop

        // ---------------- Drive + aim ----------------
        double scale = gamepad1.left_bumper ? SLOW_DRIVE_SCALE : 1.0;
        double forward = -gamepad1.left_stick_y * scale;
        double strafe = gamepad1.left_stick_x * scale;
        double turn = gamepad1.right_stick_x * scale;
        boolean onTarget = false;
        if (gamepad1.x && target != null) {
            turn = HiveVision.aimTurnCw(target.bearingDeg);
            double toSpot = 0;
            // only to the CELL the spot was saved to (or its twin on the other alliance): not the far CELL behind it
            if (settings.spotIsFor(target.cell) && Math.abs(gamepad1.left_stick_y) < 0.1) {
                toSpot = MotionController.spotForward(target.horizontalIn(), settings.spotRangeIn);
                forward = toSpot;
            }
            onTarget = turn == 0 && toSpot == 0;
        }
        robot.drive(forward, strafe, turn, 1.0);

        // ---------------- Queue ----------------
        double intake = gamepad1.right_trigger - gamepad1.left_trigger;
        boolean intakeIn = intake > 0.05;
        boolean staged = robot.stagedBroken();
        boolean fullBeam = robot.fullBroken();
        if (gamepad1.b) {
            queue.reset();
            robot.setIndexerPower(-QUEUE_UNJAM_POWER);
        } else {
            robot.setIndexerPower(queue.update(clock.seconds(), fire, true, intakeIn, staged, fullBeam, ready,
                    robot.launcherDipTooDeep(), robot.indexerTicks()));
            String ev = queue.event();
            if (ev != null) {
                if (ev.contains("JAM ")) {
                    jams++;
                }
                if (ev.contains("GATE OPEN")) {
                    volleys++;
                }
                if (ev.contains("BALL ")) {
                    beamBalls++;
                }
                say(ev);
            }
        }
        robot.setGateOpen(queue.gateOpen());
        if (fire && queue.firing()) {
            intake = Math.max(intake, 0) + FEED_INTAKE_BOOST;
        }
        double t = clock.seconds();                             // G407 push-back + pair-jam unjam (IntakeGuard)
        double intakeAmps = intakeGuard.currentDue(t, intake) ? robot.intakeCurrentAmps() : Double.NaN;
        intake = intakeGuard.update(t, intake, queue.isFull(), queue.firing(), intakeAmps, staged, fullBeam);
        if (intakeGuard.event() != null) {
            say(intakeGuard.event());
        }
        robot.setIntake(intake);
        double vel = robot.launcherVelocity();
        ballCounter.update(clock.seconds(), queue.firing(), robot.isLauncherOn() ? speed() : 0, vel);

        // ---------------- Screen ----------------
        telemetry.addData("AIM TRIM", "%+.1f deg   (D-pad LEFT = shots more left, RIGHT = more right, seen from behind the launcher)", trim);
        telemetry.addData("QUEUE FIRE POWER", "%.2f   (D-pad UP / DOWN)", firePower);
        telemetry.addData("Balls", "beam %d, flywheel dips %d, volleys %d, jams %d   (A resets)", beamBalls,
                ballCounter.balls(), volleys, jams);
        telemetry.addData("Launcher", "%s %.0f / %.0f t/s %s  (launcher %.0f, launcher2 %.0f)",
                robot.isLauncherOn() ? "ON" : "OFF (Y)", vel, speed(), ready ? "READY" : "",
                robot.launcher1Velocity(), robot.launcher2Velocity());
        String warning = robot.launcherWarning();
        if (warning != null) {
            telemetry.addData("WARNING", warning);
        }
        if (target != null) {
            telemetry.addData("CELL", "%s %d tags  camera %.1f  aim %.1f deg  floor %.0f in%s", target.cell,
                    target.tagCount, target.rawBearingDeg, target.bearingDeg, target.horizontalIn(),
                    onTarget ? "  ** ON TARGET **" : "");
        } else {
            telemetry.addData("CELL", vision == null ? "camera missing" : "no " + settings.alliance + " CELL in view");
        }
        telemetry.addData("Queue", "%s%s  staged %s, full %s%s", gamepad1.b ? "UNJAM" : queue.state().toString(),
                queue.holding() ? " (HOLD)" : "", staged ? "BALL" : "clear", fullBeam ? "BALL" : "clear",
                BioBuzzRobot.gateCalibrated() ? "" : "   GATE NOT CALIBRATED");
        telemetry.addData("Intake", intakeGuard.status());
        if (noteTimer.seconds() < 4 && !note.isEmpty()) {
            telemetry.addData("Note", note);
        }
        telemetry.addData("Saved?", "SHARE saves trim and fire power");
    }

    private static double round2(double v) {
        return Math.round(v * 100) / 100.0;
    }

    @Override
    public void stop() {
        robot.stopAll();
        if (vision != null) {
            vision.close();
        }
    }
}

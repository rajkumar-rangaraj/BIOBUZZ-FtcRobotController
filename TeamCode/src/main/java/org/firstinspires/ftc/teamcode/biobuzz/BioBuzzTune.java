package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * BioBuzz Tune - tune AIM and WINDMILL FEED on the robot, no code change. SHARE saves; TeleOp and AUTO use the values.
 *
 *   Sticks, LB                 drive (LB = slow)
 *   Square / X (hold)          aim at our CELL (+ drive to the SHOT SPOT distance if one is saved)
 *   Triangle / Y               flywheel ON / OFF (SHOT SPOT speed, or NEAR)
 *   R1 / RB (hold)             FIRE with the values below          Triggers: intake in / out
 *   Circle / B (hold)          unjam (windmill backwards)
 *   D-pad LEFT / RIGHT         AIM TRIM: shots go more LEFT / RIGHT (0.5 deg per press)
 *   D-pad UP / DOWN            PAUSE between pockets +/- 0.1 s
 *   LB + D-pad UP / DOWN       WINDMILL POWER +/- 0.05
 *   Cross / A                  reset the ball counter
 *   SHARE / Back               SAVE trim, power and pause on the robot
 */
@TeleOp(name = "BioBuzz Tune", group = "BioBuzz Test")
public class BioBuzzTune extends OpMode {

    private BioBuzzRobot robot;
    private HiveVision vision;
    private TeleOpSettings settings;
    private ShotFeeder feeder;
    private double trim, power, pause;
    private boolean launcherOn = false;
    private int pockets = 0, balls = 0, jams = 0;
    private int feederShotsSeen = 0;
    private boolean dipping = false;
    private String note = "";
    private final ElapsedTime noteTimer = new ElapsedTime();
    private final ElapsedTime clock = new ElapsedTime();

    @Override
    public void init() {
        robot = new BioBuzzRobot(hardwareMap);
        settings = TeleOpSettings.load();
        trim = settings.aimTrimDeg;
        power = settings.feedPower;
        pause = settings.feedPauseSec;
        try {
            vision = new HiveVision(hardwareMap);
        } catch (Exception e) {
            vision = null;
        }
        rebuildFeeder();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Alliance", "%s (from the last TeleOp / AUTO)", settings.alliance);
        telemetry.addData("Saved", "trim %+.1f deg, power %.2f, pause %.2f s", trim, power, pause);
        telemetry.addData("Camera", vision == null ? "NOT FOUND" : "ok");
    }

    private void rebuildFeeder() {
        settings.feedPower = power;
        settings.feedPauseSec = pause;
        feeder = ShotFeeder.fromSettings(settings);
        feederShotsSeen = 0;
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
            if (gamepad1.left_bumper) {
                power = Range.clip(round2(power + 0.05), TeleOpSettings.FEED_POWER_MIN, TeleOpSettings.FEED_POWER_MAX);
            } else {
                pause = Range.clip(round2(pause + 0.1), TeleOpSettings.FEED_PAUSE_MIN, TeleOpSettings.FEED_PAUSE_MAX);
            }
            changed = true;
        } else if (gamepad1.dpadDownWasPressed()) {
            if (gamepad1.left_bumper) {
                power = Range.clip(round2(power - 0.05), TeleOpSettings.FEED_POWER_MIN, TeleOpSettings.FEED_POWER_MAX);
            } else {
                pause = Range.clip(round2(pause - 0.1), TeleOpSettings.FEED_PAUSE_MIN, TeleOpSettings.FEED_PAUSE_MAX);
            }
            changed = true;
        }
        if (changed) {
            rebuildFeeder();
        }
        if (gamepad1.aWasPressed()) {
            pockets = 0;
            balls = 0;
            jams = 0;
            say("Counter reset");
        }
        if (gamepad1.backWasPressed()) {
            TeleOpSettings saved = TeleOpSettings.load();          // keep alliance + SHOT SPOT as they are on the robot
            saved.aimTrimDeg = trim;
            saved.feedPower = power;
            saved.feedPauseSec = pause;
            boolean ok = saved.trySave();
            say(ok ? String.format("SAVED: trim %+.1f, power %.2f, pause %.2f", trim, power, pause) : "SAVE FAILED");
            gamepad1.rumble(ok ? 600 : 200);
        }

        // ---------------- Launcher ----------------
        if (gamepad1.yWasPressed()) {
            launcherOn = !launcherOn;
        }
        boolean fire = gamepad1.right_bumper;
        robot.setLauncherVelocity(launcherOn || fire ? speed() : 0);
        boolean ready = robot.launcherReady();

        // ---------------- Drive + aim ----------------
        double scale = gamepad1.left_bumper ? SLOW_DRIVE_SCALE : 1.0;
        double forward = -gamepad1.left_stick_y * scale;
        double strafe = gamepad1.left_stick_x * scale;
        double turn = gamepad1.right_stick_x * scale;
        boolean onTarget = false;
        if (gamepad1.x && target != null) {
            turn = HiveVision.aimTurnCw(target.bearingDeg);
            double toSpot = 0;
            if (settings.hasSpot() && Math.abs(gamepad1.left_stick_y) < 0.1) {
                toSpot = MotionController.spotForward(target.rangeIn, settings.spotRangeIn);
                forward = toSpot;
            }
            onTarget = turn == 0 && toSpot == 0;
        }
        robot.drive(forward, strafe, turn, 1.0);

        // ---------------- Windmill ----------------
        double intake = gamepad1.right_trigger - gamepad1.left_trigger;
        if (gamepad1.b) {
            feeder.reset();
            robot.setIndexerPower(-INDEXER_UNJAM_POWER);
        } else {
            robot.setIndexerPower(feeder.update(clock.seconds(), robot.indexerTicks(), ready, fire));
            String ev = feeder.event();
            if (ev != null && ev.startsWith("JAM ")) {
                jams++;
            }
        }
        if (feeder.shots() != feederShotsSeen) {
            pockets += feeder.shots() - feederShotsSeen;
            feederShotsSeen = feeder.shots();
        }
        if (fire) {
            intake = Math.max(intake, 0) + FEED_INTAKE_BOOST;
        }
        robot.setIntake(intake);
        double vel = robot.launcherVelocity();
        if (!dipping && robot.isLauncherOn() && vel < speed() - AUTO_SHOT_DIP_TPS) {
            dipping = true;
            balls++;
        } else if (dipping && vel > speed() - AUTO_SHOT_DIP_TPS / 2) {
            dipping = false;
        }

        // ---------------- Screen ----------------
        telemetry.addData("AIM TRIM", "%+.1f deg   (D-pad LEFT = shots more left, RIGHT = more right, seen from behind the launcher)", trim);
        telemetry.addData("PAUSE", "%.2f s between pockets   (D-pad UP / DOWN)", pause);
        telemetry.addData("WINDMILL", "power %.2f   (LB + D-pad UP / DOWN)", power);
        telemetry.addData("Balls / pockets", "%d / %d  = %s   jams %d   (A resets)", balls, pockets,
                pockets == 0 ? "-" : String.format("%.0f%%", 100.0 * balls / pockets), jams);
        telemetry.addData("Launcher", "%s %.0f / %.0f t/s %s", robot.isLauncherOn() ? "ON" : "OFF (Y)", vel, speed(),
                ready ? "READY" : "");
        if (target != null) {
            telemetry.addData("CELL", "%s %d tags  camera %.1f  aim %.1f deg  range %.0f in%s", target.cell,
                    target.tagCount, target.rawBearingDeg, target.bearingDeg, target.rangeIn,
                    onTarget ? "  ** ON TARGET **" : "");
        } else {
            telemetry.addData("CELL", vision == null ? "camera missing" : "no " + settings.alliance + " CELL in view");
        }
        telemetry.addData("Windmill", "%s", gamepad1.b ? "UNJAM" : feeder.state().toString());
        if (noteTimer.seconds() < 4 && !note.isEmpty()) {
            telemetry.addData("Note", note);
        }
        telemetry.addData("Saved?", "SHARE saves trim, power and pause");
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

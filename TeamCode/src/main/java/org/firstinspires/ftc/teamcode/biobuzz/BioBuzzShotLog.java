package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Map;
import java.util.TreeMap;

/*
 * Shot log - one ball at a time, every shot recorded with flywheel speed, battery, camera distance and result.
 * Use it to pick LAUNCHER_NEAR/FAR_VELOCITY and AUTO_LAUNCH_VELOCITY from data.
 * Log: /sdcard/BioBuzzLogs/shots_*.csv
 *
 *   Sticks, LB, X(hold aim)   driving as in TeleOp
 *   Triggers                  intake in / out (reload)
 *   Y / Triangle              launcher ON/OFF
 *   D-pad up/down             target speed +/- 50 t/s      D-pad right/left  +/- 10 t/s
 *   RB                        fire ONE windmill pocket (only when at speed)
 *   then A / Cross = IN the CELL,  B / Circle = MISS,  Back / Share = discard (no ball came out)
 */
@TeleOp(name = "BioBuzz Shot Log", group = "BioBuzz Test")
public class BioBuzzShotLog extends OpMode {
    private enum State { READY, FEEDING, RECOVERING, AWAIT_RESULT }

    private BioBuzzRobot robot;
    private HiveVision vision;
    private CsvLog log;
    private HiveVision.Alliance alliance = BioBuzzMatchState.alliance;

    private boolean launcherOn = false;
    private double target = LAUNCHER_NEAR_VELOCITY;
    private State state = State.READY;
    private final ElapsedTime shotTimer = new ElapsedTime();
    private int feedStartTicks;
    private int shotNumber = 0;
    private double velAtFeed, minVel, recoverySec, firedTarget;
    private boolean dipped;
    private HiveVision.Target shotTarget;
    private final Map<Integer, int[]> results = new TreeMap<>();

    @Override
    public void init() {
        robot = new BioBuzzRobot(hardwareMap);
        try {
            vision = new HiveVision(hardwareMap);
        } catch (Exception e) {
            vision = null;
        }
        log = new CsvLog("shots", "shot", "t", "target_tps", "vel_at_feed", "min_vel", "dip", "recovery_s",
                "battery_v", "cell", "tags", "bearing_deg", "range_in", "floor_dist_in", "result");
    }

    @Override
    public void loop() {
        if (vision != null) {
            vision.update();
        }
        HiveVision.Target cell = vision == null ? null : vision.target(alliance, null);

        if (gamepad1.yWasPressed()) {
            launcherOn = !launcherOn;
        }
        if (gamepad1.dpadUpWasPressed()) {
            target += 50;
        } else if (gamepad1.dpadDownWasPressed()) {
            target -= 50;
        } else if (gamepad1.dpadRightWasPressed()) {
            target += 10;
        } else if (gamepad1.dpadLeftWasPressed()) {
            target -= 10;
        }
        target = Range.clip(target, 0, LAUNCHER_MAX_VELOCITY);
        robot.setLauncherVelocity(launcherOn ? target : 0);
        double vel = robot.launcherVelocity();

        double scale = gamepad1.left_bumper ? SLOW_DRIVE_SCALE : 1.0;
        double turn = gamepad1.right_stick_x * scale;
        if (gamepad1.x && cell != null) {
            turn = HiveVision.aimTurnCw(cell.bearingDeg);
        }
        robot.drive(-gamepad1.left_stick_y * scale, gamepad1.left_stick_x * scale, turn, 1.0);
        robot.setIntake(gamepad1.right_trigger - gamepad1.left_trigger);

        switch (state) {
            case READY:
                robot.setIndexerPower(0);
                if (gamepad1.rightBumperWasPressed() && robot.launcherReady()) {
                    shotNumber++;
                    feedStartTicks = robot.indexerTicks();
                    firedTarget = target;
                    velAtFeed = vel;
                    minVel = vel;
                    dipped = false;
                    recoverySec = -1;
                    shotTarget = cell;
                    shotTimer.reset();
                    state = State.FEEDING;
                }
                break;
            case FEEDING:
                robot.setIndexerPower(INDEXER_FEED_POWER);
                if (Math.abs(robot.indexerTicks() - feedStartTicks) >= INDEXER_TICKS_PER_POCKET
                        || shotTimer.seconds() > 1.5) {
                    robot.setIndexerPower(0);
                    state = State.RECOVERING;
                }
                trackDip(vel);
                break;
            case RECOVERING:
                robot.setIndexerPower(0);
                trackDip(vel);
                if (shotTimer.seconds() > 2.0) {
                    state = State.AWAIT_RESULT;
                }
                break;
            case AWAIT_RESULT:
                robot.setIndexerPower(0);
                if (gamepad1.aWasPressed()) {
                    record("IN");
                } else if (gamepad1.bWasPressed()) {
                    record("MISS");
                } else if (gamepad1.backWasPressed()) {
                    record("DISCARD");
                }
                break;
        }

        telemetry.addData("Launcher", "%s  target %.0f  actual %.0f t/s %s", launcherOn ? "ON" : "OFF (Y)",
                target, vel, robot.launcherReady() ? "READY" : "");
        telemetry.addData("Shot #" + shotNumber, state == State.AWAIT_RESULT
                ? "A = IN, B = MISS, Back = discard" : state.toString());
        telemetry.addData("Last dip", "%.0f t/s, recovery %.2f s", velAtFeed - minVel, recoverySec);
        telemetry.addData("CELL", cell == null ? "not in view" : String.format("%d tags, bearing %.1f, floor %.0f in",
                cell.tagCount, cell.bearingDeg, cell.horizontalIn()));
        for (Map.Entry<Integer, int[]> e : results.entrySet()) {
            int[] r = e.getValue();
            telemetry.addData("  " + e.getKey() + " t/s", "%d / %d in (%.0f%%)", r[0], r[1], 100.0 * r[0] / r[1]);
        }
        telemetry.addData("Battery", "%.2f V", robot.battery.getVoltage());
        telemetry.addData("Log", log.status());
    }

    private void trackDip(double vel) {
        minVel = Math.min(minVel, vel);
        double readyAt = firedTarget - LAUNCHER_READY_MARGIN;
        if (vel < readyAt) {
            dipped = true;
        } else if (dipped && recoverySec < 0) {
            recoverySec = shotTimer.seconds();
        }
    }

    private void record(String result) {
        log.row(shotNumber, getRuntime(), firedTarget, velAtFeed, minVel, velAtFeed - minVel, recoverySec,
                robot.battery.getVoltage(),
                shotTarget == null ? "" : shotTarget.cell, shotTarget == null ? 0 : shotTarget.tagCount,
                shotTarget == null ? "" : shotTarget.bearingDeg, shotTarget == null ? "" : shotTarget.rangeIn,
                shotTarget == null ? "" : shotTarget.horizontalIn(), result);
        if (!result.equals("DISCARD")) {
            int[] r = results.get((int) firedTarget);
            if (r == null) {
                r = new int[2];
                results.put((int) firedTarget, r);
            }
            r[1]++;
            if (result.equals("IN")) {
                r[0]++;
            }
        }
        state = State.READY;
    }

    @Override
    public void stop() {
        robot.stopAll();
        if (vision != null) {
            vision.close();
        }
        log.close();
    }
}

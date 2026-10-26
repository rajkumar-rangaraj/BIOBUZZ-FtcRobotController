package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * BIOBUZZ AUTO (TU06): LEAVE (3) + launch 4 preloads into our upward CELL (HIVE TIP = 20 if it tips)
 * + AUTO PARK in our LOADING ZONE (5).  LEAVE + PARK from both robots reaches the 16-point SWARM threshold.
 *
 * START (same for RED and BLUE - the field is point-symmetric):
 *   - intake side touching our ALLIANCE wall, launcher pointing into the FIELD,
 *   - robot completely outside the LOADING ZONE, next to it, about 1 in from its tape
 *     (RED: audience side of the zone, BLUE: rear side of the zone),
 *   - not touching a FLOWER, 4 POLLEN in the windmill, flower arm stowed.
 * ROUTE (AutoRoute): back 12 in (LEAVE) -> turn to aim -> camera fine-aim -> fire -> face the wall ->
 *        forward to 4 in off the wall -> strafe right into the LOADING ZONE.
 *
 * INIT options on gamepad 1: D-pad up/down = start delay, X = shooting on/off.
 */
public abstract class BioBuzzAutoBase extends LinearOpMode implements DriveIO, AutoRoute.Mechanisms {

    protected abstract HiveVision.Alliance alliance();

    private BioBuzzRobot robot;
    private HiveVision vision;
    private final ElapsedTime clock = new ElapsedTime();
    private double matchStart = 0;
    private String step = "INIT";

    @Override
    public void runOpMode() {
        robot = new BioBuzzRobot(hardwareMap);
        String visionError = null;
        try {
            vision = new HiveVision(hardwareMap);
        } catch (Exception e) {
            vision = null;
            visionError = e.getMessage();
        }
        BioBuzzMatchState.alliance = alliance();

        int startDelaySec = 0;
        boolean shoot = true;
        while (opModeInInit()) {
            if (gamepad1.dpadUpWasPressed()) {
                startDelaySec = Math.min(AUTO_MAX_START_DELAY_SEC, startDelaySec + 1);
            } else if (gamepad1.dpadDownWasPressed()) {
                startDelaySec = Math.max(0, startDelaySec - 1);
            }
            if (gamepad1.xWasPressed()) {
                shoot = !shoot;
            }
            if (vision != null) {
                vision.update();
            }
            telemetry.addData("Alliance", alliance());
            telemetry.addData("Plan", shoot ? "LEAVE + SHOOT 4 + PARK" : "LEAVE + PARK only");
            telemetry.addData("Start delay", "%d s   (D-pad up/down)", startDelaySec);
            telemetry.addData("Shooting", "%s   (X toggles)", shoot ? "ON" : "OFF");
            telemetry.addData("Heading", "%.1f deg", robot.headingDeg());
            if (vision == null) {
                telemetry.addData("Camera", "NOT FOUND: %s (IMU aim only)", visionError);
            } else {
                HiveVision.Target t = vision.target(alliance(), HiveVision.startingUpCell(alliance()));
                telemetry.addData("Camera", t == null ? "no CELL tags in view (normal at the wall)"
                        : String.format("%s %d tags, bearing %.1f", t.cell, t.tagCount, t.bearingDeg));
            }
            telemetry.addData("Battery", "%.2f V", robot.battery.getVoltage());
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) {
            closeVision();
            return;
        }

        robot.imu.resetYaw();
        matchStart = clock.seconds();
        try {
            AutoRoute.run(this, this, startDelaySec, shoot);
            loopTick();
        } finally {
            robot.stopAll();
            closeVision();
        }
    }

    // ---------------- DriveIO ----------------

    @Override
    public boolean active() {
        return opModeIsActive();
    }

    @Override
    public double seconds() {
        return clock.seconds();
    }

    @Override
    public double forwardInches() {
        return robot.forwardInches();
    }

    @Override
    public double strafeRightInches() {
        return robot.strafeRightInches();
    }

    @Override
    public double headingDeg() {
        return robot.headingDeg();
    }

    @Override
    public void drive(double forward, double strafeRight, double turnCw) {
        robot.drive(forward, strafeRight, turnCw, 1.0);
    }

    @Override
    public void loopTick() {
        telemetry.addData("Step", step);
        telemetry.addData("Time", "%.1f s", clock.seconds() - matchStart);
        telemetry.addData("Heading", "%.1f deg", robot.headingDeg());
        telemetry.addData("Moved", "fwd %.1f  right %.1f in", robot.forwardInches(), robot.strafeRightInches());
        telemetry.addData("Launcher", "%.0f / %.0f t/s", robot.launcherVelocity(), robot.launcherTarget());
        telemetry.update();
    }

    // ---------------- AutoRoute.Mechanisms ----------------

    @Override
    public void step(String name) {
        step = name;
    }

    @Override
    public void startLauncher() {
        robot.setLauncherVelocity(AUTO_LAUNCH_VELOCITY);
    }

    @Override
    public void stopLauncher() {
        robot.setLauncherVelocity(0);
    }

    @Override
    public void aimAndFire(double matchDeadlineSec) {
        aimWithCamera(Math.min(clock.seconds() + AUTO_AIM_TIMEOUT_SEC, matchDeadlineSec));
        fire(matchDeadlineSec);
    }

    private void aimWithCamera(double endSec) {
        if (vision == null) {
            return;
        }
        HiveVision.Cell preferred = HiveVision.startingUpCell(alliance());
        int settled = 0;
        while (opModeIsActive() && clock.seconds() < endSec) {
            vision.update();
            HiveVision.Target target = vision.target(alliance(), preferred);
            double turn = target == null ? 0 : HiveVision.aimTurnCw(target.bearingDeg);
            robot.drive(0, 0, turn, 1.0);
            if (target != null && turn == 0) {
                if (++settled >= 3) {
                    break;
                }
            } else {
                settled = 0;
            }
            telemetry.addData("Target", target == null ? "none" : String.format("%s bearing %.1f range %.0f",
                    target.cell, target.bearingDeg, target.rangeIn));
            loopTick();
        }
        robot.stopDrive();
    }

    /** Feed the windmill only while the flywheel is at speed, until the preloads (plus one pocket) have passed. */
    private void fire(double matchDeadlineSec) {
        int start = robot.indexerTicks();
        double needed = AUTO_FEED_POCKETS * INDEXER_TICKS_PER_POCKET;
        double end = Math.min(clock.seconds() + AUTO_FEED_TIMEOUT_SEC, matchDeadlineSec);
        while (opModeIsActive() && clock.seconds() < end && Math.abs(robot.indexerTicks() - start) < needed) {
            robot.setIndexerPower(robot.launcherReady() ? INDEXER_FEED_POWER : 0);
            telemetry.addData("Windmill", "%d / %.0f ticks", Math.abs(robot.indexerTicks() - start), needed);
            loopTick();
        }
        robot.setIndexerPower(0);
    }

    private void closeVision() {
        if (vision != null) {
            vision.close();
            vision = null;
        }
    }
}

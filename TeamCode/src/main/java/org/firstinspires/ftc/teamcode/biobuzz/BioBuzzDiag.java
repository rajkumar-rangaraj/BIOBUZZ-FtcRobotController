package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Diagnostics log for real-robot runs. Two files in /sdcard/FIRST/BioBuzzLogs/ :
 *   diag_&lt;opmode&gt;_&lt;date&gt;.csv   - 20 rows/s: sticks + buttons, every motor command, encoders, flywheel, arm, camera, loop time
 *   events_&lt;opmode&gt;_&lt;date&gt;.csv - one row per event: button actions, state changes, warnings
 * Copy them to the laptop with tools/pull_robot_logs.ps1. When disabled, every call does nothing.
 */
public class BioBuzzDiag {
    private final boolean enabled;
    private final CsvLog samples;
    private final CsvLog events;
    private final ElapsedTime clock = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastSampleMs = -1e9;
    private double lastSlowMs = -1e9;
    private double maxLoopMs = 0;
    private double battery = Double.NaN;
    private double heading = Double.NaN;
    private int warningsLogged = 0;
    private String lastWarning = null;

    public BioBuzzDiag(String opMode, boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            samples = null;
            events = null;
            return;
        }
        samples = new CsvLog("diag_" + opMode, "t", "loop_ms", "max_loop_ms", "battery_v", "heading_deg",
                "stick_lx", "stick_ly", "stick_rx", "trig_l", "trig_r", "buttons",
                "pw_lf", "pw_rf", "pw_lb", "pw_rb", "enc_lf", "enc_rf", "enc_lb", "enc_rb",
                "fwd_in", "right_in", "intake_pw", "indexer_pw", "indexer_ticks",
                "launcher_on", "launcher_target", "launcher_vel", "launcher_ready",
                "arm_state", "arm_cmd", "aim_cell", "aim_tags", "aim_bearing", "aim_range_in", "note");
        events = new CsvLog("events_" + opMode, "t", "event", "detail");
        event("START", opMode);
    }

    public boolean enabled() {
        return enabled;
    }

    public String status() {
        return enabled ? samples.status() : "OFF";
    }

    /** Call once per loop; writes a row every DIAG_SAMPLE_MS. target may be null. */
    public void sample(BioBuzzRobot r, Gamepad g, HiveVision.Target target, String note) {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        if (!enabled) {
            return;
        }
        maxLoopMs = Math.max(maxLoopMs, loopMs);
        double now = clock.milliseconds();
        if (now - lastSlowMs >= DIAG_SLOW_SAMPLE_MS) {           // each costs a separate hub read: keep them slow
            battery = r.battery.getVoltage();
            heading = r.headingDeg();
            lastSlowMs = now;
        }
        if (now - lastSampleMs < DIAG_SAMPLE_MS) {
            return;
        }
        lastSampleMs = now;
        samples.row(now / 1000.0, loopMs, maxLoopMs, battery, heading,
                (double) g.left_stick_x, (double) g.left_stick_y, (double) g.right_stick_x,
                (double) g.left_trigger, (double) g.right_trigger, buttons(g),
                r.lastDrivePower[0], r.lastDrivePower[1], r.lastDrivePower[2], r.lastDrivePower[3],
                r.leftFront.getCurrentPosition(), r.rightFront.getCurrentPosition(),
                r.leftBack.getCurrentPosition(), r.rightBack.getCurrentPosition(),
                r.forwardInches(), r.strafeRightInches(), r.lastIntakePower, r.lastIndexerPower, r.indexerTicks(),
                r.isLauncherOn(), r.launcherTarget(), r.launcherVelocity(), r.launcherReady(),
                r.armState(), r.flowerArm.getPosition(),
                target == null ? "" : target.cell, target == null ? 0 : target.tagCount,
                target == null ? "" : target.bearingDeg, target == null ? "" : target.rangeIn,
                note == null ? "" : note);
        maxLoopMs = 0;
        String w = r.launcherWarning();
        if (w != null && !w.equals(lastWarning) && warningsLogged < 50) {
            event("WARNING", w);
            warningsLogged++;
        }
        lastWarning = w;
    }

    public void event(String name, String detail) {
        if (enabled) {
            events.row(clock.seconds(), name, detail == null ? "" : detail);
        }
    }

    public void close() {
        if (!enabled) {
            return;
        }
        event("STOP", String.format("%.1f s", clock.seconds()));
        samples.close();
        events.close();
    }

    private static String buttons(Gamepad g) {
        StringBuilder sb = new StringBuilder();
        if (g.a) sb.append("A ");
        if (g.b) sb.append("B ");
        if (g.x) sb.append("X ");
        if (g.y) sb.append("Y ");
        if (g.left_bumper) sb.append("LB ");
        if (g.right_bumper) sb.append("RB ");
        if (g.dpad_up) sb.append("DU ");
        if (g.dpad_down) sb.append("DD ");
        if (g.dpad_left) sb.append("DL ");
        if (g.dpad_right) sb.append("DR ");
        if (g.back) sb.append("BACK ");
        return sb.toString().trim();
    }
}

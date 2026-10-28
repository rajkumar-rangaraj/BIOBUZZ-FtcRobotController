package org.firstinspires.ftc.teamcode.biobuzz;

/** Values handed from the AUTO OpMode to the TeleOp OpMode (both run in the same app process). */
public final class BioBuzzMatchState {
    private BioBuzzMatchState() {
    }

    public static HiveVision.Alliance alliance = HiveVision.Alliance.RED;

    /** Diagnostics log on/off. Toggled in INIT (Back / Share) and kept until the robot app restarts. */
    public static boolean diagnostics = BioBuzzConstants.DIAG_DEFAULT_ON;
}

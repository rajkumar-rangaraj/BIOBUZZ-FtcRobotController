package org.firstinspires.ftc.teamcode.biobuzz;

/** Values handed from the AUTO OpMode to the TeleOp OpMode (both run in the same app process). */
public final class BioBuzzMatchState {
    private BioBuzzMatchState() {
    }

    public static HiveVision.Alliance alliance = TeleOpSettings.load().alliance;

    /** Sets the alliance and saves it on the hub, so it survives app restarts and new installs. */
    public static void rememberAlliance(HiveVision.Alliance a) {
        alliance = a;
        TeleOpSettings s = TeleOpSettings.load();
        if (s.alliance != a) {
            s.alliance = a;
            s.trySave();
        }
    }

    /** Diagnostics log on/off. Toggled in INIT (Back / Share) and kept until the robot app restarts. */
    public static boolean diagnostics = BioBuzzConstants.DIAG_DEFAULT_ON;
}

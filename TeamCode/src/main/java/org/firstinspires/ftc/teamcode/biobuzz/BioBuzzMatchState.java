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

    /** TeleOp field-centric driving on/off. Toggled in TeleOp INIT (LB / L1) and kept until the robot app restarts. */
    public static boolean fieldCentric = BioBuzzConstants.FIELD_CENTRIC_DEFAULT;

    private static double autoHeadingDeg = Double.NaN;
    private static long autoHeadingNanos = 0;

    /**
     * AUTO, every loop: the robot's heading relative to its DRIVERS (0 = intake away from them, counter-clockwise +).
     * TeleOp INIT takes it when AUTO ran less than AUTO_HEADING_HANDOFF_SEC ago, so field-centric driving starts right
     * without a reset (the robot doesn't move between AUTO and TeleOp in a match).
     */
    public static void recordAutoHeading(double driverHeadingDeg) {
        autoHeadingDeg = driverHeadingDeg;
        autoHeadingNanos = System.nanoTime();
    }

    /**
     * The heading AUTO left the robot at (NaN when AUTO didn't run recently). TeleOp INIT reads it; TeleOp START clears
     * it (clearAutoHeading), so INIT-ing TeleOp twice before START still gets it (review, 10 Nov).
     */
    public static double autoHeading() {
        double age = (System.nanoTime() - autoHeadingNanos) / 1e9;
        return !Double.isNaN(autoHeadingDeg) && age >= 0 && age <= BioBuzzConstants.AUTO_HEADING_HANDOFF_SEC
                ? autoHeadingDeg : Double.NaN;
    }

    public static void clearAutoHeading() {
        autoHeadingDeg = Double.NaN;
    }
}

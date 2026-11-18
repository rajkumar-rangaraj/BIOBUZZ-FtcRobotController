package org.firstinspires.ftc.teamcode.biobuzz.pp;

/**
 * Tuned numbers for the Pinpoint TeleOp ("BioBuzz TeleOp PP"). Everything shared with the camera TeleOp (flywheel
 * table, presets, windmill, aim trim) stays in BioBuzzConstants, so both TeleOps shoot the same.
 *
 * Frame: the ALLIANCE frame = AutoField's RED frame (origin field centre, +X toward the BLUE wall, +Y to the left of the
 * RED drivers, heading CCW from +X, 0 = intake away from our drivers). The field is point-symmetric, so a BLUE robot
 * uses the same numbers: its alliance frame is the real field turned 180 deg.
 */
public final class PpConstants {
    private PpConstants() {
    }

    // ---------------- Robot geometry (CALIBRATE: tape measure) ----------------
    /** Camera lens behind the robot centre (the camera and launcher are on the back). Estimated 7-9 in from AUTO logs. */
    public static final double CAMERA_BACK_IN = 8.0;
    /** Camera lens left of the robot centre line. */
    public static final double CAMERA_LEFT_IN = 0.0;

    // ---------------- Our two CELL openings, alliance frame (CALIBRATE) ----------------
    // From the Pedro AUTO logs (pose + camera, 13-14 Nov, RED and BLUE mirrored): the CELL in front of the audience-wall
    // shot spot at (-12.5, -17.0 +/- 1), the far one at about (-12.5, +16.5) (few samples, seen from 67 in).
    public static final double CELL_A_X = -12.5;
    public static final double CELL_A_Y = -17.0;
    public static final double CELL_B_X = -12.5;
    public static final double CELL_B_Y = 16.5;
    /** CellMap's UP / DOWN band (the one UP / DOWN decision): UP at or above, DOWN below; in between a place keeps what
     *  it was (misread heights). AUTO uses a single 45. */
    public static final double CELL_UP_HEIGHT_IN = 47;
    public static final double CELL_DOWN_HEIGHT_IN = 43;
    /** The target moves to the other CELL after this many UP sightings of it in a row ... */
    public static final int TARGET_SWITCH_SIGHTINGS = 2;
    /** ... while the current target hasn't been seen UP for this long. */
    public static final double TARGET_HOLD_SEC = 0.5;
    /** CELL opening height (camera estimate when UP; Pedro AUTO logs 13-14 Nov: 51.4 in). For the aim trim's tilt. */
    public static final double CELL_OPENING_HEIGHT_IN = 51.4;
    /**
     * Which CELL names sit at place A. Practice HIVE (13-14 Nov AUTO logs): RED SCORING (RED_REAR) and BLUE AUDIENCE were
     * at A, the opposite of the manual's figure. TeleOp PP learns it on its own once the position is known (CellMap),
     * this is only the first guess.
     */
    public static final boolean PRACTICE_CELL_NAMES = true;

    // ---------------- Position fixes from the camera (TagFix) ----------------
    public static final double FIX_MAX_FLOOR_IN = 72;          // farther sightings are too noisy
    public static final double FIX_MAX_TURN_DPS = 90;          // turning faster: the frame time matters too much
    public static final double FIX_MAX_SPEED_IPS = 45;
    public static final double FIX_GAIN_MULTI_TAG = 0.35;      // share of the error removed per frame (2+ tags)
    public static final double FIX_GAIN_ONE_TAG = 0.15;
    public static final double FIX_REJECT_IN = 12;             // a fix this far off the pose is an outlier ...
    public static final int FIX_SNAP_AFTER = 4;                // ... unless this many in a row agree: then the pose snaps
    public static final double FIX_AGREE_IN = 4;               // (agree = within this of each other)
    public static final int FIX_SWAP_CONFIRM = 2;              // CAMERA start: our other CELL seen where swapped names
                                                               // put it this many times = the names were swapped
    public static final double FIX_DEFAULT_LATENCY_SEC = 0.06; // frame age when the SDK gives no capture time
    public static final double FIX_MAX_FRAME_AGE_SEC = 0.25;

    // ---------------- Pose (FieldPose) ----------------
    public static final double POSE_MAX_SPEED_IPS = 90;        // the robot can't move faster (312 rpm 104 mm ~ 67 in/s)
    public static final double POSE_JUMP_MARGIN_IN = 4;        // a raw move bigger than speed x dt + this = Pinpoint glitch

    // ---------------- Aim (HeadingLock, fire gate) ----------------
    public static final double LOCK_KP = 0.015;                // turn power per degree (as AIM_KP)
    public static final double LOCK_KD = 0.0012;               // turn power per deg/s of turn rate (damping)
    public static final double LOCK_MIN_TURN = 0.10;           // as AIM_MIN_TURN: less stalls
    public static final double LOCK_MAX_TURN = 0.45;           // more than AIM_MAX_TURN: hold the aim against a push
    public static final double LOCK_DEADBAND_DEG = 0.6;
    public static final double FIRE_TOLERANCE_DEG = 1.5;       // horizontal aim error at which a ball may go
    public static final double FIRE_MAX_TURN_DPS = 25;         // and turning slower than this
    public static final double FIRE_GIVEWAY_SEC = 2.0;         // a ready ball held this long goes anyway, but only
    public static final double FIRE_GIVEWAY_MAX_ERROR_DEG = 5; // this close, not while the driver turns (a SHOT SPOT
                                                               // that can't be reached is dropped instead)
    public static final double STICK_OVERRIDE = 0.15;          // right stick past this = the driver turns

    // ---------------- Flywheel settle (between volley balls) ----------------
    // 7-14 Nov: balls fed while FIRE was held left +30 t/s over the target (first balls: +0): the flywheel overshoots
    // while it recovers from the dip, inside LauncherGate's +/-30. The next ball waits until the speed is no more than
    // SETTLE_HIGH over and SETTLE_LOW under the target and not rising (the velocity reads in 20 t/s steps, so one step
    // up is allowed). A tired battery sits ~16 t/s low: after SETTLE_FALLBACK_SEC near the target, up to
    // SETTLE_FALLBACK_LOW under is fine too. Too fast is never let through (it is the LONG ball).
    public static final double SETTLE_HIGH_TPS = 15;
    public static final double SETTLE_LOW_TPS = 25;
    public static final double SETTLE_MAX_RISE_TPS = 20;       // per reading
    public static final double SETTLE_FALLBACK_LOW_TPS = 40;
    public static final double SETTLE_FALLBACK_SEC = 0.3;

    // ---------------- Volley (one RB press fires everything) ----------------
    public static final int VOLLEY_EMPTY_POCKETS = 5;          // this many pockets with no ball launched = empty
                                                               // (7 Nov: ~2.7 pockets per ball, 4 pockets a turn)
    public static final double VOLLEY_MAX_SEC = 8;

    // ---------------- Pre-spin, telemetry, logging ----------------
    public static final double PRESPIN_MAX_DIST_IN = 80;       // auto pre-spin while this close to our UP CELL
    public static final double TELEMETRY_PERIOD_SEC = 0.25;
    public static final double POSE_LOG_PERIOD_SEC = 0.5;
}

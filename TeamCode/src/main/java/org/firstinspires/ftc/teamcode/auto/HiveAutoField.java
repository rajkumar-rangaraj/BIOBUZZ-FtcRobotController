package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoField.*;

import org.firstinspires.ftc.teamcode.auto.AutoField.Pose;

/**
 * Places for the HIVE AUTO (RED frame, see {@link AutoField}; BLUE = RED turned 180 deg about the centre). Measured from
 * the official field CAD (field-cad-step.step, sliced 12 Nov 2026) and the Competition Manual TU09. No SDK dependency.
 *
 * HIVE structure (manual 9.6, CAD am-5854 / am-5853):
 * - The frame is two A-shaped legs, one at each side (x +-24.7), anchored under the TILES; the space between them is
 *   open. The legs LEAN IN: a RED leg's inner edge is at x = -23.37 + 0.273 h (h = height above the TILES), at
 *   |y| = 18.26..19.35 - 0.446 h. Floor parts (feet, foot bars): x -24.75..-22.75, |y| <= 19.5, 2.15 in tall.
 * - The HIVEs hang from pivots 43.95 in up. The lowest point of the RED HIVE (a CELL tipped down) is 30.65 in above the
 *   TILES; our robot is 14.25 in tall (PPE7 cover), so it drives UNDER the HIVE between the legs. Robots driving under
 *   the HIVE are allowed (manual G409 example D).
 * - Each up CELL starts with 3 NECTAR; when the HIVE TIPS, the CELL that was up spills them toward its end of the field.
 *
 * Robot (Fusion model, 12 Nov): full width up to 8.0 in tall (wheels 4.3 in at the very edge, rail end caps 8.0 in);
 * only the middle +-5.7 in is taller (camera cradle 12.8 in, hubs 13.8 in, PPE7 cover 14.25 in).
 */
public final class HiveAutoField {
    private HiveAutoField() {
    }

    // ---------------- HIVE frame (RED side; the BLUE side is its mirror) ----------------

    /** Inner edge of a RED A-frame leg at height h: x = LEG_X0 + LEG_LEAN * h. */
    public static final double LEG_X0 = -23.37;
    public static final double LEG_LEAN = 0.2733;
    /** A leg's |y| at height h: LEG_Y_LO0 - LEG_Y_SLOPE * h .. LEG_Y_HI0 - LEG_Y_SLOPE * h. */
    public static final double LEG_Y_LO0 = 18.26;
    public static final double LEG_Y_HI0 = 19.35;
    public static final double LEG_Y_SLOPE = 0.4455;
    /** Frame feet and foot bars on the TILES (2.15 in tall). */
    public static final double FRAME_FLOOR_X_MIN = -24.75;
    public static final double FRAME_FLOOR_X_MAX = -22.75;
    public static final double FRAME_HALF_Y = 19.5;
    /** Lowest point of the HIVE (a CELL tipped down). */
    public static final double HIVE_LOWEST_IN = 30.65;

    // ---------------- Robot body (two stacked boxes) ----------------

    /** The whole footprint is at most this tall (rail end caps; the wheels at the very edge are 4.3 in). */
    public static final double ROBOT_LOW_HEIGHT_IN = 8.01;
    /** Above ROBOT_LOW_HEIGHT_IN only this middle strip (+- from the centre line) is taller. */
    public static final double ROBOT_UPPER_HALF_WIDTH_IN = 5.7;
    public static final double ROBOT_TOP_IN = 14.25;

    /** Leg box the low body must stay out of (heights 0..ROBOT_LOW_HEIGHT_IN), RED side. */
    public static final double LEG_LOW_X_MAX = LEG_X0 + LEG_LEAN * ROBOT_LOW_HEIGHT_IN;
    public static final double LEG_LOW_Y_MIN = LEG_Y_LO0 - LEG_Y_SLOPE * ROBOT_LOW_HEIGHT_IN;
    /** Leg box the upper body must stay out of (heights ROBOT_LOW_HEIGHT_IN..ROBOT_TOP_IN). */
    public static final double LEG_HIGH_X_MAX = LEG_X0 + LEG_LEAN * ROBOT_TOP_IN;
    public static final double LEG_HIGH_Y_MIN = LEG_Y_LO0 - LEG_Y_SLOPE * ROBOT_TOP_IN;
    public static final double LEG_HIGH_Y_MAX = LEG_Y_HI0 - LEG_Y_SLOPE * ROBOT_LOW_HEIGHT_IN;

    // ---------------- Route under the HIVE ----------------

    /**
     * Lane under our HIVE: robot centre line. Our side's legs need x >= ~-11.7 (2 in clear of the low body at the end
     * caps' height, 2.8 in at the wheels); our side ends at x 0 (G402). -10.4 leaves 1.8 in to the legs (CAD, low body
     * model) and 0.9 in to the middle line.
     */
    public static final double HIVE_LANE_X = -10.4;
    /** Robot centre this far from the middle (y 0) = fully out of the frame's legs (19.5 + 9 + 3.5). */
    public static final double HIVE_CLEAR_Y = 32.0;
    /** Heading under the HIVE: intake toward the REAR wall (launcher + camera look back at the HIVE). */
    public static final double FACE_REAR = 90;

    // ---------------- Far CELL (the one that comes UP after the first TIP) ----------------

    /** Centre of a CELL opening when UP, mirrored from the AUDIENCE CELL (CAD y -1.6..-20.1) about the pivot (y 0). */
    public static final double FAR_CELL_X = CELL_X;
    public static final double FAR_CELL_Y = 10.85;
    /**
     * Far shot spot, in the HIVE lane, facing straight back at the far CELL (exactly the lane heading: turning even 3 deg
     * on the way puts a corner within 1.5 in of a leg). The launcher's line passes 2.3 in from the CELL centre (opening
     * 20 in wide). The mirror of the near shot spot (y 59.5, x -12.7) would hit FLOWER 1 (rear wall, x -26.4..-20.4,
     * y 65.6..70.6): here the front-left corner stays 1.3 in off the planner's FLOWER circle (2 in off the CAD flower).
     * About 3 in closer to the CELL than the near shot.
     */
    public static final double FAR_SHOT_Y = 56.5;
    public static final Pose FAR_SHOT = new Pose(HIVE_LANE_X, FAR_SHOT_Y, FACE_REAR);
    /**
     * Camera floor distance to the far CELL at the far shot spot when the camera can't see it (for the shot speed):
     * the near shot spot logged 38.3-39.4 in (9 Nov), the far spot is ~3 in closer.
     */
    public static final double FAR_SHOT_PLANNED_FLOOR_IN = 36.0;

    // ---------------- FLOWER 1 (rear wall, our side): step 3.5 ----------------

    /** FLOWER 1 centre (CAD) and the front face of its lower ring (the retrieval side, toward the field). */
    public static final double FLOWER_X = -23.4;
    public static final double FLOWER_RING_FRONT_Y = 65.65;
    /**
     * Robot centre when the intake's upper Gecko row touches the FLOWER's middle-ring lip (robot_reach_probe, 18 Oct:
     * 208.65 mm in front of the ring face): the "stock stop" the flower test kit uses. CALIBRATE on a real FLOWER.
     */
    public static final double FLOWER_STOP_Y = FLOWER_RING_FRONT_Y - 208.65 / 25.4;
    /** Where the arm goes down: intake ~6 in short of the ring. */
    public static final double FLOWER_APPROACH_Y = FLOWER_STOP_Y - 6.0;
    public static final Pose FLOWER_APPROACH = new Pose(FLOWER_X, FLOWER_APPROACH_Y, FACE_REAR);
    public static final Pose FLOWER_STOP = new Pose(FLOWER_X, FLOWER_STOP_Y, FACE_REAR);

    /** The near CELL's spot, for reference (AutoField.SHOT). */
    public static final Pose NEAR_SHOT = SHOT;
}

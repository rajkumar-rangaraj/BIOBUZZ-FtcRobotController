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
 * Robot (Fusion model, 12-13 Nov): full width up to 8.0 in tall (wheels 4.3 in at the very edge, rail end caps 8.0 in);
 * only the middle +-5.7 in is taller (camera cradle 12.8 in, hubs 13.8 in, PPE7 cover 14.25 in). Not centred on Pedro's
 * pose: see ROBOT_FRONT_IN / START_OFFSET_IN.
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
     * Lane under our HIVE: robot centre line. -10.4: the low body 1.8 in from the legs (model; 2.8 in at wheel height),
     * our side's line (x 0, G402) 1.4 in; at the far spot the as-built robot can turn 4 deg from FAR_AIM_HEADING toward
     * the middle line and 7 deg the other way and stay 0.4 in clear of it, the REAR wall and FLOWER 1 (HivePlannerTest).
     */
    public static final double HIVE_LANE_X = -10.4;
    /** Robot centre this far from the middle (y 0) = fully out of the frame's legs (19.5 + 9 + 3.5). */
    public static final double HIVE_CLEAR_Y = 32.0;
    /** Heading under the HIVE: intake toward the REAR wall (launcher + camera look back at the HIVE). */
    public static final double FACE_REAR = 90;

    // ---------------- Robot as built (CAD, 13 Nov) vs the planner's 18 x 18 in box ----------------

    /**
     * Pedro's pose = the drivetrain centre (mecanum wheel centres at CAD y -144 / +120 mm: 12 mm ahead of the CAD
     * origin). From there the intake front is 9.76 in, the launcher back 8.03 in, the sides 8.9 in, the camera lens
     * 4.63 in back on the centre line (12.2 in high, tilted up 30 deg). The planner uses 9.0 / 9.0, so with the S1 start
     * (intake against the audience wall) the real robot is START_OFFSET_IN further toward the REAR wall than Pedro
     * thinks, all AUTO long.
     */
    public static final double ROBOT_FRONT_IN = 9.76;
    public static final double ROBOT_BACK_IN = 8.03;
    public static final double ROBOT_HALF_WIDTH_IN = 8.9;
    public static final double CAMERA_BACK_IN = 4.63;
    public static final double START_OFFSET_IN = ROBOT_FRONT_IN - HALF_LENGTH_IN;

    // ---------------- CELL AprilTag stickers (CAD am-5888, RED) ----------------

    /**
     * A CELL's tag sticker (4 tags in a row, 17 x 4.3 in) is on its bottom face. Up: centre 49.67 in high, 12.89 in
     * from the pivot (y 0) toward its own end; tipped down: 35.65 in high, 11.4 in from the pivot. The HIVE tilts 30 deg
     * either way and BOTH stickers face the same way: down and toward the end that is up. So the audience-side spot sees
     * our CELLs only before TIP 1, the REAR-side spot only after it (from the far spot, no tag in view in good light =
     * most likely NOT tipped).
     */
    public static final double TAG_UP_HEIGHT_IN = 49.67;
    public static final double TAG_DOWN_HEIGHT_IN = 35.65;
    public static final double TAG_UP_Y = 12.89;

    // ---------------- Far CELL (the one that comes UP after the first TIP) ----------------

    /** Centre of a CELL opening when UP, mirrored from the AUDIENCE CELL (CAD y -1.6..-20.1) about the pivot (y 0). */
    public static final double FAR_CELL_X = CELL_X;
    public static final double FAR_CELL_Y = 10.85;
    /**
     * Far shot spot, in the HIVE lane. The real mirror of the near shot spot: with the as-built robot
     * (START_OFFSET_IN, ROBOT_FRONT_IN) the intake ends ~2 in off the REAR wall and the camera sees the far CELL's
     * sticker from 41.3 in, as the near spot sees the near one (9 Nov run 212355: seen, 46.3 in high); closer and the
     * sticker leaves the top of the picture. The robot arrives straight down the lane, heading 90 (turning under the
     * HIVE puts a corner within 1.5 in of a leg); the far aim then turns it on the spot to FAR_AIM_HEADING (FAR_AIM):
     * its centre line (camera and launcher are on it) through the far CELL's opening, where the camera aim ends
     * (HiveVision's bearing is to the opening's centre). FLOWER 1 (rear wall, ring x -26.4..-20.4) stays 1.4 in away.
     */
    public static final double FAR_SHOT_Y = 58.0;
    public static final Pose FAR_SHOT = new Pose(HIVE_LANE_X, FAR_SHOT_Y, FACE_REAR);
    public static final double FAR_AIM_HEADING = Math.toDegrees(Math.atan2(FAR_SHOT_Y + START_OFFSET_IN - FAR_CELL_Y,
            HIVE_LANE_X - FAR_CELL_X));
    public static final Pose FAR_AIM = new Pose(HIVE_LANE_X, FAR_SHOT_Y, FAR_AIM_HEADING);
    /**
     * The camera aim at the far spot may turn the robot this far either way from FAR_AIM_HEADING (aim trim, the CELL's
     * real place); HivePlanner.farAimLimits cuts it further where Pedro really stopped, so the as-built robot stays
     * FAR_AIM_CLEAR_IN off the REAR wall, FLOWER 1 and our side's line with FAR_AIM_HEADING_ERR_DEG of Pedro heading
     * error (HivePlannerTest). An aim trim bigger than this is cut there (INIT warns).
     */
    public static final double FAR_AIM_MAX_TURN_DEG = 2.5;
    public static final double FAR_AIM_CLEAR_IN = 0.4;
    public static final double FAR_AIM_HEADING_ERR_DEG = 1.0;
    /** Already closer than this to the REAR wall, FLOWER 1 or our side's line: the far aim does not turn at all. */
    public static final double FAR_AIM_MIN_CLEAR_IN = 0.1;
    /** Camera floor distance to the far CELL at the far spot when the camera can't see it (for the shot speed). */
    public static final double FAR_SHOT_PLANNED_FLOOR_IN = 39.0;

    // ---------------- FLOWERs (CAD footprints at robot heights: lower ring, middle ring, pipes) ----------------

    /** A FLOWER's footprint: CAD_HALF_WIDTH along its wall either side of its centre, CAD_DEPTH out from the wall. */
    public static final double FLOWER_HALF_WIDTH_IN = 2.98;
    public static final double FLOWER_DEPTH_IN = AutoField.WALL_IN - 65.63;

    // ---------------- FLOWER 1 (rear wall, our side): step 3.5 ----------------

    /** FLOWER 1 centre (CAD) and the front face of its lower ring (the retrieval side, toward the field). */
    public static final double FLOWER_X = -23.4;
    public static final double FLOWER_RING_FRONT_Y = 65.65;
    /**
     * Pedro y when the intake's upper Gecko row touches the FLOWER's middle-ring lip (robot_reach_probe, 18 Oct: CAD
     * origin 208.65 mm in front of the ring face; Pedro's centre is 12 mm ahead of it; minus START_OFFSET_IN): the "stock
     * stop" the flower test kit uses. The creep also ends on contact. CALIBRATE on a real FLOWER.
     */
    public static final double FLOWER_STOP_Y = FLOWER_RING_FRONT_Y - (208.65 - 12.0) / 25.4 - START_OFFSET_IN;
    /** Where the arm goes down: intake ~6 in short of the ring. */
    public static final double FLOWER_APPROACH_Y = FLOWER_STOP_Y - 6.0;
    public static final Pose FLOWER_APPROACH = new Pose(FLOWER_X, FLOWER_APPROACH_Y, FACE_REAR);
    public static final Pose FLOWER_STOP = new Pose(FLOWER_X, FLOWER_STOP_Y, FACE_REAR);

    /** The near CELL's spot, for reference (AutoField.SHOT). */
    public static final Pose NEAR_SHOT = SHOT;

    /**
     * Slot A facing the REAR wall (HIVE AUTO's park from the far spot): 1.6 in lower than AutoField's SLOT_A_Y so the
     * as-built robot (short back toward the zone, START_OFFSET_IN) is still 4.9 in inside the LOADING ZONE.
     */
    public static final double REAR_SLOT_A_Y = SLOT_A_Y - 1.6;
}

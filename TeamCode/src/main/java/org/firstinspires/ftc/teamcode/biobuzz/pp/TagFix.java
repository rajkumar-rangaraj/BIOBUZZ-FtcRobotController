package org.firstinspires.ftc.teamcode.biobuzz.pp;

/**
 * Corrects the pose from a camera sighting of one of our CELLs.
 *
 * The camera gives the CELL opening's floor position relative to the lens (forward / right). With the Pinpoint heading
 * at the moment the frame was taken and the CELL's known field position, that gives the robot's field position at
 * that moment. Only the position is corrected (the Pinpoint gyro is better than the camera for heading).
 * <ul>
 *   <li>Used only when the CELL is UP (DOWN CELLs sit elsewhere), seen within FIX_MAX_FLOOR_IN, the frame is fresh,
 *       and the robot isn't turning or driving fast (the frame time is only known to a few ms).</li>
 *   <li>First fix (position unknown): the pose is placed there, if the result is a place the robot can be (inside the
 *       field, not under the HIVE); else the CELL's other place is tried (names can be swapped).</li>
 *   <li>Then each fix removes a share of the error (more with 2+ tags). A fix more than FIX_REJECT_IN off is an outlier;
 *       FIX_SNAP_AFTER outliers in a row that agree with each other mean the pose was wrong (a slip, a hard hit): it
 *       snaps to them.</li>
 *   <li>CELL names (which name sits at which place) can't be learned from a pose that was itself placed from a name:
 *       a wrong guess and a pose off by the gap between the CELLs look the same. So:
 *       <ul>
 *         <li>pose from AUTO / a wall start ({@link #poseFromStart}): sightings vote where their CELL is
 *             ({@link CellMap#vote}) until the names are confirmed (a fix that agrees, or a remap);</li>
 *         <li>pose from a CAMERA first fix: our OTHER CELL decides. If it fits the pose, the names are confirmed; if it
 *             fits the swapped names, after FIX_SWAP_CONFIRM sightings both names move and the pose moves by the
 *             gap;</li>
 *         <li>once confirmed, nothing renames the CELLs: a CELL-gap error is a pose error and snaps.</li>
 *       </ul></li>
 * </ul>
 * No FTC SDK dependency: unit-tested (TagFixTest).
 */
public class TagFix {

    public enum Result { NO_HEADING, UNKNOWN_CELL, DOWN, FAR, OLD_FRAME, MOVING, IMPLAUSIBLE, FIRST_FIX, CORRECTED,
        OUTLIER, SNAPPED }

    private final double camBackIn, camLeftIn;
    private int outliers = 0;
    private double[] lastOutlierResidual = null;
    private double residualIn = Double.NaN;
    private String event = null;
    private boolean poseIndependent = false;     // the pose came from AUTO / a wall start, not from a CELL name
    private boolean namesConfirmed = false;      // the name -> place mapping has been checked
    private String firstName = null;             // the guessed name a CAMERA-start pose rests on ...
    private CellMap.Place firstPlace = null;     // ... and where it was assumed to be
    private int swapVotes = 0;

    public TagFix(double camBackIn, double camLeftIn) {
        this.camBackIn = camBackIn;
        this.camLeftIn = camLeftIn;
    }

    public static TagFix standard() {
        return new TagFix(PpConstants.CAMERA_BACK_IN, PpConstants.CAMERA_LEFT_IN);
    }

    /**
     * @param now      seconds, same clock as the pose
     * @param fwdIn    CELL opening, floor distance along the camera's axis (camera looks out of the robot's back)
     * @param rightIn  CELL opening, floor distance to the camera's right
     * @param heightIn tag height above the floor (UP / DOWN)
     * @param frameT   when the frame was taken
     */
    public Result apply(double now, FieldPose pose, CellMap cells, String cellName, int tags, double fwdIn,
                        double rightIn, double heightIn, double frameT) {
        event = null;
        residualIn = Double.NaN;
        CellMap.Place place = cells.place(cellName);
        if (place == null) {
            return Result.UNKNOWN_CELL;
        }
        boolean isUp = cells.saw(cellName, heightIn, now);          // UP / DOWN even without a heading
        if (cells.event() != null) {
            event = cells.event();
        }
        if (!pose.headingKnown()) {
            return Result.NO_HEADING;
        }
        if (!isUp) {
            return Result.DOWN;
        }
        if (Math.hypot(fwdIn, rightIn) > PpConstants.FIX_MAX_FLOOR_IN) {
            return Result.FAR;
        }
        double[] at = pose.fieldAt(frameT);
        if (at == null || now - frameT > PpConstants.FIX_MAX_FRAME_AGE_SEC) {
            return Result.OLD_FRAME;
        }
        if (Math.abs(pose.turnRateDps()) > PpConstants.FIX_MAX_TURN_DPS || pose.speedIps() > PpConstants.FIX_MAX_SPEED_IPS) {
            return Result.MOVING;
        }
        double h = at[2];

        if (!pose.positionKnown()) {
            double[] m = robotXY(h, fwdIn, rightIn, cells.x(place), cells.y(place), camBackIn, camLeftIn);
            if (!plausible(m[0], m[1])) {
                CellMap.Place o = CellMap.other(place);
                double[] m2 = robotXY(h, fwdIn, rightIn, cells.x(o), cells.y(o), camBackIn, camLeftIn);
                if (!plausible(m2[0], m2[1])) {
                    return Result.IMPLAUSIBLE;
                }
                m = m2;
                cells.assign(cellName, o);                      // the guess was impossible: the names are known now
                place = o;                                      // (assign moved this sighting's UP state: no 2nd saw)
                namesConfirmed = true;
            }
            pose.snapAt(frameT, m[0], m[1]);
            poseIndependent = false;                            // this pose rests on the name -> place guess
            firstName = cellName;
            firstPlace = place;
            swapVotes = 0;
            outliers = 0;
            event = String.format("FIRST_FIX %s at %s: robot (%.1f, %.1f) hdg %.1f, %d tags%s", cellName, place, m[0],
                    m[1], h, tags, namesConfirmed ? "" : " (CELL names not confirmed yet)");
            return Result.FIRST_FIX;
        }

        // Pose placed from a guessed name: our other CELL tells whether the guess was right. Swapped names put the
        // first CELL at this one's place and this one at the first's, so the pose would be off by the gap between them.
        if (!namesConfirmed && !poseIndependent && firstPlace != null && place != firstPlace) {
            double[] mNow = robotXY(h, fwdIn, rightIn, cells.x(place), cells.y(place), camBackIn, camLeftIn);
            double[] rNow = pose.residualAt(frameT, mNow[0], mNow[1]);
            double gx = cells.x(place) - cells.x(firstPlace), gy = cells.y(place) - cells.y(firstPlace);
            double[] mSwap = robotXY(h, fwdIn, rightIn, cells.x(firstPlace), cells.y(firstPlace), camBackIn, camLeftIn);
            double[] rSwap = pose.residualAt(frameT, mSwap[0] - gx, mSwap[1] - gy);
            if (Math.hypot(rNow[0], rNow[1]) <= PpConstants.FIX_REJECT_IN) {
                namesConfirmed = true;
                event = "CELL names confirmed by " + cellName;
            } else if (Math.hypot(rSwap[0], rSwap[1]) <= PpConstants.FIX_REJECT_IN) {
                if (++swapVotes < PpConstants.FIX_SWAP_CONFIRM) {
                    residualIn = Math.hypot(rNow[0], rNow[1]);
                    return Result.OUTLIER;
                }
                cells.assign(firstName, place);                 // both names move; the pose moves by the gap
                pose.shift(gx, gy);
                namesConfirmed = true;
                outliers = 0;
                lastOutlierResidual = null;
                event = String.format("SWAPPED CELL names: %s is at %s; pose moved %.1f in", firstName, place,
                        Math.hypot(gx, gy));
                return Result.SNAPPED;
            } else {
                swapVotes = 0;
            }
        }

        // Names learned only from a pose that doesn't rest on them (AUTO / wall start), and only until confirmed: after
        // that a sighting a CELL-gap off is a pose error and snaps (below) instead of renaming the CELLs.
        if (poseIndependent && !namesConfirmed) {
            double[] cellSeen = cellXY(at[0], at[1], h, fwdIn, rightIn, camBackIn, camLeftIn);
            if (cells.vote(cellName, cellSeen[0], cellSeen[1])) {
                event = cells.event();
                place = cells.place(cellName);
                namesConfirmed = true;
            }
        }
        double[] m = robotXY(h, fwdIn, rightIn, cells.x(place), cells.y(place), camBackIn, camLeftIn);
        double[] r = pose.residualAt(frameT, m[0], m[1]);
        residualIn = Math.hypot(r[0], r[1]);
        if (residualIn > PpConstants.FIX_REJECT_IN) {
            boolean agrees = lastOutlierResidual != null && Math.hypot(r[0] - lastOutlierResidual[0],
                    r[1] - lastOutlierResidual[1]) <= PpConstants.FIX_AGREE_IN;
            outliers = agrees ? outliers + 1 : 1;
            lastOutlierResidual = r;
            if (outliers >= PpConstants.FIX_SNAP_AFTER && plausible(m[0], m[1])) {
                pose.snapAt(frameT, m[0], m[1]);
                outliers = 0;
                lastOutlierResidual = null;
                event = String.format("SNAPPED %s: pose was %.1f in off (%d fixes agreed)", cellName, residualIn,
                        PpConstants.FIX_SNAP_AFTER);
                return Result.SNAPPED;
            }
            return Result.OUTLIER;
        }
        if (poseIndependent && !namesConfirmed && residualIn <= PpConstants.FIX_AGREE_IN) {
            namesConfirmed = true;                              // an independent pose agrees with this name's place
        }
        outliers = 0;
        lastOutlierResidual = null;
        double gain = tags >= 2 ? PpConstants.FIX_GAIN_MULTI_TAG : PpConstants.FIX_GAIN_ONE_TAG;
        pose.shift(gain * r[0], gain * r[1]);
        return Result.CORRECTED;
    }

    /** The pose was set from a start that doesn't depend on the CELL names (AUTO's pose, a wall start). */
    public void poseFromStart() {
        poseIndependent = true;
        firstPlace = null;
        firstName = null;
    }

    /** The CELL name -> place mapping has been checked (by an independent pose or our other CELL). */
    public boolean namesConfirmed() {
        return namesConfirmed;
    }

    /** Pose error (in) found by the last fix, NaN if it made none. */
    public double residualIn() {
        return residualIn;
    }

    /**
     * May this frame steer the aim (the camera's position offset)? Accepted fixes, and FAR ones: too noisy to move the
     * pose, but the camera's own view of the CELL is still what the shot must line up with (the fire gate checks the
     * camera bearing, so the aim has to follow it).
     */
    public static boolean usableForAim(Result r) {
        return r == Result.CORRECTED || r == Result.SNAPPED || r == Result.FIRST_FIX || r == Result.FAR;
    }

    /** FIRST_FIX / SNAPPED / REMAP detail from the last apply, null if none. */
    public String event() {
        return event;
    }

    /** Robot centre that sees the CELL at (cellX, cellY) this way with this heading. */
    public static double[] robotXY(double headingDeg, double fwdIn, double rightIn, double cellX, double cellY,
                                   double camBackIn, double camLeftIn) {
        double[] rel = cellFromCentre(headingDeg, fwdIn, rightIn, camBackIn, camLeftIn);
        return new double[]{cellX - rel[0], cellY - rel[1]};
    }

    /** Where the camera puts the CELL, from the robot pose. */
    public static double[] cellXY(double x, double y, double headingDeg, double fwdIn, double rightIn,
                                  double camBackIn, double camLeftIn) {
        double[] rel = cellFromCentre(headingDeg, fwdIn, rightIn, camBackIn, camLeftIn);
        return new double[]{x + rel[0], y + rel[1]};
    }

    /** CELL minus robot centre, field frame. The camera looks backward: its right is the robot's left. */
    private static double[] cellFromCentre(double headingDeg, double fwdIn, double rightIn, double camBackIn,
                                           double camLeftIn) {
        double rx = -camBackIn - fwdIn;                 // robot frame: x forward (intake), y left
        double ry = camLeftIn + rightIn;
        double c = Math.cos(Math.toRadians(headingDeg)), s = Math.sin(Math.toRadians(headingDeg));
        return new double[]{c * rx - s * ry, s * rx + c * ry};
    }

    /** The camera's floor vector to a point it sees at (x right, y along the lens, z up) with the lens tilted up. */
    public static double[] floorVector(double xIn, double yIn, double zIn, double tiltDeg) {
        double t = Math.toRadians(tiltDeg);
        return new double[]{yIn * Math.cos(t) - zIn * Math.sin(t), xIn};
    }

    /** Horizontal bearing to that point, CCW positive (left of the camera's axis = +), as the SDK's bearing. */
    public static double bearingDeg(double fwdIn, double rightIn) {
        return Math.toDegrees(Math.atan2(-rightIn, fwdIn));
    }

    /** A robot centre the robot can actually be at: on the field and not inside the HIVE's floor frame. */
    static boolean plausible(double x, double y) {
        double edge = 70.5 - 7;
        if (Math.abs(x) > edge || Math.abs(y) > edge) {
            return false;
        }
        return !(Math.abs(x) < 24.75 + 6 && Math.abs(y) < 19.5 + 6);
    }
}

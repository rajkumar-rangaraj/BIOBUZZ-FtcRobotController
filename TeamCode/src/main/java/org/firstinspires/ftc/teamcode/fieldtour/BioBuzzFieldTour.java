package org.firstinspires.ftc.teamcode.fieldtour;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

/*
 * BioBuzz Field Tour (test only) - place the robot on any tile, tell it which tile + which way it faces, and it tours
 * the whole field (lap near the walls, lap round the HIVE frame) with Pedro Pathing, then comes back to where it
 * started. Clear field except the HIVE. Field frame and map: see FieldMap.
 *
 * INIT:  D-pad LEFT / RIGHT = tile column (1 at the RED wall ... 6 at the BLUE wall)
 *        D-pad DOWN / UP    = tile row (1 = right side ... 6 = left side, seen from the RED alliance station)
 *        Y / Triangle       = which way the robot FRONT (intake side) faces
 * PLAY:  B / Circle         = STOP at once
 */
@TeleOp(name = "BioBuzz Field Tour", group = "BioBuzz Test")
public class BioBuzzFieldTour extends OpMode {
    /** Fraction of the robot's tuned top speed used for the tour. */
    static final double TOUR_SPEED_FRACTION = 0.30;
    static final double STUCK_SEC = 2.0;
    static final double STUCK_MIN_MOVE_IN = 1.0;

    private enum State { WAITING, DRIVING, DONE, STOPPED, STUCK, NOT_TUNED, BAD_START }

    private Follower follower;
    private String followerError;
    private int column = 1, row = 1;
    private FieldMap.Facing facing = FieldMap.Facing.BLUE_WALL;
    private double homeX, homeY, homeHeadingDeg;
    private List<TourPlanner.Point> plan;
    private int index = 0;
    private State state = State.WAITING;
    private double speedInPerSec;
    private final ElapsedTime clock = new ElapsedTime();
    private final ElapsedTime segmentTimer = new ElapsedTime();
    private final ElapsedTime moveTimer = new ElapsedTime();
    private double lastMoveX, lastMoveY;

    @Override
    public void init() {
        try {
            follower = Constants.create(hardwareMap);
        } catch (Exception e) {
            follower = null;
            followerError = e.getMessage();
        }
        speedInPerSec = TOUR_SPEED_FRACTION * Constants.foresightConfig.maxAchievableForwardVelocity.get();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpadLeftWasPressed()) {
            column = Math.max(1, column - 1);
        } else if (gamepad1.dpadRightWasPressed()) {
            column = Math.min(FieldMap.TILES, column + 1);
        } else if (gamepad1.dpadDownWasPressed()) {
            row = Math.max(1, row - 1);
        } else if (gamepad1.dpadUpWasPressed()) {
            row = Math.min(FieldMap.TILES, row + 1);
        }
        if (gamepad1.yWasPressed()) {
            facing = facing.next();
        }
        homeX = FieldMap.tileCentreX(column);
        homeY = FieldMap.tileCentreY(row);
        homeHeadingDeg = facing.headingDeg;
        String problem = FieldMap.startProblem(homeX, homeY);

        telemetry.addLine("FIELD TOUR - put the robot in the MIDDLE of a tile, square to the walls");
        telemetry.addData("Tile", "column %d (D-pad L/R), row %d (D-pad D/U)", column, row);
        telemetry.addData("Front faces", "%s   (Y changes)", facing.label);
        telemetry.addData("Start", problem == null ? String.format("x %.1f  y %.1f in  OK", homeX, homeY) : "NOT ALLOWED: " + problem);
        addMap();
        if (problem == null) {
            List<TourPlanner.Point> p = TourPlanner.plan(homeX, homeY);
            telemetry.addData("Tour", "%d legs, %.0f in, about %.0f s at %.0f in/s", p.size(),
                    TourPlanner.length(homeX, homeY, p), TourPlanner.length(homeX, homeY, p) / speedInPerSec * 1.3,
                    speedInPerSec);
        }
        if (!Constants.FORESIGHT_TUNED) {
            telemetry.addLine("Pedro NOT TUNED: PLAY only shows the plan (run AutoTune first)");
        }
        if (follower == null) {
            telemetry.addData("Pedro", "could not start: %s", followerError);
        }
    }

    /** 6 x 6 map, BLUE wall on the right, left side (seen from RED) at the top. # = HIVE frame, arrow = robot. */
    private void addMap() {
        String arrow = facing == FieldMap.Facing.BLUE_WALL ? ">" : facing == FieldMap.Facing.LEFT_SIDE ? "^"
                : facing == FieldMap.Facing.RED_WALL ? "<" : "v";
        telemetry.addLine("RED wall |  map  | BLUE wall");
        for (int r = FieldMap.TILES; r >= 1; r--) {
            StringBuilder sb = new StringBuilder("  " + r + "  ");
            for (int c = 1; c <= FieldMap.TILES; c++) {
                boolean hive = FieldMap.startProblem(FieldMap.tileCentreX(c), FieldMap.tileCentreY(r)) != null;
                sb.append(c == column && r == row ? arrow : hive ? "#" : ".").append(' ');
            }
            telemetry.addLine(sb.toString());
        }
        telemetry.addLine("      1 2 3 4 5 6");
    }

    @Override
    public void start() {
        clock.reset();
        if (FieldMap.startProblem(homeX, homeY) != null) {
            state = State.BAD_START;
            return;
        }
        plan = TourPlanner.plan(homeX, homeY);
        if (follower == null || !Constants.FORESIGHT_TUNED) {
            state = State.NOT_TUNED;
            return;
        }
        follower.setPose(pedro(homeX, homeY));
        state = State.DRIVING;
        index = 0;
        startSegment();
    }

    private Pose pedro(double x, double y) {
        return new Pose(x + 72, y + 72, Math.toRadians(homeHeadingDeg));
    }

    private double fieldX() {
        return follower.pose().x() - 72;
    }

    private double fieldY() {
        return follower.pose().y() - 72;
    }

    private void startSegment() {
        TourPlanner.Point p = plan.get(index);
        Path path = line(follower.pose(), pedro(p.x, p.y))
                .constant(Math.toRadians(homeHeadingDeg))                  // never turns: footprint stays square
                .with(Constants.foresightConfig.maxPathSpeed.at(speedInPerSec));
        follower.follow(path);
        segmentTimer.reset();
        moveTimer.reset();
        lastMoveX = fieldX();
        lastMoveY = fieldY();
    }

    @Override
    public void loop() {
        if (follower != null && state == State.DRIVING) {
            follower.update();
            if (gamepad1.b) {
                follower.stop();
                state = State.STOPPED;
            } else if (segmentTimer.seconds() > 0.2 && !follower.isBusy()) {
                index++;
                if (index >= plan.size()) {
                    state = State.DONE;
                    follower.hold(pedro(homeX, homeY));
                } else {
                    startSegment();
                }
            } else if (Math.hypot(fieldX() - lastMoveX, fieldY() - lastMoveY) >= STUCK_MIN_MOVE_IN) {
                lastMoveX = fieldX();
                lastMoveY = fieldY();
                moveTimer.reset();
            } else if (segmentTimer.seconds() > 1.0 && moveTimer.seconds() > STUCK_SEC) {
                follower.stop();
                state = State.STUCK;
            }
        } else if (follower != null && state == State.DONE) {
            follower.update();                                  // keep holding HOME
            if (gamepad1.b) {
                follower.stop();
                state = State.STOPPED;
            }
        }

        telemetry.addData("State", "%s   (B = stop)", state);
        if (plan != null && index < plan.size() && state == State.DRIVING) {
            telemetry.addData("Leg", "%d / %d  -> %s", index + 1, plan.size(), plan.get(index).label);
        }
        if (follower != null && state != State.NOT_TUNED && state != State.BAD_START) {
            double x = fieldX(), y = fieldY();
            double hdg = Math.toDegrees(follower.pose().heading());
            FieldMap.Place near = FieldMap.nearest(x, y);
            telemetry.addData("Where am I", "x %.1f  y %.1f in  heading %.0f deg", x, y, hdg);
            telemetry.addData("Nearest", "%s, %.0f in", near.label, Math.hypot(near.x - x, near.y - y));
            telemetry.addData("From HOME", "%.1f in, heading off %.1f deg", Math.hypot(x - homeX, y - homeY),
                    normalizeDeg(hdg - homeHeadingDeg));
        }
        if (state == State.NOT_TUNED && plan != null) {
            telemetry.addLine("Pedro not tuned - the plan (field inches):");
            for (int i = 0; i < plan.size(); i++) {
                telemetry.addData(String.valueOf(i + 1), "%s  (%.1f, %.1f)", plan.get(i).label, plan.get(i).x, plan.get(i).y);
            }
        }
        if (state == State.STUCK) {
            telemetry.addLine("STUCK: the robot did not move for " + STUCK_SEC + " s - check for something in the way");
        }
        telemetry.addData("Time", "%.1f s", clock.seconds());
    }

    private static double normalizeDeg(double d) {
        while (d > 180) {
            d -= 360;
        }
        while (d <= -180) {
            d += 360;
        }
        return d;
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.stop();
        }
    }
}

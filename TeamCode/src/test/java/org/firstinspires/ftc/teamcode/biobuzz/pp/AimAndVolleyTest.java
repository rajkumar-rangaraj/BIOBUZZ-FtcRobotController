package org.firstinspires.ftc.teamcode.biobuzz.pp;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Aim heading + distance from the pose, the heading lock, the flywheel settle gate and the volley. */
public class AimAndVolleyTest {

    @Test
    public void solverAimsTheCameraAtTheCell() {
        ShotSolver.Aim a = ShotSolver.solve(-12.5, -58, -90, -12.5, -17, 8, 0, 0);
        assertEquals(33, a.distanceIn, 1e-9);
        assertEquals(-90, a.aimHeadingDeg, 1e-9);
        assertEquals(0, a.errorDeg, 1e-9);

        ShotSolver.Aim t = ShotSolver.solve(-12.5, -58, -90, -12.5, -17, 8, 0, 2);   // + trim turns LEFT (CCW)
        assertEquals(-2, ShotSolver.cameraBearing(-12.5, -58, t.aimHeadingDeg, -12.5, -17, 8, 0), 1e-4);  // as X does
        assertTrue(t.errorDeg > 1.5 && t.errorDeg < 2);     // the camera is behind the turn centre: less than 2 deg
    }

    @Test
    public void solverFromTheSideAndWithAnOffsetCamera() {
        ShotSolver.Aim a = ShotSolver.solve(-50, -50, 0, -12.5, -17, 8, 0, 0);
        double expected = Math.toDegrees(Math.atan2(33, 37.5)) + 180 - 360;
        assertEquals(expected, a.aimHeadingDeg, 1e-6);
        ShotSolver.Aim off = ShotSolver.solve(-12.5, -58, -90, -12.5, -17, 8, 3, 0);
        assertEquals(0, ShotSolver.cameraBearing(-12.5, -58, off.aimHeadingDeg, -12.5, -17, 8, 3), 1e-4);
    }

    @Test
    public void trimIsSolvedInTheTiltedCameraFrameLikeTheCameraTeleOp() {
        double up = 51.4 - 12.2;
        ShotSolver.Aim a = ShotSolver.solve(-12.5, -58, -90, -12.5, -17, 8, 0, 10, 30, up);
        double sdk = ShotSolver.sdkBearing(-12.5, -58, a.aimHeadingDeg, -12.5, -17, 8, 0, 30, up);
        assertEquals(-10, sdk, 1e-3);                       // SDK bearing + trim = 0, as X aims
        assertTrue(Math.abs(a.errorDeg) > 10);              // a floor angle bigger than the trim
        ShotSolver.Aim none = ShotSolver.solve(-12.5, -58, -90, -12.5, -17, 8, 0, 0, 30, up);
        assertEquals(0, none.errorDeg, 1e-6);               // no trim: straight at the CELL either way
    }

    @Test
    public void farFramesSteerTheAimSoThePoseAndTheCameraAgree() {
        double cam = 8, bx = -12.5, by = 16.5;              // CELL B seen from the far corner: ~94 in, FAR for fixes
        double rx = -60, ry = -60;                          // the robot really is here ...
        double px = -52, py = -60;                          // ... the pose says 8 in to the right
        ShotSolver.Aim poseOnly = ShotSolver.solve(px, py, 0, bx, by, cam, 0, 0);
        double h = poseOnly.aimHeadingDeg;                  // the lock settles here on the pose alone
        double camBearing = ShotSolver.cameraBearing(rx, ry, h, bx, by, cam, 0);
        assertTrue(Math.abs(camBearing) > 2);               // the camera gate would hold every ball

        // The frame (FAR: no pose correction) -> where the camera puts the robot -> aim offset
        double c = Math.cos(Math.toRadians(h)), s = Math.sin(Math.toRadians(h));
        double ex = bx - rx, ey = by - ry;
        double fwd = -(c * ex + s * ey) - cam, right = -s * ex + c * ey;
        FieldPose p = FieldPose.standard();
        p.update(0, 0, 0, 0, true);
        p.setPose(px, py, h);
        CellMap m = new CellMap(-12.5, -17, bx, by, false);
        assertEquals(TagFix.Result.FAR, new TagFix(cam, 0).apply(0.01, p, m, "RED_REAR", 4, fwd, right, 51, 0));
        assertTrue(TagFix.usableForAim(TagFix.Result.FAR));
        double[] seen = TagFix.robotXY(h, fwd, right, bx, by, cam, 0);
        ShotSolver.Aim withCamera = ShotSolver.solve(px + (seen[0] - px), py + (seen[1] - py), h, bx, by, cam, 0, 0);
        assertEquals(0, ShotSolver.cameraBearing(rx, ry, withCamera.aimHeadingDeg, bx, by, cam, 0), 1e-4);
    }

    @Test
    public void poseFireGateChecksTheCameraAndGivesWayOnlyWhenNearlyAimed() {
        PoseFireGate g = new PoseFireGate(1.5, 2.0, 25, 2.0, 5);
        assertTrue(g.update(0, true, true, 0.4, 0, 0.5, false, false));
        assertFalse(g.update(0.1, true, true, 0.4, 0, 3.0, false, false));     // the camera disagrees: held
        assertEquals(PoseFireGate.Hold.CAMERA, g.hold());
        assertTrue(g.update(2.2, true, true, 0.4, 0, 3.0, false, false));      // 2 s, both within 5 deg: goes
        assertTrue(g.gaveWay());

        g.reset();
        assertFalse(g.update(3.0, true, true, 0.4, 0, 6.0, false, false));
        assertFalse(g.update(5.5, true, true, 0.4, 0, 6.0, false, false));     // camera 6 deg off: never gives way
        assertFalse(g.update(6.0, true, true, 0.4, 0, Double.NaN, false, true)); // driver turning: never
        assertFalse(g.update(9.0, true, true, 0.4, 0, Double.NaN, false, true));
        assertEquals(PoseFireGate.Hold.DRIVER_TURNING, g.hold());

        g.reset();
        assertFalse(g.update(10, true, true, 0.4, 0, Double.NaN, true, false)); // driving to the SHOT SPOT
        assertFalse(g.update(12.1, true, true, 0.4, 0, Double.NaN, true, false));
        assertTrue(g.spotGaveWay());                                           // the spot is given up, not the aim
        assertTrue(g.update(12.2, true, true, 0.4, 0, Double.NaN, false, false));
        g.reset();
        assertFalse(g.spotGaveWay());
    }

    @Test
    public void headingLockTurnsTheShortWayWithLimits() {
        HeadingLock l = new HeadingLock(0.015, 0.0012, 0.10, 0.45, 0.6);
        assertEquals(0, l.turnCw(0.5, 0), 0);
        assertEquals(-0.10, l.turnCw(2, 0), 1e-9);          // CCW error -> negative clockwise power, at least the min
        assertEquals(0.45, l.turnCw(-90, 0), 1e-9);
        assertTrue(l.turnCw(5, 200) > 0);                   // swinging fast toward it: brake
    }

    @Test
    public void settleBlocksOvershootAndRecovery() {
        FlywheelSettle s = new FlywheelSettle(15, 25, 20, 40, 0.3);
        assertFalse(s.update(0.00, 1400, 1300));            // dipped
        assertFalse(s.update(0.03, 1400, 1380));            // rising fast
        assertFalse(s.update(0.06, 1400, 1430));            // overshoot (+30)
        assertTrue(s.update(0.09, 1400, 1410));             // back, falling
        assertTrue(s.update(0.12, 1400, 1400));
        assertFalse(s.update(0.15, 1400, 1420));            // one step over: +20 is too fast
    }

    @Test
    public void settleFallsBackForATiredBattery() {
        FlywheelSettle s = new FlywheelSettle(15, 25, 20, 40, 0.3);
        assertFalse(s.update(0.0, 1400, 1365));             // -35: not yet
        assertFalse(s.update(0.2, 1400, 1365));
        assertTrue(s.update(0.31, 1400, 1365));             // near the target for 0.3 s: fine
        assertFalse(s.update(0.4, 1400, 1430));             // too fast is never fine
    }

    @Test
    public void volleyRunsUntilEmptyTapCancelJamOrTimeout() {
        VolleyFire v = new VolleyFire(5, 8);
        assertFalse(v.update(0, false, false, 0, false, false));
        assertTrue(v.update(0.1, true, false, 0, false, false));
        assertTrue(v.update(0.5, false, false, 2, true, false));     // ball 1 after 2 pockets
        assertTrue(v.update(1.0, false, false, 6, true, false));     // ball 2: the empty count starts again
        assertTrue(v.update(1.5, false, false, 10, false, false));
        assertFalse(v.update(1.6, false, false, 11, false, false));  // 5 pockets with nothing
        assertEquals(VolleyFire.Stop.EMPTY, v.lastStop());
        assertEquals(2, v.balls());

        assertTrue(v.update(2.0, true, false, 11, false, false));
        assertFalse(v.update(2.1, true, false, 11, false, false));
        assertEquals(VolleyFire.Stop.TAP, v.lastStop());
        assertTrue(v.update(2.2, true, false, 11, false, false));
        assertFalse(v.update(2.3, false, true, 11, false, false));
        assertEquals(VolleyFire.Stop.CANCEL, v.lastStop());
        assertFalse(v.update(2.4, true, true, 11, false, false));    // B held: no start
        assertTrue(v.update(2.5, true, false, 11, false, false));
        assertFalse(v.update(2.6, false, false, 11, false, true));
        assertEquals(VolleyFire.Stop.JAMMED, v.lastStop());
        assertTrue(v.update(3.0, true, false, 11, false, false));
        assertFalse(v.update(11.0, false, false, 12, false, false));
        assertEquals(VolleyFire.Stop.TIMEOUT, v.lastStop());
    }
}

package org.firstinspires.ftc.teamcode.biobuzz.pp;

/**
 * One press of FIRE fires every ball: the volley keeps asking the windmill for pockets until
 * <ul>
 *   <li>FIRE is pressed again (TAP) or B / unjam (CANCEL),</li>
 *   <li>VOLLEY_EMPTY_POCKETS pockets in a row launched nothing (EMPTY: the robot is out of balls),</li>
 *   <li>the windmill gave up on a jam (JAMMED), or</li>
 *   <li>VOLLEY_MAX_SEC passed (TIMEOUT).</li>
 * </ul>
 * Drivers released and re-pressed FIRE before almost every ball (7-14 Nov logs: 2.15 s per ball vs 0.46 s held).
 * No FTC SDK dependency: unit-tested (VolleyFireTest).
 */
public class VolleyFire {

    public enum Stop { TAP, CANCEL, EMPTY, JAMMED, TIMEOUT }

    private final int emptyPockets;
    private final double maxSec;
    private boolean active = false;
    private double startT;
    private int pocketBase;
    private int balls;
    private int volleys = 0;
    private Stop lastStop = null;
    private String event = null;

    public VolleyFire(int emptyPockets, double maxSec) {
        this.emptyPockets = emptyPockets;
        this.maxSec = maxSec;
    }

    public static VolleyFire standard() {
        return new VolleyFire(PpConstants.VOLLEY_EMPTY_POCKETS, PpConstants.VOLLEY_MAX_SEC);
    }

    /**
     * Call once per loop, after the windmill and the ball counter.
     *
     * @param firePressed FIRE was pressed this loop (an edge, not held)
     * @param cancel      B / unjam held
     * @param pockets     pockets finished so far (ShotFeeder.shots(), only ever grows)
     * @param ballLaunched a ball left this loop (BallCounter)
     * @param jammed      the windmill gave up (ShotFeeder JAMMED)
     * @return true while the volley wants the windmill to feed
     */
    public boolean update(double t, boolean firePressed, boolean cancel, int pockets, boolean ballLaunched,
                          boolean jammed) {
        event = null;
        if (!active) {
            if (firePressed && !cancel) {
                active = true;
                startT = t;
                pocketBase = pockets;
                balls = 0;
                volleys++;
                event = "VOLLEY_START #" + volleys;
            }
            return active;
        }
        if (ballLaunched) {
            balls++;
            pocketBase = pockets;
        }
        if (firePressed) {
            stop(t, Stop.TAP);
        } else if (cancel) {
            stop(t, Stop.CANCEL);
        } else if (jammed) {
            stop(t, Stop.JAMMED);
        } else if (pockets - pocketBase >= emptyPockets) {
            stop(t, Stop.EMPTY);
        } else if (t - startT >= maxSec) {
            stop(t, Stop.TIMEOUT);
        }
        return active;
    }

    private void stop(double t, Stop why) {
        active = false;
        lastStop = why;
        event = String.format("VOLLEY_STOP #%d %s balls=%d time=%.2f", volleys, why, balls, t - startT);
    }

    public boolean active() {
        return active;
    }

    /** Balls launched in the current (or last) volley. */
    public int balls() {
        return balls;
    }

    public Stop lastStop() {
        return lastStop;
    }

    /** VOLLEY_START / VOLLEY_STOP from the last update, null if none. */
    public String event() {
        return event;
    }
}

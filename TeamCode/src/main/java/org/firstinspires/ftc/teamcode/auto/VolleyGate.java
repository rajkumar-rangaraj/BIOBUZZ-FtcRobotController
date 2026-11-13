package org.firstinspires.ftc.teamcode.auto;

/**
 * HIVE AUTO, G417: a volley other than the pre-loads may only feed while the camera evidence it fired on (its CELL seen
 * UP, or the other end seen DOWN) is fresh. Older than {@code waitAfterSec}: WAIT (the windmill stops at once, even
 * mid-pocket, and goes on when the camera sees it again); older than {@code stopAfterSec}, or the CELL confirmed going
 * down: STOP (the volley is over). While it waits the feeder's clock stands still (feederTime), so a pocket's jam
 * timeout never counts the wait. No FTC SDK dependency: unit-tested.
 */
public final class VolleyGate {

    public enum Verdict { GO, WAIT, STOP }

    private final double waitAfterSec;
    private final double stopAfterSec;
    private double evidenceAt;
    private double waitStart = Double.NaN;
    private double waitedSec = 0;
    private int waits = 0;

    /** @param evidenceAt when the camera last saw the evidence the volley was decided on */
    public VolleyGate(double evidenceAt, double waitAfterSec, double stopAfterSec) {
        this.evidenceAt = evidenceAt;
        this.waitAfterSec = waitAfterSec;
        this.stopAfterSec = stopAfterSec;
    }

    /**
     * Call once per loop.
     *
     * @param seenAt   when the camera's latest frame of the evidence CELL agreed with it, or a negative number when that
     *                 frame does not agree (or there is none)
     * @param cellDown the CELL fired at is confirmed going down (or its other end up)
     */
    public Verdict update(double t, double seenAt, boolean cellDown) {
        evidenceAt = Math.max(evidenceAt, seenAt);
        if (cellDown || t - evidenceAt > stopAfterSec) {
            endWait(t);
            return Verdict.STOP;
        }
        if (t - evidenceAt > waitAfterSec) {
            if (Double.isNaN(waitStart)) {
                waitStart = t;
                waits++;
            }
            return Verdict.WAIT;
        }
        endWait(t);
        return Verdict.GO;
    }

    private void endWait(double t) {
        if (!Double.isNaN(waitStart)) {
            waitedSec += t - waitStart;
            waitStart = Double.NaN;
        }
    }

    /** The feeder's clock: {@code t} less every wait so far (including one going on). */
    public double feederTime(double t) {
        return t - waitedSec - (Double.isNaN(waitStart) ? 0 : t - waitStart);
    }

    public double evidenceAgeSec(double t) {
        return t - evidenceAt;
    }

    public boolean waiting() {
        return !Double.isNaN(waitStart);
    }

    public int waits() {
        return waits;
    }

    /** Seconds waited so far, the wait going on at {@code t} included. */
    public double waitedSec(double t) {
        return waitedSec + (Double.isNaN(waitStart) ? 0 : t - waitStart);
    }
}

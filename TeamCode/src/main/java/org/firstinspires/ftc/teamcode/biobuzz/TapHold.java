package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * One button, read once per loop: a tap (let go within holdSec) or a hold (down holdSec). The hold fires while the
 * button is still down; a release that a slow loop hides is judged by the middle of the gap it happened in (review
 * 10 Nov: a 1.05 s press seen down at 0.99 s and up at 1.10 s counted as a tap). No FTC SDK dependency: unit-tested.
 */
public class TapHold {

    public enum Event { NONE, TAP, HOLD }

    private final double holdSec;
    private double downSince = Double.NaN;           // seen going down (NaN: up)
    private double seenDownAt;                       // last loop it was seen down
    private boolean held = false;                    // the hold already fired for this press

    public TapHold(double holdSec) {
        this.holdSec = holdSec;
    }

    /**
     * @param t       now (seconds)
     * @param pressed it went down since the last call (the SDK's wasPressed latch)
     * @param down    it is down now
     */
    public Event update(double t, boolean pressed, boolean down) {
        if (pressed) {
            downSince = t;
            seenDownAt = t;
            held = false;
        }
        if (Double.isNaN(downSince)) {
            return Event.NONE;
        }
        if (down) {
            seenDownAt = t;
            if (!held && t - downSince >= holdSec) {
                held = true;
                return Event.HOLD;
            }
            return Event.NONE;
        }
        double start = downSince;                    // let go between seenDownAt and t
        downSince = Double.NaN;
        if (held) {
            return Event.NONE;
        }
        return (seenDownAt + t) / 2 - start >= holdSec ? Event.HOLD : Event.TAP;
    }
}

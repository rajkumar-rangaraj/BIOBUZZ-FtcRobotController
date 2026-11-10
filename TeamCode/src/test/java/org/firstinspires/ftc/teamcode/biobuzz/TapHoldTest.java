package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class TapHoldTest {

    @Test
    public void aShortPressIsATapWhenLetGo() {
        TapHold b = new TapHold(1.0);
        assertEquals(TapHold.Event.NONE, b.update(0, true, true));
        assertEquals(TapHold.Event.NONE, b.update(0.1, false, true));
        assertEquals(TapHold.Event.TAP, b.update(0.2, false, false));
        assertEquals(TapHold.Event.NONE, b.update(0.3, false, false));
    }

    @Test
    public void aPressAndReleaseBetweenTwoLoopsIsATap() {
        TapHold b = new TapHold(1.0);
        assertEquals(TapHold.Event.TAP, b.update(0.5, true, false));
    }

    @Test
    public void holdingFiresOnceWhileDownAndNothingOnRelease() {
        TapHold b = new TapHold(1.0);
        b.update(0, true, true);
        double t = 0;
        int holds = 0;
        for (int k = 0; k < 30; k++) {
            t += 0.05;
            if (b.update(t, false, true) == TapHold.Event.HOLD) {
                holds++;
                assertEquals(1.0, t, 0.051);
            }
        }
        assertEquals(1, holds);
        assertEquals(TapHold.Event.NONE, b.update(t + 0.05, false, false));
    }

    @Test
    public void aHoldWhoseReleaseASlowLoopHidesIsStillAHold() {
        // review 10 Nov (GPT-6.1): seen down at 0.989 s, up at 1.10 s (a 111 ms loop): was taken for a tap
        TapHold b = new TapHold(1.0);
        b.update(0, true, true);
        assertEquals(TapHold.Event.NONE, b.update(0.989, false, true));
        assertEquals(TapHold.Event.HOLD, b.update(1.10, false, false));
    }

    @Test
    public void aReleaseJustBeforeTheMarkIsATap() {
        TapHold b = new TapHold(1.0);
        b.update(0, true, true);
        b.update(0.90, false, true);
        assertEquals(TapHold.Event.TAP, b.update(1.02, false, false));    // let go ~0.96 s
    }
}

package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class CellCheckTest {

    /** The real readings at the S1 shot spot fall clearly on their side of the threshold. */
    @Test
    public void realReadingsAreClear() {
        for (double up : new double[]{46.1, 46.3}) {                       // 9 Nov, our CELL up
            assertTrue(up + " in is UP", CellCheck.up(up));
            assertTrue("at least 6 in of margin", up - CellCheck.UP_MIN_HEIGHT_IN >= 6.0);
        }
        for (double down : new double[]{29.3, 31.0, 32.1}) {               // 13 Nov, practice HIVE left tipped
            assertFalse(down + " in is DOWN", CellCheck.up(down));
            assertTrue("at least 6 in of margin", CellCheck.UP_MIN_HEIGHT_IN - down >= 6.0);
        }
    }

    /** One odd frame can't flip the verdict either way. */
    @Test
    public void medianIgnoresOneOddFrame() {
        assertTrue(CellCheck.up(CellCheck.median(Arrays.asList(46.0, 46.2, 31.0, 46.3, 46.1))));
        assertFalse(CellCheck.up(CellCheck.median(Arrays.asList(30.0, 31.2, 47.0, 29.8, 30.5))));
    }

    /** Only the last FRAMES heights count (the end of the aim, at the final heading). */
    @Test
    public void usesTheLastFrames() {
        ArrayList<Double> h = new ArrayList<>(Arrays.asList(30.0, 30.0, 30.0, 30.0, 30.0));
        h.addAll(Collections.nCopies(CellCheck.FRAMES, 46.0));
        assertEquals(46.0, CellCheck.median(h), 1e-9);
        assertEquals(46.0, CellCheck.median(Arrays.asList(46.0)), 1e-9);
        assertEquals(45.0, CellCheck.median(Arrays.asList(44.0, 46.0)), 1e-9);
    }

    /** No frames: no verdict, and "not up" (the op mode only asks when it saw the CELL). */
    @Test
    public void noFramesIsNotUp() {
        assertTrue(Double.isNaN(CellCheck.median(new ArrayList<Double>())));
        assertFalse(CellCheck.up(Double.NaN));
        assertFalse(CellCheck.up(new ArrayList<Double>()));
    }

    /** The aim verdict: a steady UP fires; a latest frame reading DOWN vetoes it, even when the median is still UP. */
    @Test
    public void latestDownFrameVetoesTheShot() {
        assertTrue(CellCheck.up(Arrays.asList(46.1, 46.2, 46.3)));
        assertTrue(CellCheck.up(Arrays.asList(46.2)));
        assertTrue(CellCheck.up(Arrays.asList(31.0, 46.0, 46.2, 46.1, 46.3)));      // one odd early frame
        assertFalse(CellCheck.up(Arrays.asList(46.0, 46.2, 30.5)));                 // just tipped?
        assertFalse(CellCheck.up(Arrays.asList(46.0, 30.2, 30.5, 46.1)));           // median DOWN
        assertFalse(CellCheck.up(Arrays.asList(30.0, 31.0, 32.0)));
    }
}

package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ShotResultsTest {

    @Test
    public void aVolleyIsLabelledInTheOrderTheBallsFlew() {
        ShotResults s = new ShotResults(10);
        s.launched(1.0, 48, 1530, 5, 1525);
        s.launched(1.8, 48, 1530, 5, 1525);
        s.launched(2.6, 48, 1530, 5, 1525);
        assertEquals(3, s.waiting(3.0));
        assertEquals(1, s.label(3.0, ShotResults.Result.IN).number);
        assertEquals(2, s.label(3.5, ShotResults.Result.SHORT).number);
        assertEquals(3, s.label(4.0, ShotResults.Result.IN).number);
        assertNull("every ball has its result", s.label(4.5, ShotResults.Result.LONG));
        assertEquals(0, s.waiting(4.5));
        assertEquals(3, s.labelled());
        assertEquals(2, s.count(ShotResults.Result.IN));
        assertEquals(1, s.count(ShotResults.Result.SHORT));
        assertEquals(0, s.count(ShotResults.Result.LONG));
    }

    @Test
    public void aLabelWithoutARecentBallIsNotSaved() {
        ShotResults s = new ShotResults(10);
        assertNull(s.label(1.0, ShotResults.Result.IN));
        s.launched(2.0, Double.NaN, 1450, 0, Double.NaN);
        assertNull("the ball is older than the window", s.label(12.5, ShotResults.Result.IN));
        assertEquals(0, s.labelled());
        assertEquals(1, s.balls());
    }

    @Test
    public void oldBallsDropOutSoTheNextLabelGoesToARecentOne() {
        ShotResults s = new ShotResults(10);
        s.launched(1.0, 40, 1465, 0, 1465);                          // never labelled
        s.launched(20.0, 56, 1590, 5, 1585);
        ShotResults.Ball b = s.label(21.0, ShotResults.Result.LONG);
        assertEquals(2, b.number);
        assertEquals(ShotResults.Result.LONG, b.result());
        assertEquals(56, b.floorIn, 0);
        assertEquals(5, b.targetTps - b.tableTps, 1e-9);
        assertTrue(Double.isNaN(new ShotResults(10).launched(0, Double.NaN, 1450, 0, Double.NaN).floorIn));
    }
}

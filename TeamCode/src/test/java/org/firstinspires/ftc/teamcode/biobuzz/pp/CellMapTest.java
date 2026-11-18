package org.firstinspires.ftc.teamcode.biobuzz.pp;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** CELL places, UP / DOWN target, and learning swapped CELL names. */
public class CellMapTest {

    private static CellMap map() {
        return new CellMap(-12.5, -17, -12.5, 16.5, true);
    }

    @Test
    public void practiceNamesAndTheirTwins() {
        CellMap m = map();
        assertEquals(CellMap.Place.A, m.place("RED_REAR"));
        assertEquals(CellMap.Place.A, m.place("BLUE_AUDIENCE"));
        assertEquals(CellMap.Place.B, m.place("RED_AUDIENCE"));
        assertEquals(CellMap.Place.B, m.place("BLUE_REAR"));
        assertEquals("RED_REAR", m.nameAt(CellMap.Place.A, true));
        assertEquals("BLUE_AUDIENCE", m.nameAt(CellMap.Place.A, false));
        assertNull(m.place("nonsense"));
        CellMap manual = new CellMap(-12.5, -17, -12.5, 16.5, false);
        assertEquals(CellMap.Place.B, manual.place("RED_REAR"));
    }

    @Test
    public void targetFollowsTheUpCellAfterRepeatedSightings() {
        CellMap m = map();
        assertEquals(CellMap.Place.A, m.target());          // A starts UP
        assertTrue(m.saw("RED_AUDIENCE", 51, 0.0));         // B seen UP once: not yet
        assertEquals(CellMap.Place.A, m.target());
        assertTrue(m.saw("RED_AUDIENCE", 51, 0.05));        // twice in a row: B
        assertEquals(CellMap.Place.B, m.target());
        assertFalse(m.saw("RED_AUDIENCE", 30, 0.1));        // B DOWN once: not yet
        assertFalse(m.saw("RED_AUDIENCE", 30, 0.15));       // twice: back to A
        assertEquals(CellMap.Place.A, m.target());
        assertFalse(m.saw("RED_REAR", 30, 0.2));            // A DOWN too, B known DOWN: stay
        assertFalse(m.saw("RED_REAR", 30, 0.25));
        assertEquals(CellMap.Place.A, m.target());
    }

    @Test
    public void heightsInTheBandKeepTheLastState() {
        CellMap m = map();
        assertFalse(m.saw("RED_REAR", 45, 0));              // never seen: in the band = not UP
        assertTrue(m.saw("RED_REAR", 50, 0.05));
        assertTrue(m.saw("RED_REAR", 44, 0.1));             // misread low, still in the band: still UP
        assertFalse(m.saw("RED_REAR", 40, 0.15));
        assertFalse(m.saw("RED_REAR", 46, 0.2));            // misread high, in the band: still DOWN
    }

    @Test
    public void bothCellsInOneFrameDontFlipTheTarget() {
        CellMap m = map();
        for (int i = 0; i < 20; i++) {                      // A and B both UP in every frame
            double t = 0.05 * i;
            m.saw("RED_REAR", 51, t);
            m.saw("RED_AUDIENCE", 51, t);
            assertEquals(CellMap.Place.A, m.target());
        }
    }

    @Test
    public void swappedNamesAreLearnedAfterAFewSightings() {
        CellMap m = map();
        assertFalse(m.vote("RED_REAR", -12, 16));           // RED_REAR seen at B
        assertFalse(m.vote("RED_REAR", -13, 17));
        assertTrue(m.vote("RED_REAR", -12.5, 16));
        assertEquals(CellMap.Place.B, m.place("RED_REAR"));
        assertEquals(CellMap.Place.B, m.place("BLUE_AUDIENCE"));
        assertEquals(CellMap.Place.A, m.place("RED_AUDIENCE"));
        assertEquals(CellMap.Place.A, m.place("BLUE_REAR"));
    }

    @Test
    public void aSightingAtItsOwnPlaceResetsTheVotes() {
        CellMap m = map();
        m.vote("RED_REAR", -12, 16);
        m.vote("RED_REAR", -12, 16);
        m.vote("RED_REAR", -12.5, -17);                     // at A again
        assertFalse(m.vote("RED_REAR", -12, 16));
        assertEquals(CellMap.Place.A, m.place("RED_REAR"));
    }
}

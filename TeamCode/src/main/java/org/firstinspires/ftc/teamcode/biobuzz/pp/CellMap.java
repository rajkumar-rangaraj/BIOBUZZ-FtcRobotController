package org.firstinspires.ftc.teamcode.biobuzz.pp;

import java.util.HashMap;
import java.util.Map;

/**
 * Our ALLIANCE's two CELL openings in the alliance frame, which one is UP, and which CELL name sits where.
 *
 * Places, not names: on the practice HIVE the names were the other way round from the manual's figure (Pedro AUTO
 * logs 13-14 Nov), so a name is only mapped to a place. The first guess is {@link PpConstants#PRACTICE_CELL_NAMES};
 * {@link #vote} remaps a name after a few sightings that clearly sit at the other place.
 *
 * UP / DOWN from a sighting's tag height, with a band between (see {@link #saw}). The target is the place seen UP
 * (place A, in front of the audience-wall start, until a CELL has been seen); it moves to the other place only after
 * repeated sightings, never within one frame.
 * No FTC SDK dependency: names are plain strings (HiveVision.Cell.name()). Unit-tested (CellMapTest).
 */
public class CellMap {

    public enum Place { A, B }

    static final int VOTES_TO_REMAP = 3;
    static final double VOTE_NEAR_IN = 8;            // a sighting this near a place counts for it ...
    static final double VOTE_FAR_IN = 14;            // ... if it is at least this far from the mapped one

    private final double ax, ay, bx, by;
    private final Map<String, Place> map = new HashMap<>();
    private final Map<String, Integer> votes = new HashMap<>();
    private Place target = Place.A;
    private final Boolean[] up = new Boolean[2];
    private final int[] upStreak = new int[2], downStreak = new int[2];
    private final double[] lastUpT = {-1e9, -1e9};
    private String event = null;

    public CellMap(double ax, double ay, double bx, double by, boolean practiceNames) {
        this.ax = ax;
        this.ay = ay;
        this.bx = bx;
        this.by = by;
        Place redRear = practiceNames ? Place.A : Place.B;
        Place other = redRear == Place.A ? Place.B : Place.A;
        map.put("RED_REAR", redRear);
        map.put("BLUE_AUDIENCE", redRear);              // the same CELL for the other ALLIANCE (point symmetry)
        map.put("RED_AUDIENCE", other);
        map.put("BLUE_REAR", other);
    }

    public static CellMap standard() {
        return new CellMap(PpConstants.CELL_A_X, PpConstants.CELL_A_Y, PpConstants.CELL_B_X, PpConstants.CELL_B_Y,
                PpConstants.PRACTICE_CELL_NAMES);
    }

    /** Place of a CELL name, null for an unknown name. */
    public Place place(String cellName) {
        return map.get(cellName);
    }

    public double x(Place p) {
        return p == Place.A ? ax : bx;
    }

    public double y(Place p) {
        return p == Place.A ? ay : by;
    }

    public static Place other(Place p) {
        return p == Place.A ? Place.B : Place.A;
    }

    /** The CELL name mapped to a place for this ALLIANCE (RED_* or BLUE_*), null if none. */
    public String nameAt(Place p, boolean red) {
        for (Map.Entry<String, Place> e : map.entrySet()) {
            if (e.getValue() == p && e.getKey().startsWith(red ? "RED" : "BLUE")) {
                return e.getKey();
            }
        }
        return null;
    }

    /**
     * A CELL seen with its tags this high at time t: updates UP / DOWN and the target. Returns true when it counts as UP.
     * UP at CELL_UP_HEIGHT_IN or more, DOWN under CELL_DOWN_HEIGHT_IN; in between the place keeps what it was. The
     * target moves to the other place only after TARGET_SWITCH_SIGHTINGS UP sightings of it in a row while the target
     * hasn't been seen UP for TARGET_HOLD_SEC (both CELLs in one frame, or a misread height, can't flip it).
     */
    public boolean saw(String cellName, double heightIn, double t) {
        event = null;
        Place p = place(cellName);
        if (p == null || Double.isNaN(heightIn)) {
            return false;
        }
        int i = p.ordinal();
        Boolean state = heightIn >= PpConstants.CELL_UP_HEIGHT_IN ? Boolean.TRUE
                : heightIn < PpConstants.CELL_DOWN_HEIGHT_IN ? Boolean.FALSE : up[i];
        if (state == null) {
            return false;
        }
        if (state) {
            upStreak[i]++;
            downStreak[i] = 0;
            lastUpT[i] = t;
        } else {
            downStreak[i]++;
            upStreak[i] = 0;
        }
        up[i] = state;
        int need = PpConstants.TARGET_SWITCH_SIGHTINGS;
        int o = other(p).ordinal();
        if (state && target != p && upStreak[i] >= need && t - lastUpT[o] >= PpConstants.TARGET_HOLD_SEC) {
            event = "TARGET " + p + " (" + cellName + " seen UP)";
            target = p;
        } else if (!state && target == p && downStreak[i] >= need && !Boolean.FALSE.equals(up[o])) {
            event = "TARGET " + other(p) + " (" + cellName + " at " + p + " seen DOWN)";
            target = other(p);
        }
        return state;
    }

    /** UP sightings in a row at a place (tests). */
    int upStreak(Place p) {
        return upStreak[p.ordinal()];
    }

    /** The place to shoot at. */
    public Place target() {
        return target;
    }

    /** UP / DOWN last seen at a place, null if never seen. */
    public Boolean up(Place p) {
        return up[p.ordinal()];
    }

    /**
     * With the position known: where the camera put this CELL. Several sightings in a row clearly at the other place
     * remap the name (and its twin on the other ALLIANCE). Returns true when it remapped.
     */
    public boolean vote(String cellName, double cellX, double cellY) {
        event = null;
        Place p = place(cellName);
        if (p == null) {
            return false;
        }
        Place o = other(p);
        double toMapped = Math.hypot(cellX - x(p), cellY - y(p));
        double toOther = Math.hypot(cellX - x(o), cellY - y(o));
        if (toOther > VOTE_NEAR_IN || toMapped < VOTE_FAR_IN) {
            votes.put(cellName, 0);
            return false;
        }
        int n = votes.getOrDefault(cellName, 0) + 1;
        votes.put(cellName, n);
        if (n < VOTES_TO_REMAP) {
            return false;
        }
        assign(cellName, o);
        event = "REMAP " + cellName + " -> " + o + " (seen there " + n + " times)";
        return true;
    }

    /**
     * Maps a CELL name (and its twin) to a place, the partner names to the other one. UP / DOWN and the target move
     * with the CELL that was seen at the wrong place.
     */
    public void assign(String cellName, Place to) {
        Place from = place(cellName);
        if (from == null || from == to) {
            return;
        }
        map.put(cellName, to);
        map.put(twin(cellName), to);
        map.put(partner(cellName), from);
        map.put(twin(partner(cellName)), from);
        votes.clear();
        int f = from.ordinal(), d = to.ordinal();
        Boolean a = up[f];
        up[f] = up[d];
        up[d] = a;
        int u = upStreak[f];
        upStreak[f] = upStreak[d];
        upStreak[d] = u;
        int w = downStreak[f];
        downStreak[f] = downStreak[d];
        downStreak[d] = w;
        double l = lastUpT[f];
        lastUpT[f] = lastUpT[d];
        lastUpT[d] = l;
        // The target is a place: it moves with the names only when it was chosen by seeing that CELL UP
        if (target == from && Boolean.TRUE.equals(up[d])) {
            target = to;
        } else if (target == to && Boolean.TRUE.equals(up[f])) {
            target = from;
        }
    }

    /** The other ALLIANCE's CELL at the same place (point symmetry). */
    static String twin(String name) {
        switch (name) {
            case "RED_REAR":
                return "BLUE_AUDIENCE";
            case "BLUE_AUDIENCE":
                return "RED_REAR";
            case "RED_AUDIENCE":
                return "BLUE_REAR";
            default:
                return "RED_AUDIENCE";
        }
    }

    /** Our ALLIANCE's other CELL. */
    static String partner(String name) {
        switch (name) {
            case "RED_REAR":
                return "RED_AUDIENCE";
            case "RED_AUDIENCE":
                return "RED_REAR";
            case "BLUE_REAR":
                return "BLUE_AUDIENCE";
            default:
                return "BLUE_REAR";
        }
    }

    /** What changed in the last saw / vote (TARGET, REMAP), null if nothing. */
    public String event() {
        return event;
    }
}

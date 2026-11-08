# BioBuzz AUTO (Pedro Pathing)

Op modes **BioBuzz AUTO RED** and **BioBuzz AUTO BLUE**. The old AUTO is still on the list as **OLD AUTO RED / BLUE** (group "BioBuzz Old"), as a backup.

## What it does (30 s)

| When (s) | Step | Points |
|---|---|---|
| 0–1 | Leave the wall, spin up the flywheel | LEAVE +3 (counted at the end) |
| 1–9 | Camera aim at our up CELL, fire the 4 pre-loads | HIVE TIP +20 if it tips |
| 9–14 | Drive to our GARDEN, intake its 4 POLLEN | (used for the stretch or in TeleOp) |
| 14–24 | **Stretch:** back to the spot, fire again, only if the camera sees our CELL still **up** | another TIP? |
| **by 24** | **PARK** in our LOADING ZONE, slot A or B | PARK +5 |

**Parking always wins.** At the park time (24 s, or earlier when the drive to the slot is long), the robot stops whatever it is doing and parks.

- If the robot is blocked for 1 s before parking (for example by our partner), it drops that step and goes to park.
- If it is blocked while parking, it waits and tries again until the end of AUTO.

Rules the code keeps:

- **G407 (max 4 balls):** it only goes to the GARDEN after the windmill is empty (4 balls counted, or 3 full windmill turns). Otherwise it parks.
- **G417:** it never fires into a CELL the camera sees **down** (HIVE already tipped).
- **G402:** every route stays on our side of the field.
- It never touches a FLOWER or the HIVE frame. A laptop test checks every route.

## Where things are (RED; BLUE is the same turned around)

```
            REAR wall
   +---------------------------+
   |A  <- LOADING ZONE (RED)   |
   |B     (on our wall)        |
 R |                           | B
 E |S2      [HIVE]             | L
 D |  FLOWER                   | U
   |                           | E
   |GARDEN     S1 (in front of our up CELL)
   +---------------------------+
          AUDIENCE wall
```

- **S1 = CELL wall start** (best): intake against the **audience** wall (BLUE: rear wall), centred on our up CELL, square to the wall. The robot shoots from here.
- **S2 = ALLIANCE wall start:** intake against our alliance wall, centred on the field's middle line (between the FLOWER and the LOADING ZONE). It drives about 5 s to the shot spot, so the stretch is usually skipped.
- **Slot A** = the CORNER end of the LOADING ZONE. **Slot B** = the MIDDLE end (toward the FLOWER). The robot ends 2 in off the wall with 5 in of its body inside the zone. The two slots leave a gap of about 11 in between the robots.

## Before every match: agree with your partner

1. **Start:** only one robot can use S1.
2. **Parking:** we park in **A**, they park in **B**, or the other way round. Tell them they must park only at their end of the zone.
3. **GARDEN:** we take our GARDEN POLLEN. They should not.
4. Our robot drives on a lane beside the HIVE on our side of the field. The partner should stay out of it at the end of AUTO.

## INIT buttons (gamepad 1)

| Button | Setting |
|---|---|
| D-pad ← / → | Start S1 / S2 |
| □ / X | Park slot A / B |
| ✕ / A | Shoot pre-loads on/off |
| △ / Y | GARDEN on/off |
| ○ / B | Stretch (second volley) on/off |
| D-pad ↑ / ↓ | Start delay 0–10 s |
| LB / RB | Park-by time 18–27 s (default 24) |

The screen shows the plan times and **Route check: OK**. It also shows **Pedro: ready**. If it says **NOT READY** (no Pinpoint, or Pedro not tuned), AUTO only backs 8 in off the wall (LEAVE, 3 points).

**Team rule:** place the robot, press INIT, and don't touch it until START. The Pinpoint calibrates while the robot is still.

## Things to check on the field (first practice)

1. **Robot size:** measure from the robot centre to the intake bumper, and from the centre to the side. Set `HALF_LENGTH_IN` / `HALF_WIDTH_IN` in `auto/AutoField.java` (now 9 / 9).
2. **Shot spot:** from S1, do the 4 pre-loads score? If they land short or long, check the camera floor distance in the log and the `AUTO_SPEED_*` table.
3. **GARDEN sweep:** does the intake catch the POLLEN against the wall? If it misses, change `GARDEN_SIDE_GAP_IN` (smaller = closer to the wall).
4. **CELL up/down:** the log column `height_in` should read about 59 for an up CELL. The code fires only above 45 (`CELL_UP_MIN_HEIGHT_IN`).
5. **Parking:** run with slot A and with slot B while a partner (or a box) sits in the other slot. The robot must not touch it.

Logs: `/sdcard/BioBuzzLogs/pedroauto_*.csv`. Steps and `EVENT` lines show each leg, each ball, any BLOCKED, and when PARK started.

## Code

| File | What |
|---|---|
| `auto/AutoField.java` | Places from the official field CAD (LOADING ZONE, GARDEN, CELL, FLOWERs), starts, shot spot, slots |
| `auto/AutoPlanner.java` | Routes, time estimates, park deadline, robot-footprint check |
| `auto/BioBuzzPedroAutoBase.java` | The op mode (Pedro drive, camera aim, ShotFeeder fire, intake, park) |
| `auto/BioBuzzPedroAutoRed/Blue.java` | RED / BLUE entries |
| `test/.../auto/AutoPlannerTest.java` | Laptop tests: legal starts, slots, every route clear, parking from anywhere, time budget |

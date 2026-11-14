# BioBuzz AUTO (Pedro Pathing)

Op modes **BioBuzz AUTO RED** and **BioBuzz AUTO BLUE**. The old AUTO is still on the list as **OLD AUTO RED / BLUE** (group "BioBuzz Old"), as a backup.

## What it does (30 s)

| When (s) | Step | Points |
|---|---|---|
| 0–1 | Leave the wall, spin up the flywheel | LEAVE +3 (counted at the end) |
| 1–7 | Camera aim at our up CELL, fire the 4 pre-loads at **1440 t/s** | HIVE TIP +20 if it tips |
| 7–14 | Drive to our GARDEN, sweep slowly along the wall into the corner with the intake on. The sweep aims **2 in past** the alliance wall, so the intake really touches it; it ends once the wall has stopped the robot for 1 s (stopped more than 6 in before the wall = BLOCKED, e.g. by a robot: it parks instead) | (POLLEN for TeleOp) |
| 14–18 | At the corner the intake keeps running for up to 4 s while the robot goes **2 in back** from where the wall stopped it **and pushes in again**, so the last POLLEN rolls in (cut short only when it must park) | |
| ~18 | **PARK** in our LOADING ZONE: slot **B** (default) is parked by ~24 s, slot A by ~28 s | PARK +5 |
| (off) | **Stretch** (○ in INIT, off by default): back to the spot and fire the GARDEN POLLEN; it fires only if the camera still sees our CELL **up** at the end of the aim | another TIP? |

**Parking always wins.** At the park time (24 s, or earlier when the drive to the slot is long), the robot stops whatever it is doing and parks. Drive times are measured from the 9 Nov runs (park drive about 6 s to B, 10 s to A).

- If the robot is blocked for 1 s before parking (for example by our partner), it drops that step and goes to park.
- If it is blocked while parking, it waits and tries again until the end of AUTO.

Rules the code keeps:

- **G407 (max 4 balls):** it only goes to the GARDEN after the windmill is empty (4 balls counted, or 3 full windmill turns). Otherwise it parks.
- **G417:** it never fires into a CELL the camera sees **down** (HIVE already tipped). It looks only at our up CELL: up reads about 46 in, down about 30 in. It fires only when both the median of the last 5 frames and the latest frame are above 39 in (a latest frame reading down stops it). If the camera doesn't see our CELL at all, the pre-loads still go on the planned heading. Once firing has started, all 4 pre-loads go (team decision, 13 Nov), even if the HIVE tips during the volley.
- **Aim limit:** the camera aim turns at most 8° from the planned heading (6.5° when turning without Pedro, which coasts on). Our CELL is straight ahead at the shot spot: logged bearings were within ±1.1°. 2 in from the wall, a robot corner would touch the wall at about 15°, or about 11° if the robot is 0.5 in closer.
- **G402:** every route stays on our side of the field.
- It never touches a FLOWER or the HIVE frame. A laptop test checks every route.

## If something goes wrong

- **Pinpoint not ready at START:** AUTO puts the start pose into the Pinpoint and gives it up to 1 s to be READY there. If it still isn't, AUTO runs the **FALLBACK** on the wheel encoders and the camera:
  - **S1 with shoot on:** after the start delay, back up 2 in to the shot spot, aim with the camera (turns on the spot, at most 6.5°), check the CELL is up, fire the pre-loads, then back up to 8 in (LEAVE).
  - **S2 or shoot off:** back up 8 in (LEAVE only).
  - There is no GARDEN and no PARK in the fallback.
- **Pose lost while driving:** Pedro stops at once and never drives again in this AUTO. The pose counts as lost when:
  - it is NaN, or farther out than the robot can get (a Pinpoint that restarts reads the field corner);
  - it jumps farther between two checks than the robot can move in that time;
  - the Pinpoint is not READY for 0.25 s. On a bad read the Pinpoint driver keeps the last pose, and Pedro would drive on that frozen pose, so the motors are cut on every bad reading. One bad read is normal: the robot pauses for a moment and carries on.

  Then:
  - If the robot was still at S1 and hadn't fired yet, it runs the same FALLBACK.
  - Otherwise it makes LEAVE (team test plan: not touching a wall at 0:00) and stands still. Near the start, it backs up to 8 in. Elsewhere, it backs up until the intake is 4 in off the wall in front of it (GARDEN corner, shot spot, park slot), going by the last trusted pose.
- **Pinpoint stuck in INIT:** if it is not READY for 2 s, AUTO resets it (again every 5 s while the problem lasts). Keep the robot still.
- **AUTO right after a TeleOp, without a power cycle** (13 Nov 21:47: POSE LOST at 0.2 s, then NO PEDRO twice until a power cycle): the SDK's Pinpoint driver refuses a reading more than 5000 mm from the last one it took, and the start pose is written only to the device. After a TeleOp, which tracks in its own frame, the start pose could be refused for good. Fixed: INIT reads the Pinpoint unchecked, and START takes the first reading after the start pose unchecked and checks the pose is right.

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
- **S2 = ALLIANCE wall start:** intake against our alliance wall, centred on the field's middle line (between the FLOWER and the LOADING ZONE). It drives about 5 s to the shot spot, so the GARDEN is short or skipped.
- **Slot B** (default) = the MIDDLE end of the LOADING ZONE (toward the FLOWER). **Slot A** = the CORNER end. The robot ends 2 in off the wall with 5 in of its body inside the zone. The two slots leave a gap of about 11 in between the robots.
- **Why B is the default:** the way to A goes past B. When A is chosen, the robot slows to 30 % while it passes B, in a lane exactly halfway between the HIVE frame and a robot parked in B (3.9 in from each). The partner must park at their end of the zone, close to the wall.

## Before every match: agree with your partner

1. **Start:** only one robot can use S1.
2. **Parking:** we park in **A**, they park in **B**, or the other way round. Tell them they must park only at their end of the zone.
3. **GARDEN:** we take our GARDEN POLLEN. They should not.
4. Our robot drives on a lane beside the HIVE on our side of the field. The partner should stay out of it at the end of AUTO.

## INIT buttons (gamepad 1)

| Button | Setting |
|---|---|
| D-pad ← / → | Start S1 / S2 |
| □ / X | Park slot B (default) / A |
| ✕ / A | Shoot pre-loads on/off |
| △ / Y | GARDEN on/off |
| ○ / B | Stretch (second volley) on/off (default off) |
| D-pad ↑ / ↓ | Start delay 0–10 s |
| LB / RB | Park-by time 18–27 s (default 24) |

The screen shows the plan times and **Route check: OK**. It also shows **Pedro: ready**, the Pinpoint status, the battery, and our CELL (**UP** / **DOWN** with its height). Warnings starting with **!!** come first:

- **!! PEDRO NOT READY:** the reason, and what the FALLBACK will do (see "If something goes wrong").
- **!! BATTERY** below 12.5 V: put in a charged battery. On 13 Nov, from about 12 V, the hubs dropped out and the Pinpoint faulted.
- **!! OUR CELL looks DOWN:** AUTO will not shoot (G417). In practice, set the HIVE to the match start first.

**Team rule:** place the robot, press INIT, and don't touch it until START. The Pinpoint calibrates while the robot is still.

## Things to check on the field (first practice)

1. **Robot size:** measure from the robot centre to the intake bumper, and from the centre to the side. Set `HALF_LENGTH_IN` / `HALF_WIDTH_IN` in `auto/AutoField.java` (now 9 / 9).
2. **Shot spot:** from S1, do the 4 pre-loads score? AUTO fires at one speed, `AUTO_LAUNCH_VELOCITY` in `biobuzz/BioBuzzConstants.java`: 1440 t/s, after 1450 went slightly long on 13 Nov. If shots land short or long, change it in 10 t/s steps (one D-pad trim step). The log's `CELL ... UP` line shows the camera floor distance (about 39 in).
3. **GARDEN sweep:** does the intake catch the POLLEN against the wall? If it misses, change `GARDEN_SIDE_GAP_IN` (smaller = closer to the wall).
   - The sweep and every push at the corner aim `GARDEN_PUSH_IN` (2 in) past the wall.
   - The log line `GARDEN WIGGLE done: ... deepest x` shows how far the robot got. With the bumper on the wall it reads -61.5 (RED frame).
   - At the corner the robot backs up `GARDEN_WIGGLE_IN` (2 in; 1–2 in works) from where it stopped.
   - All three are in `auto/AutoField.java`.
4. **CELL up/down:** at the shot spot, the log column `height_in` reads about 46 for an up CELL and about 30 for a down one. The code fires only when the median of the last 5 frames is above 39 (`CellCheck.UP_MIN_HEIGHT_IN`).
5. **Parking:** run with slot A and with slot B while a partner (or a box) sits in the other slot. The robot must not touch it.
6. **Battery:** 12.5 V or more before every match.

Logs: `/sdcard/BioBuzzLogs/pedroauto_*.csv`. Steps and `EVENT` lines show each leg, each ball, any BLOCKED, AGAINST THE WALL, POSE LOST, the FALLBACK, and when PARK started. The `CONFIG` line has the battery, Pinpoint status and launcher speed at START.

## Code

| File | What |
|---|---|
| `auto/AutoField.java` | Places from the official field CAD (LOADING ZONE, GARDEN, CELL, FLOWERs), starts, shot spot, slots |
| `auto/AutoPlanner.java` | Routes, time estimates, park deadline, robot-footprint check |
| `auto/CellCheck.java` | G417 check: is our CELL up (median camera height) |
| `auto/BioBuzzPedroAutoBase.java` | The op mode (Pedro drive, camera aim, ShotFeeder fire, intake, park, FALLBACK) |
| `auto/BioBuzzPedroAutoRed/Blue.java` | RED / BLUE entries |
| `test/.../auto/AutoPlannerTest.java`, `CellCheckTest.java` | Laptop tests: legal starts, slots, every route clear, parking from anywhere, time budget, GARDEN push, CELL check |

# BioBuzz Field Tour (Pedro Pathing test)

Branch `field-tour` only. The match code on `master` is not changed.

You place the robot on any tile and tell it which tile it's on and which way it faces. It then drives:

1. a lap near the walls,
2. a lap around the HIVE frame,
3. back to where it started, facing the same way.

Like a robot vacuum, but the field map is known, so it doesn't have to build one.

## What's in it

| Part | File |
|---|---|
| Pedro Pathing 3 (`com.pedropathing:revhub:3.0.2`, `tuning:1.0.3`) | `TeamCode/build.gradle` |
| Robot settings for Pedro (motor names and directions, Pinpoint, Foresight) | `pedro/Constants.java` |
| AutoTune procedures (copied from the Pedro Quickstart) | `pedro/Tuning.java`, `pedro/procedures/` |
| Field map from the official field CAD (walls, tiles, HIVE A-frame, flowers) | `fieldtour/FieldMap.java` |
| Tour planner (laps, stays off walls and out of the HIVE frame) | `fieldtour/TourPlanner.java` + laptop tests |
| The program `BioBuzz Field Tour` (TeleOp list) | `fieldtour/BioBuzzFieldTour.java` |

## Field map

The map is seen from above, with the **RED alliance wall on the left** and the **BLUE wall on the right**.

- **Columns** 1–6 go from the RED wall to the BLUE wall.
- **Rows** 1–6 go from the right side to the left side, as seen standing at the RED alliance station.
- The **HIVE frame** (about 50 × 39 in, in the middle) is a no-go area.
  - The 4 middle tiles and the 4 tiles beside them can't be used as a start.
  - The screen marks these tiles with `#`.

## Step 1: tune Pedro (one time, about 2–4 h)

1. Install this branch on the robot. Connect the laptop to the robot Wi-Fi and open **http://192.168.43.1:10158**.
2. **Mecanum** procedure: enter `left_front_drive`, `right_front_drive`, `left_back_drive`, `right_back_drive`. Check that each wheel spins the right way.
3. **Pinpoint** procedure: name `pinpoint`. It measures the pod directions and offsets. Paste the generated `PinpointConfig` into `Constants.java`.
4. **Foresight** procedure: needs a clear area of about 6 ft. Paste the generated `ForesightConfig` into `Constants.java`.
5. **Tests**, then **Line** and **Curve**: the robot should go out and come back accurately.
6. Set `FORESIGHT_TUNED = true` in `Constants.java`, rebuild and install.

Until step 6 is done, the Field Tour only **shows** the plan and doesn't drive.

## Step 2: run the tour

1. Clear the field. Only the HIVE may be on it.
2. Put the robot in the **middle of a tile**, square to the walls.
3. Select **BioBuzz Field Tour** and press INIT.
   - Set the tile with the **D-pad**: left/right for the column, down/up for the row.
   - Press **Y / △** to choose which way the intake side faces.
   - Check the map shows your tile with an arrow and "Start: OK".
4. Press PLAY. It drives at 30 % of top speed.
   - **B / ○ stops it at once.**
   - It also stops on its own if it is blocked for 2 s ("STUCK").
5. At the end, the screen shows **From HOME: x in, heading off y°**. The target is under 2 in and 2°.

## Limits

- It can't see obstacles. The field must be clear except the HIVE.
- The camera can't find the robot's position this season, because the HIVE moves and there are no wall tags. So the start tile is entered by hand.
- Flowers aren't avoided yet. They're listed in the map; the tour only assumes there are none.

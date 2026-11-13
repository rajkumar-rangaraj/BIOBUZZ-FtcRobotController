# BioBuzz HIVE AUTO (test version)

Op modes **BioBuzz HIVE AUTO RED** and **BioBuzz HIVE AUTO BLUE** (branch `auto-hive`). **BioBuzz AUTO is unchanged**: keep using it in matches until the HIVE AUTO has worked on our field.

## The plan (team, 12 Nov)

| When (s, typical) | Step | Points |
|---|---|---|
| 0–6 | Fire the 4 pre-loads into our up CELL (it already holds 3 NECTAR) | HIVE TIP +20 |
| 6–14 | GARDEN sweep, then 1–2 s of back and forth at the corner | (4 POLLEN) |
| 14–21 | Drive **under our HIVE** to its other end, intake off | |
| 21–26 | Fire the GARDEN POLLEN into the far CELL (it came up when the HIVE tipped) | TIP 2? +20 |
| 26–29 | **PARK** in slot **A**: straight across from the far spot, facing the REAR wall | LEAVE 3 + PARK 5 |
| (3.5) | FLOWER 1 with the flower arm + intake: only when the robot is empty and there is time | |

**Parking always wins**, as in BioBuzz AUTO: at the park time the robot stops whatever it is doing and parks.

## Timeline (laptop model, fitted to the 9 Nov runs)

| Case | Pre-loads done | GARDEN done (corner time) | At the far spot | Feed time at the far CELL | Parked |
|---|---|---|---|---|---|
| Slot A, typical pre-loads | 6.1 s | 14.0 s (1.5 s) | 21.3 s | **3.3 s (~4 balls)** | 29.0 s |
| Slot A, slow pre-loads + long TIP watch | 8.8 s | 16.1 s (1.0 s) | 23.4 s | 1.2 s (~1 ball) | 29.0 s |
| Slot A, typical, BioBuzz AUTO's 4 s corner | 6.1 s | 16.5 s (4.0 s) | no time: parks | – | ~26 s |
| Slot B, typical pre-loads | 6.1 s | 13.5 s (1.0 s) | 20.8 s | 1.7 s (~2 balls) | 29.0 s |
| Slot B, slow pre-loads | 8.8 s | 16.1 s (1.0 s) | no time: parks | – | ~22 s |

**It is very tight.** It only works well with:
- **slot A**: about 3 s to park from the far spot, against 5 s to slot B.
- **a short corner time**: at least 1 s, at most 2 s (BioBuzz AUTO uses 4 s). The robot cuts it short by itself when the far CELL needs the time.
- **a fast pre-load volley**: the camera aim waits at most 1.5 s here, against 3.5 s in BioBuzz AUTO. From the wall spot the camera almost never sees the CELL (1 of 8 runs on 9 Nov), so BioBuzz AUTO mostly waited 3.5 s for nothing.

## Is it legal? (Competition Manual TU09, checked)

- **Driving under the HIVE is allowed**: G409 example D names it. In the field CAD the HIVE's lowest point is **30.6 in** above the TILES; our robot is **14.3 in** tall.
- **The HIVE frame's legs lean in.** From the CAD: a leg's inner edge is at x −23.4 on the floor and at −19.6 at 14 in up. The robot drives a lane at **x −10.4**: 1.8 in from the legs (2.8 in at wheel height) and 0.9 in from the middle line (G402: our side only).
- **G417**: launch only into an **up** CELL. See "How it stays safe" below.
- **G407**: at most 4 balls. The GARDEN only happens once the windmill is empty. After any GARDEN attempt the robot counts as "may hold balls". The intake is off under the HIVE (spilled balls may lie there) and on plan B's way back. The FLOWER only runs when a volley has emptied the robot.
- **G418**: FLOWER POLLEN may only come out of the bottom. The arm's wedge lifts the bottom POLLEN out, as with the flower test kit.

## How it stays safe (G417)

The camera watches **both CELLs of our HIVE** whenever it sees one.
- **Its verdict:** UP at 45 in or higher, DOWN below 38 in, anything in between is "not sure" and decides nothing. A verdict counts only after **3 fresh frames in a row** agree. (An UP CELL read 45.5–46.3 in on 9 Nov; a DOWN one has never been measured: the log's `height_in` column will tell.)
- **The HIVE is a seesaw:** a CELL counts as down when it reads down *or* its other end reads up.
- Near CELL confirmed down, or far CELL confirmed up = **TIPPED**.
- Near CELL still up, or far CELL still down, 1 s after the last pre-load = **NOT TIPPED**.

What the robot does:
1. **Pre-loads.** It fires all 4, as the coach asked. It stops early only if the camera *confirms* the HIVE tipped in the middle. Then the CELL is going down: more balls would break G417 and could undo the TIP. The rest go to the far CELL, and the GARDEN is skipped (G407).
2. **NOT TIPPED** → it takes the GARDEN POLLEN back to the near CELL instead, and fires only if the camera confirms that CELL up.
3. **Far CELL**:
   - Confirmed **up**: aims and fires.
   - **Down** (or the near CELL confirmed up), or seen but not sure: keeps the POLLEN.
   - Not seen at all: fires straight down the lane only if **"blind far shot"** is ON (Circle, ON by default) **and** the TIP was seen or all 4 pre-loads were counted leaving the robot. Never when NOT TIPPED was seen. With the switch OFF, every far shot needs the camera.
4. **Every volley stops at once** when the camera confirms its CELL going down. That covers TIP 2 in the middle of the far volley.
5. **After the FLOWER** (if it ever has time): it fires at the far CELL again only if the camera confirms it up.

**"Blind far shot" before matches: decide.** 4 counted launches mean 4 balls left the robot, not that they went in. If the pre-loads miss and the HIVE did not tip, the far CELL is down, and a blind far volley is a **G417 MAJOR FOUL**.
- The camera rarely sees the CELLs from the wall spots, so with the switch **OFF** the far shot will mostly not fire.
- **ON is right for testing the plan.** For matches, keep it ON only if our pre-loads tip the HIVE nearly every time.
- **Turn it OFF if our partner shoots into our HIVE in AUTO.** They could tip it back before we get there. OFF means no blind shot at all.

## Things to know

- **The far CELL starts empty.** The near CELL had 3 NECTAR to help TIP 1. 4 POLLEN may not be enough for TIP 2. If not, they stay in the CELL and help the next TIP in TeleOp.
- **TIP 1 spills the near CELL** (3 NECTAR + POLLEN) at the audience end of the HIVE, right where the robot drives in. The intake is off there, so the robot pushes them. If it gets stuck for 1 s, it gives up the step and parks.
- **Straight down the lane:** the far shot is 2.3 in right of the CELL's centre. The opening is 20 in wide. It can't go further left because of the legs.
- **FLOWER (3.5):** after the far shot there is almost never time. It does run with the GARDEN OFF (Triangle): then FLOWER 1 replaces the GARDEN as the ball source. It needs the flower arm calibrated (BioBuzz Arm Setup). The stop point is from the CAD: `FLOWER_STOP_Y`, CALIBRATE.
- In slot A the robot ends **facing the REAR wall**, which is the same footprint as BioBuzz AUTO's (it's square). TeleOp's field-centric start gets the right heading from AUTO.

## Before every match: agree with your partner

1. **Start:** we use S1 (in front of our up CELL).
2. **Parking:** we park in **A** (corner end), they park in **B**. Tell them to stay close to the wall at their end.
3. **Our HIVE:** we tip it with the pre-loads and shoot its other end at ~21–26 s. If they also shoot at our HIVE in AUTO, turn **blind far shot OFF**.
4. **Lanes:** we drive under our HIVE (middle of the field) and across the REAR end to slot A at the end of AUTO.

## INIT buttons (gamepad 1)

| Button | Setting |
|---|---|
| D-pad ← / → | Start S1 / S2 |
| Square / X | Park slot A / B (**A** is the default here) |
| Cross / A | Shoot the pre-loads ON / OFF |
| Triangle / Y | GARDEN ON / OFF (OFF + FLOWER ON = balls from FLOWER 1 instead) |
| Circle / B | **Blind far shot**: the far CELL without the camera, after a seen TIP or 4 counted pre-loads. ON / OFF (OFF = camera only) |
| R3 (right stick press) | FLOWER 1 (3.5) ON / OFF |
| D-pad ↑ / ↓ | Start delay |
| LB / RB | Park by (default **27 s**; BioBuzz AUTO 24 s) |

The INIT screen shows the plan times, the route check and the camera.

## First test on our field

1. **Install branch `auto-hive`** (it also has BioBuzz AUTO, unchanged).
2. **Dry run, no balls, HIVE in place.** Watch the lane under the HIVE: is there 1.5–2 in to the legs on our side? The robot must not touch the HIVE or go past the middle line. If it is off, tell us how far: `HIVE_LANE_X` in `HiveAutoField.java`.
3. **Full run with balls**, 3–5 times. Note:
   - Did the HIVE tip with the pre-loads?
   - How many GARDEN POLLEN came in with the short corner time?
   - Did the far shots go in? Did TIP 2 happen?
   - Did it park by 30 s?
4. **Pull the logs:** `/sdcard/BioBuzzLogs/hiveauto_*.csv`. The `tip` column and the `EVENT` lines show what it decided and why: TIP watch, "far CELL not in view: firing…", "NOT TIPPED…", FIRE done, PARK.
5. If the far CELL is often short of time, try park-by 28 (RB). If TIP 2 never happens with 4 POLLEN, the far shot still leaves them in the CELL for TeleOp.

## Code

| File | What |
|---|---|
| `auto/BioBuzzHiveAutoBase.java` | The op mode: a copy of BioBuzzPedroAutoBase with the new plan, TIP watch and FLOWER |
| `auto/HivePlanner.java` | Routes, HIVE-leg footprint check, park from anywhere, adaptive corner time |
| `auto/HiveAutoField.java` | Places from the field CAD: legs, lane, far spot, FLOWER 1 |
| `biobuzz/HiveCells.java` | Every visible CELL of our HIVE (HiveVision unchanged) |
| `HivePlannerTest.java` | 12 laptop tests: CAD leg model, lane, every route, park from 150+ points, timeline |

# BioBuzz HIVE AUTO (test version)

Op modes **BioBuzz HIVE AUTO RED** and **BioBuzz HIVE AUTO BLUE** (branch `auto-hive`). **BioBuzz AUTO is unchanged**: keep using it in matches until the HIVE AUTO has worked on our field.

## The plan (team, 12 Nov)

| When (s, typical) | Step | Points |
|---|---|---|
| 0–6 | Fire all 4 pre-loads into our up CELL (it already holds 3 NECTAR) | HIVE TIP +20 |
| 6–14 | GARDEN sweep, then 1–2 s of back and forth at the corner | (4 POLLEN) |
| 14–21 | Drive **under our HIVE** to its other end, intake off | |
| 21–26 | Camera checks the far CELL came up, then fires the GARDEN POLLEN into it | TIP 2? +20 |
| 26–29 | **PARK** in slot **A**: straight across from the far spot, facing the REAR wall | LEAVE 3 + PARK 5 |
| (3.5) | FLOWER 1 with the flower arm + intake: only when the robot is empty and there is time | |

**Parking always wins**, as in BioBuzz AUTO: at the park time the robot stops whatever it is doing and parks.

## Timeline (laptop model, fitted to the 9 Nov runs)

| Case | Pre-loads done | GARDEN done (corner time) | At the far spot | Feed time at the far CELL | Parked |
|---|---|---|---|---|---|
| Slot A, typical | 6.2 s | 13.9 s (1.3 s) | 21.2 s | **3.3 s (~4 balls)** | 29.0 s |
| Slot A, slow pre-loads | 8.8 s | 16.1 s (1.0 s) | 23.5 s | 1.0 s (~1 ball) | 29.0 s |
| Slot A, BioBuzz AUTO's 4 s corner | 6.2 s | 16.5 s (4.0 s) | no time: parks | – | ~26 s |
| Slot B, typical | 6.2 s | 13.5 s (1.0 s) | 20.9 s | 1.5 s (~2 balls) | 29.0 s |
| Slot B, slow pre-loads | 8.8 s | 16.1 s (1.0 s) | no time: parks | – | ~22 s |

"Typical" assumes a lit field where the camera sees the CELL: the aim is done in under 1 s.

**It is very tight.** It only works well with:
- **slot A**: about 3 s to park from the far spot, against 5 s to slot B.
- **a short corner time**: at least 1 s, at most 2 s (BioBuzz AUTO uses 4 s). The robot cuts it short by itself when the far CELL needs the time.
- **a quick pre-load aim**: the aim stops as soon as the robot is lined up. If the camera can't see the CELL, it waits at most 1.5 s, against 3.5 s in BioBuzz AUTO. (On 9 Nov in the dark garage the camera saw the CELL in only 1 of 8 runs; a lit field should be much better.)

## Is it legal? (Competition Manual TU09, checked)

- **Driving under the HIVE is allowed**: G409 example D names it. In the field CAD the HIVE's lowest point is **30.6 in** above the TILES; our robot is **14.3 in** tall.
- **The HIVE frame's legs lean in.** From the CAD: a leg's inner edge is at x −23.4 on the floor and at −19.6 at 14 in up. The robot drives a lane at **x −10.2**: 2.0 in from the legs (3.0 in at wheel height) and 1.2 in from the middle line (G402: our side only).
- **G417**: launch only into an **up** CELL. See "How it stays safe" below.
- **G407**: at most 4 balls. The GARDEN only happens once the windmill is empty. After any GARDEN attempt the robot counts as "may hold balls". The intake is off under the HIVE (spilled balls may lie there) and on plan B's way back. The FLOWER only runs when a volley has emptied the robot.
- **G418**: FLOWER POLLEN may only come out of the bottom. The arm's wedge lifts the bottom POLLEN out, as with the flower test kit.

## How it stays safe (G417)

**What the camera can see (field CAD).** Each CELL has its tag sticker on its bottom face.
- The HIVE tilts 30° either way, and **both stickers always face the end that is up**.
- Before TIP 1, both face the audience side: the near spot can see them, the far spot can't.
- After TIP 1, both face the REAR side: the far spot can see them, the near spot can't.
- So from the far spot in a lit field, **seeing nothing most likely means the HIVE did not tip**.

**The camera's verdict on a CELL:**
- In the CAD a sticker is 49.7 in high when UP and 35.7 in when DOWN; the camera read an UP CELL as 46.3 in.
- UP = 42 in or higher, DOWN = below 37 in, anything in between is "not sure" and decides nothing.
- A verdict counts only after **3 fresh frames in a row** agree.
- **The HIVE is a seesaw:** a CELL counts as down when it reads down *or* its other end reads up.

What the robot does:
1. **Pre-loads: all 4**, as the coach asked (the same as BioBuzz AUTO). They only stay on board if the camera confirms, before the first ball, that the HIVE already tipped (for example, our partner tipped it). Then they go to the far CELL.
2. **TIP watch** at the near spot, up to 1.2 s after the last pre-load:
   - The near CELL is still up, or the far CELL is still down, after 1 s = **NOT TIPPED**. The robot takes the GARDEN POLLEN back to the near CELL instead, and fires only if the camera confirms it up.
   - The stickers turn away = probably tipped: on to the far CELL.
3. **Far CELL** (the far spot gives the camera the same view as the near spot):
   - Camera confirms it **up** (or confirms the near CELL down from there): aims and fires.
   - **Down**, or seen but not sure: keeps the POLLEN for TeleOp.
   - **Nothing in view: keeps the POLLEN**, unless **"blind far shot"** is ON (Circle, **OFF by default**). With it ON, the robot fires anyway after 4 counted pre-loads.
     - Only turn it ON when the camera can't be trusted (bad light).
     - 4 counted launches don't prove a TIP. If they missed, a blind far volley is a **G417 MAJOR FOUL**.
4. **The far volley stops at once** if the far CELL drops out of view or reads down (TIP 2 turns its sticker away). The rest stay on board for TeleOp.
5. **After the FLOWER** (if it ever has time): it fires at the far CELL again only if the camera confirms it up.

## The robot as built vs the planner

The planner treats the robot as an 18 × 18 in box centred on Pedro's pose. The real one (CAD) is not centred:
- **Pedro's centre** is the drivetrain centre. From there the **intake front is 9.76 in**, the **launcher back 8.03 in**, and the **camera 4.63 in back** (12.2 in high, tilted up 30°).
- Starting with the intake against the audience wall, the real robot is **0.76 in further toward the REAR wall** than Pedro thinks, all AUTO long.

The far spot, FLOWER stop and rear slot use these numbers:
- **Far spot:** the intake ends 2 in off the REAR wall, and the camera is 41.3 in from the far sticker (near spot: 41.2 in).
- **FLOWER stop:** set from the same offsets.
- **Rear slot A:** 4.9 in inside the zone.

## Things to know

- **The far CELL starts empty.** The near CELL had 3 NECTAR to help TIP 1. 4 POLLEN may not be enough for TIP 2. If not, they stay in the CELL and help the next TIP in TeleOp.
- **TIP 1 spills the near CELL** (3 NECTAR + POLLEN) at the audience end of the HIVE, right where the robot drives in. The intake is off there, so the robot pushes them. If it gets stuck for 1 s, it gives up the step and parks.
- **Straight down the lane:** the far shot is 2.5 in right of the CELL's centre. The opening is 20 in wide. It can't go further left because of the legs and FLOWER 1.
- **FLOWER (3.5):** after the far shot there is almost never time. It does run with the GARDEN OFF (Triangle): then FLOWER 1 replaces the GARDEN as the ball source. It needs the flower arm calibrated (BioBuzz Arm Setup). The stop point is from the CAD: `FLOWER_STOP_Y`, CALIBRATE.
- In slot A the robot ends **facing the REAR wall**. TeleOp's field-centric start gets the right heading from AUTO.

## Before every match: agree with your partner

1. **Start:** we use S1 (in front of our up CELL).
2. **Parking:** we park in **A** (corner end), they park in **B**. Tell them to stay close to the wall at their end.
3. **Our HIVE:** we tip it with the pre-loads and shoot its other end at ~21–26 s. Ask them not to shoot our HIVE in AUTO.
4. **Lanes:** we drive under our HIVE (middle of the field) and across the REAR end to slot A at the end of AUTO.

## INIT buttons (gamepad 1)

| Button | Setting |
|---|---|
| D-pad ← / → | Start S1 / S2 |
| Square / X | Park slot A / B (**A** is the default here) |
| Cross / A | Shoot the pre-loads ON / OFF |
| Triangle / Y | GARDEN ON / OFF (OFF + FLOWER ON = balls from FLOWER 1 instead) |
| Circle / B | **Blind far shot**: fire at the far CELL without the camera after 4 counted pre-loads. OFF (default) / ON |
| R3 (right stick press) | FLOWER 1 (3.5) ON / OFF |
| D-pad ↑ / ↓ | Start delay |
| LB / RB | Park by (default **27 s**; BioBuzz AUTO 24 s) |

The INIT screen shows the plan times, the route check and the camera.

## First test on our field (good light, please)

1. **Install branch `auto-hive`** (it also has BioBuzz AUTO, unchanged).
2. **Dry run, no balls, HIVE in place.** Watch the lane under the HIVE: is there 1.5–2 in to the legs on our side? The robot must not touch the HIVE or go past the middle line. At the far spot, check the intake is about 2 in off the REAR wall and clear of FLOWER 1. If something is off, tell us how far: `HIVE_LANE_X` / `FAR_SHOT_Y` in `HiveAutoField.java`.
3. **Full run with balls**, 3–5 times. Note:
   - Did the HIVE tip with the pre-loads?
   - How many GARDEN POLLEN came in with the short corner time?
   - Did the far shots go in? Did TIP 2 happen?
   - Did it park by 30 s?
4. **Pull the logs:** `/sdcard/BioBuzzLogs/hiveauto_*.csv`.
   - The `EVENT` lines show what it decided and why: "TIP after the pre-loads", "FAR CELL … UP", "far CELL not in view…", "NOT TIPPED…", FIRE done, PARK.
   - The `height_in` column shows what the camera read: UP should be about 46, DOWN about 32 (bands 42 / 37).
5. If the far CELL is often short of time, try park-by 28 (RB). If TIP 2 never happens with 4 POLLEN, the far shot still leaves them in the CELL for TeleOp.

## Code

| File | What |
|---|---|
| `auto/BioBuzzHiveAutoBase.java` | The op mode: a copy of BioBuzzPedroAutoBase with the new plan, CELL tracking and FLOWER |
| `auto/HivePlanner.java` | Routes, HIVE-leg and FLOWER footprint checks, park from anywhere, adaptive corner time |
| `auto/HiveAutoField.java` | Places from the field CAD (legs, lane, far spot, tag stickers, FLOWERs) and the robot as built |
| `biobuzz/HiveCells.java` | Every visible CELL of our HIVE (HiveVision unchanged) |
| `HivePlannerTest.java` | 15 laptop tests: CAD leg model, lane, camera view at the far spot, every route, park from 150+ points, timeline |

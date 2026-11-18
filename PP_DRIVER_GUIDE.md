# BioBuzz TeleOp PP – Driver Guide

The new TeleOp. It knows where the robot is on the field, so it **aims by itself** and **fires all your balls with one tap**.

If anything feels wrong, switch back to the old **BioBuzz TeleOp**. Nothing in it has changed.

---

## 1. Pick a start spot

Before INIT, put the robot in **one** of these places. It's easiest to use **CELL wall**.

```
              REAR wall
     +---------------------------+
     |                           |
 RED |                           | BLUE
 wall|  ALLIANCE      [HIVE]     | wall
     |   wall                    |
     |  (start)                  |
     |                           |
     |          [CELL wall start]|
     +---------------------------+
            AUDIENCE wall
         (the spectators' side)
```
*(The picture is for RED. For BLUE, turn everything around: use the REAR wall instead of the AUDIENCE wall.)*

| Start | Where to put the robot |
|---|---|
| **CELL wall** ⭐ | **Intake touching the audience wall** (BLUE: the rear wall), **right in front of our CELL that is UP**. The camera looks straight at that CELL. It's the same spot as the AUTO start "S1". |
| **ALLIANCE wall** | **Intake touching our own driver-station wall**, in the **middle of that wall**, between the FLOWER and the LOADING ZONE. It's the same spot as the AUTO start "S2". |
| **CAMERA** | **Anywhere**, but the **intake must point AWAY from you**, straight toward the far wall. The robot finds itself the first time the camera sees our CELL. |
| **AUTO** | Only after AUTO has run. Don't move the robot; just INIT the TeleOp. |

---

## 2. INIT: set it up (robot must stay still)

| Press | What it does |
|---|---|
| **B (Circle)** | RED alliance |
| **A (Cross)** | BLUE alliance |
| **D-pad LEFT / RIGHT** | Choose the start spot from step 1 |
| **Back (Share)** | Turn the log ON. **Always ON in practice.** |

Check the screen:

- **Alliance** is correct
- **Start** matches where the robot really is
- **Pinpoint: ok** (if it says *calibrating*, wait and **don't touch the robot**)

Then press **START**.

---

## 3. Driving

| Button | What it does |
|---|---|
| **Left stick** | Drive |
| **Right stick** | Turn. **Always wins** over the auto aim. |
| **LB (hold)** | Slow, careful driving |
| **Right trigger / Left trigger** | Intake IN / OUT |
| **RB – TAP once** 🔥 | **VOLLEY:** turns to the CELL and fires **every ball**. Tap again to stop. |
| **X (hold)** | Aim only, no firing |
| **B (hold)** | Unjam the windmill, and stop the volley |
| **Y** | Pre-spin on/off. Normally leave it ON; the flywheel is ready when you get there. |
| **A** | Flower arm out / in |
| **D-pad UP / DOWN** | Far / near shot speed (only if the auto speed is wrong) |
| **Options (tap)** | "The intake points away from me **now**". Use it after a big crash if driving feels wrong. |

### How to shoot
1. Drive to your shooting spot (**30–50 in from the CELL**).
2. **Stop moving.**
3. **TAP RB once. Don't hold it.**
4. Wait. The controller buzzes when the volley starts, and buzzes twice when the robot is empty.
5. Drive away and get more balls.

**Don't press RB again for each ball.** One tap fires them all. Pressing again **stops** the volley.

---

## 4. What the screen tells you

| Screen says | Meaning | What to do |
|---|---|---|
| **PINPOINT + tags** | ✅ Everything works | Nothing |
| **camera aim (position not known yet)** | Robot doesn't know where it is yet | Point the **back of the robot (camera) at our CELL** for a moment |
| **CAMERA ONLY** | Pinpoint failed. It still aims, using the camera. | Keep playing; tell the programmers after the match |
| **BALL HELD: aim … deg off** | Waiting to be aimed | Wait; it fires by itself |
| **BALL HELD: driver turning** | You are turning | Let go of the right stick |
| **BALL HELD: camera says …** | The camera and the robot disagree | Wait 2 s; it fires anyway if it's close |

---

## 5. If something goes wrong

| Problem | Fix |
|---|---|
| Robot aims at the wrong place | Tap **Options** with the intake pointing away from you, then show the camera our CELL |
| Balls stuck | Hold **B**, then tap **RB** again |
| Nothing fires | Look at the screen line **BALL HELD** and do what it says |
| Still bad | **Switch to the old BioBuzz TeleOp** |

---

## 6. Practice checklist (first time only)

Tick each one:

- [ ] **Start: CELL wall.** Drive around, come back to the start spot. The screen numbers (x, y) are the same as at the start.
- [ ] **Start: CAMERA.** Show the camera our CELL. The screen changes to **PINPOINT + tags**.
- [ ] **Hold X** while the CELL is **behind you** (camera can't see it). The robot turns to it by itself.
- [ ] **Hold X**, and someone **pushes** the robot. It turns back.
- [ ] **3 balls, tap RB once.** All 3 go, and it buzzes twice at the end.
- [ ] **Shoot 40 balls with the old TeleOp and 40 with the new one**, from the same spots. A helper writes down IN / SHORT / LONG / SIDE for each ball.
- [ ] After practice, **pull the logs** (`.\tools\pull_robot_logs.ps1`) and send them to the programmers.

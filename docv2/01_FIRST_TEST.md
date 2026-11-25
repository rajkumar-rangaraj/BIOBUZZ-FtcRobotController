# 01 – First test: v2 Hardware Test

**Goal:** prove that every motor, servo, sensor and the camera is wired, named and working **on its own**, before any
v2 robot code drives them together. Nothing here changes the robot's settings; you only write down what you see.

**Time:** about 45 minutes. **People:** 2 (one on the Driver Hub, one watching the robot).

---

## Part A – Install the v2 code (once)

1. Open the folder **`C:\repo\FtcRobotController-v2`** in Android Studio. (This is the `v2` branch. The v1 code is in
   `C:\repo\FtcRobotController` – a different folder.)
2. Check the branch at the bottom right of Android Studio says **v2**.
3. Connect the laptop to the **Control Hub's Wi-Fi**.
4. Choose **Run ▶ 'TeamCode'**. Wait for "Install successfully finished".
5. **Disconnect the laptop from the hub's Wi-Fi** (otherwise the Driver Hub cannot connect).

## Part B – Make the v2 configuration on the Driver Hub (once)

Keep the v1 configuration: make a **new** one so you can always go back.

1. Driver Hub: **⋮ menu → Configure Robot → New**. Scan. Name the configuration **`v2`**.
2. Enter the devices **exactly** as below (names are case-sensitive; a typo shows as MISSING in the test).

### Control Hub

| Port | Device | Configuration type | Name |
|---|---|---|---|
| Motor 0 | Front-left drive wheel | goBILDA 5202/3/4 series | `left_front_drive` |
| Motor 1 | Back-left drive wheel | goBILDA 5202/3/4 series | `left_back_drive` |
| Motor 2 | Flywheel motor 1 | goBILDA 5202/3/4 series | `launcher` |
| Motor 3 | Flywheel motor 2 | goBILDA 5202/3/4 series | `launcher2` |
| Servo 0 | Left corner intake wheel | Continuous Rotation Servo | `left_intake_servo` |
| Servo (free) | Gate servo (switched to **servo mode**) | Servo | `gate` |
| Digital (free) | Ball-1 beam receiver | Digital Device | `beam_staged` |
| Digital (free) | Ball-4 beam receiver | Digital Device | `beam_full` |
| I2C bus 0 | Built-in IMU (already there) | REV internal IMU | `imu` |
| I2C port 0 | Pinpoint (optional) | goBILDA Pinpoint Odometry Computer | `pinpoint` |
| USB | Logitech C270 | Webcam | `Webcam 1` |

### Expansion Hub

| Port | Device | Configuration type | Name |
|---|---|---|---|
| Motor 0 | Front-right drive wheel | goBILDA 5202/3/4 series | `right_front_drive` |
| Motor 1 | Back-right drive wheel | goBILDA 5202/3/4 series | `right_back_drive` |
| Motor 2 | Intake roller | goBILDA 5202/3/4 series | `intake` |
| Motor 3 | Queue motor (belts + Geckos; the old windmill motor) | goBILDA 5202/3/4 series | `indexer` |
| Servo 0 | Right corner intake wheel | Continuous Rotation Servo | `right_intake_servo` |
| Servo 1 | Flower arm (servo mode) | Servo | `flower_arm` |

The beams and the gate can go on any free port of either hub. **Every motor's encoder cable goes in the encoder port
with the same number as its power port.**

3. **Save**, then **Activate** the `v2` configuration (saving is not activating).

> Not everything is built yet? Fine. Configure what is plugged in; the rest will show **MISSING** in the test and the
> test still runs.

## Part C – Safety before pressing PLAY

- [ ] Robot **on a box**, all four wheels free.
- [ ] **No balls** in the robot.
- [ ] Hands, cables and tools away from the **flywheel** and the **gate**.
- [ ] Battery charged (the test shows the voltage; above 12.5 V is good).
- [ ] The person watching the robot can reach the **STOP** button on the Driver Hub, or says "stop" loudly.

## Part D – Run "v2 Hardware Test"

1. Driver Hub: select **v2 Hardware Test** (group v2) → **INIT**.
2. Read the INIT screen:
   - **Hubs: 2 (Control + Expansion)**. If it says 1: check the RS485 cable and the Expansion Hub power.
   - **Missing from the configuration: none**. Anything listed is a name typo or not plugged in – fix it before
     going on (or write it down if that part is not built yet).
   - No **CODE ERROR** lines.
3. Press **PLAY**. Nothing moves until you hold a trigger or press A.

### Controls (gamepad 1)

| Button | Does |
|---|---|
| D-pad LEFT / RIGHT | Change page: MOTORS → SERVOS → SENSORS → CAMERA |
| D-pad UP / DOWN | Pick a device (`>>` marks it) |
| Right trigger / left trigger | Run the picked motor or corner servo forward / backward (release = stop) |
| B | Zero the picked motor's encoder (only when it is not powered) |
| RB / LB | Gate or arm: move the target +/- 0.01 (hold **Y** for 0.05 steps) |
| A | Gate or arm: **send** the target (the servo only moves after A). SENSORS page: zero the IMU yaw |
| X | Gate or arm: let go (servo goes limp) |
| Y | SENSORS page: zero the Pinpoint (keep the robot still) |

After picking another device you must **release the triggers once** before it responds ("RELEASE the triggers first").

### D1 – Safety checks first (in this order)

| # | Do | Must see | OK? |
|---|---|---|---|
| 1 | MOTORS, pick `launcher`, squeeze the right trigger gently | **Both** `launcher` and `launcher2` show speed (t/s). If one stays at 0, fix its encoder cable **before** steps 2–4 | ☐ |
| 2 | Hold the right trigger, then release | Power reads **0.00 at once**; the disc coasts down | ☐ |
| 3 | Spin it up, release, press the **left** trigger straight away | **"WAIT for the flywheel to stop…"** until it slows, then it ramps the other way | ☐ |
| 4 | Spin `launcher` up, release, pick `launcher2`, press either trigger while it coasts | **WAIT** | ☐ |
| 5 | CAMERA page, then leave it | Camera **STREAMING** on the page; stops when you leave; comes back when you return. Page through it quickly a few times: it ends up stopped | ☐ |
| 6 | SERVOS, pick `gate`, press **A** once | The gate goes to 0.50 **without a twitch** toward an end | ☐ |

### D2 – Every device (fill in the sheet in Part E)

**MOTORS page** – for each motor: hold the right trigger a little.
- The **right part** moves (the wheel you expect, the queue belts, the intake roller, the flywheel).
- Note if the encoder counts **UP** or **DOWN** for the right trigger.
- Note which way it moves for the right trigger (wheel: forward/backward; queue: balls toward the flywheel or back;
  intake: pulls in or pushes out).
- Note the current (A) when it turns freely. A drive wheel over ~2 A free = something rubs.

**SERVOS page**
- Corner wheels (`left_intake_servo`, `right_intake_servo`): right trigger – does it pull a ball **IN**?
- **Gate** (`gate`): the **drop gate** (since 25 Nov): its two prongs slide straight down. Press A (goes to 0.50),
  then RB / LB in small steps. Find and write down:
  - **CLOSED**: prong tops level with the middle of a ball (they hold ball 1); the servo arm points up the lane.
  - **OPEN**: prongs 45 mm lower, their tops below the bottom belts. About 90° of servo turn from CLOSED.
  - Don't drive the servo past either end: the arm's pin hits the end of the carriage slot. Stop at once if it binds.
  - With a ball resting at the gate, opening must **not** push the ball back.
- **Flower arm** (`flower_arm`): same way, write down **STOWED** and **DEPLOYED**.
- X lets a servo go limp when you are done.

**SENSORS page**
- Each beam: reading with **no ball** and **with a ball** held in the beam. They must be different.
  (Adafruit 2167: LOW = beam broken. Always LOW with no ball = check the emitter power and the wiring first.)
- IMU: press A (zero), turn the robot **left** by hand – yaw must go **up**. Robot flat: pitch and roll near 0.
- Pinpoint (if fitted): press Y (zero, robot still), push the robot forward – one pod counts; sideways – the other.

**CAMERA page**
- Shows **STREAMING** at about 30 fps. Hold an AprilTag in front of the lens: its id shows with a range.
- Picture: Driver Hub menu → **Camera Stream**.

4. Press **STOP** when done.

## Part E – Results sheet (print this, or copy it into the team notebook)

Date: ____________ Tested by: ____________ Battery: ______ V

**Motors** (right trigger = + power)

| Motor | Right part moves? | Moves which way? | Encoder UP / DOWN | Free current (A) | Notes |
|---|---|---|---|---|---|
| left_front_drive | ☐ | fwd / back | | | |
| left_back_drive | ☐ | fwd / back | | | |
| right_front_drive | ☐ | fwd / back | | | |
| right_back_drive | ☐ | fwd / back | | | |
| launcher | ☐ | shoots out / back | | | |
| launcher2 | ☐ | shoots out / back | | | |
| indexer (queue) | ☐ | balls up / down | | | |
| intake | ☐ | pulls in / pushes out | | | |

**Flywheel pair:** spinning `launcher` alone, does `launcher2`'s encoder count the **same** way as `launcher`'s?
☐ same ☐ opposite

**Servos**

| Servo | Result |
|---|---|
| left_intake_servo | + power pulls IN? ☐ yes ☐ no |
| right_intake_servo | + power pulls IN? ☐ yes ☐ no |
| gate | CLOSED = ______ OPEN = ______ (drop gate: prongs 45 mm down) |
| flower_arm | STOWED = ______ DEPLOYED = ______ |

**Sensors**

| Sensor | No ball | With ball |
|---|---|---|
| beam_staged | HIGH / LOW | HIGH / LOW |
| beam_full | HIGH / LOW | HIGH / LOW |

| Sensor | Result |
|---|---|
| IMU | Turn left → yaw goes up? ☐ yes. Flat: pitch ____ roll ____ |
| Pinpoint | Forward pod counts? ☐ Sideways pod counts? ☐ (or not fitted) |
| Camera | STREAMING ☐ fps ____ tag id seen ____ |
| Hubs | 2 found ☐ |

**Safety checks D1:** 1 ☐ 2 ☐ 3 ☐ 4 ☐ 5 ☐ 6 ☐

Problems found / fixed:

______________________________________________________________________________

## When something is wrong

| You see | Likely cause | Fix |
|---|---|---|
| A device listed as MISSING | Name typo, wrong type, or not plugged in | Compare with the tables in Part B (exact spelling) |
| A motor does not move, current 0 | Power cable / wrong port | Check the cable and the port number on the screen |
| It moves but the encoder stays 0 | Encoder cable missing or in the wrong port | Encoder port number = power port number |
| The **wrong** motor's encoder counts | Encoder cables swapped | Swap them back |
| Encoder counts DOWN for + power | Normal – the motor or encoder is mounted the other way | Just write it down; v2 Setup sets the direction |
| Only 1 hub | Expansion Hub not connected | RS485 cable, power cable, hub address |
| Camera not STREAMING | USB cable, or the camera not in the configuration | Re-plug, check `Webcam 1` |
| A beam never changes | Emitter not powered or not aimed | Point emitter at receiver; check 3.3–5 V to the emitter |

## Going back to v1 (e.g. for a match)

1. Open `C:\repo\FtcRobotController`, check out the tag **`biobuzz-v1`**, **Run ▶ 'TeamCode'**.
2. Driver Hub: **Configure Robot** → select the **v1** configuration → **Activate**.

## Done when

- [ ] All six safety checks in D1 pass.
- [ ] Every built device has a line filled in on the results sheet.
- [ ] The sheet is kept for **02 – v2 Setup**.

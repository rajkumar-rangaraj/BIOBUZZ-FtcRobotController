# v2 shooting plan (for review): fast and accurate, built on Pedro Pathing

Status: **plan only, no code yet** (21 Nov 2026). Branch `v2`, folder `C:\repo\FtcRobotController-v2`.
This document is written so a reviewer who has not seen our chat can check it. Section 17 lists the questions we want
a second opinion on. Numbers marked **(est.)** are calculated, not measured; section 15 says how each one gets measured.

---

## 1. The robot and the game (what matters for shooting)

- **Game pieces:** POLLEN balls, about Ø72 mm (2.8 in), about 25 g. They are launched into our alliance's **HIVE CELL**.
  The HIVE tips, so a CELL can face **up** (it can score) or **down**.
- **Rule G417:** never launch into a downward CELL. Our code must **fail safe**: if it doesn't know, it doesn't fire.
- **Indexer v2** (new this season, CAD in goBildaStarter `indexer-v2`):
  - a 40° single-file ball lane (the **queue**: belts plus Gecko wheels, one motor, `indexer`);
  - a **gate** servo paddle at the end of the lane. It opens +90° toward the intake; turning it the other way hits the hood;
  - two break beams: `beam_staged`, with a ball waiting at the gate, and `beam_full`, with the lane full. The robot
    holds up to 4 balls; the 5th is pushed back (G407 push-back from the `indexer-v2` branch);
  - the **flywheel**: a 96 mm Hogback wheel with a Ø73 × 6.35 mm steel disc (**ordered, not fitted yet**), on a shaft
    driven by **two goBILDA 5203 6000 rpm motors** (`launcher`, `launcher2`, 28 ticks/rev, 1:1), belted together;
  - the printed hood: 74° exit, ball leaves 9 in above the floor. The ball rolls between the tread and the hood, so it
    leaves at about half the tread speed;
  - the camera (`Webcam 1`) on the camera bridge above the launcher, looking at the CELL.
- **Drive:** mecanum, goBILDA Pinpoint odometry (4-bar pods), **Pedro Pathing 3.0.2** (`com.pedropathing:revhub`) with
  the AutoTune page at `http://192.168.43.1:10158`. Pedro 3's API is new; don't use Pedro 1.x/2.x examples.
- **Field frame used by v1 (`auto/AutoField`, `fieldtour/FieldMap`):** origin at the field centre, +X toward the BLUE
  wall, +Y toward the rear wall, heading in degrees CCW from +X. Everything is written for RED; BLUE is the 180° turn
  about the centre. **Pedro's frame is different:** origin at the bottom-left corner of a 144 in field, radians.
  The conversion goes in one place and gets its own tests (section 7.2).

## 2. What went wrong in v1 (measured, from logs and notes)

| Problem | v1 numbers | Source |
|---|---|---|
| Slow AUTO volley | camera aim up to 3.5 s + fixed 0.5 s settle + per pocket (0.2 s turn + 0.35 s pause + 0.1 s ready hold), about 2.7 pockets per ball launched → about 1.5–2 s per ball | `BioBuzzConstants`, 7 Nov logs |
| Aim | at the shot the camera bearing had a median of **−11°** (1 Nov, 5 % of balls scored) | MEMORY §6 |
| Flywheel dip | one wheel, I = 1.66e-4 kg·m²: the speed fell **16.7 / 20.5 / 24.3 %** (p10 / median / p90) per ball | `FLYWHEEL_PHYSICS.md` |
| Recovery | 0.6 s at P 40; better at P 100 | `BioBuzzConstants` |
| Speed spread vs scoring window | the dip spread alone gives about ±4.4 % ball speed; the window is only **±4.2–4.7 %** → about 60–70 % IN at best | `FLYWHEEL_PHYSICS.md` |
| Feed while the ball is grabbed | balls taken while the windmill was still pushing: 33 % IN; after it stopped: 59 % (p = 0.09) | 10 Nov labels |
| Spin-up | flywheel + motor time constant about 1.3 s | `BioBuzzConstants` |

What worked and is kept: tag-cluster CELL detection (`HiveVision`, SDK 12 `AprilTagClusterDetection`), the height-based
CELL UP check (`CellCheck`), labelled shots plus `tools/shot_results.py` for the speed table, and the AUTO with a fixed
shot pose (3 of 4 balls launched, the first scored).

## 3. Targets

| Target | Value | Measured how |
|---|---|---|
| **Volley time** (arrive at the shot pose → last of 4 balls out) | **≤ 1.2 s** (v1: about 5–8 s) | log timestamps |
| **Time between balls** | ≤ 0.30 s | log |
| **Accuracy** | **≥ 80 % IN** from every shot pose | labelled shots |
| **Never fire into a down CELL** | 0 shots | log + video |
| **Path end error** | ≤ 1.0 in, ≤ 1.0° | Path Check (section 7.5) |

Time budget for one volley (est.):

| Phase | Time | Why it is short |
|---|---|---|
| Spin-up | 0 s on the clock | starts at the beginning of the path (section 4.4) |
| Aim | 0 s on the clock | the path's end heading already points at the CELL (section 8) |
| CELL check | 0 s on the clock | done during the last part of the path (section 9) |
| Settle | ≤ 0.2 s | measured stillness, not a fixed 0.5 s (section 7.4) |
| Ball 1 | ~0.1 s | gate open + travel to the wheel (measure) |
| Balls 2–4 | 3 × 0.25–0.30 s | recovery-limited (section 4.3) |
| **Total** | **≈ 1.0–1.2 s** | |

---

## 4. Flywheel physics (v2)

### 4.1 Inputs

| Quantity | Value | Note |
|---|---|---|
| Wheel | Hogback 96 mm, tread radius r = 0.048 m | |
| Inertia, v1 (one motor, no disc) | 1.66e-4 kg·m² | CAD: wheel 1.31e-4, hub 0.05e-4, rotor ~0.3e-4 |
| + steel disc | +1.50e-4 | Ø73 × 6.35 mm, 161 g (182 g with screws) |
| + second motor rotor, pulleys, belt | ~+0.35e-4 (est.) | **measure** (section 15) |
| **Inertia, v2** | **I ≈ 3.5e-4 kg·m² (est., ±15 %)** | about 2.1× v1 |
| Motor (each) | free 6000 rpm (ω_f = 628 rad/s), stall torque τ_s ≈ 0.144 N·m at 12 V | goBILDA 5203 1:1 |
| Ball | m = 0.025 kg, thin shell (c = 2/3) | |
| Ball speed factor | η = 0.71 with one wheel in v1 (fitted) | **v2 must be re-fitted** |

### 4.2 Speeds and energy at the usual setpoint (1400 t/s)

- ω = 1400 ÷ 28 × 2π = **314 rad/s** (3000 rpm, half of free speed)
- tread speed = 50 rev/s × π × 0.096 = **15.1 m/s**
- ball speed = η × tread ÷ 2 = 0.71 × 7.54 = **5.35 m/s** (v1 needed 5.52 m/s at about 36 in)
- stored energy = ½ I ω² = **17 J** (v1: 8.8 J)
- ball energy out, translation + spin = ½mv² + ⅓mv² ≈ 0.6 J, about 3.5 % of stored energy. The capture loses about as
  much again, which is why the dip is bigger than an energy balance says.

### 4.3 The dip and the recovery

**Dip.** The sticking-impulse model gives ω_after = ω ÷ (1 + (1 + c) m r² ÷ (4 I)).
- v2: (5/3 × 0.025 × 0.048²) ÷ (4 × 3.5e-4) = 0.069, so the model dip is **6.4 %**.
- v1 measured 20.5 % where the model said 12.6 %, a ratio of 1.63. Scaled the same way, **v2 ≈ 10–11 % ≈ 145 t/s**.
- Cross-check with the inertia ratio α = 2.1: D' = D ÷ (α(1 − D) + D) = 0.205 ÷ 1.88 = **10.9 %**. ✔
- Expected dip spread (p10–p90): about 9.5–14.4 %, so about ±2.5 % ball speed (v1: ±4.4 %). The speed-only IN rate goes
  from about 60–70 % to **about 84–90 %** (est.).

**Recovery after a ball** (both motors at full power).
- At ω = ω_f/2 the motor torque at full power is τ_s(1 − ω/ω_f). Holding the speed needs a power of u_hold, about
  0.55 (est.; it's the measured F term). The net torque is n · τ_s · (1 − u_hold) = 2 × 0.144 × 0.45 ≈ **0.13 N·m**.
- Δω = 0.109 × 314 = 34 rad/s, so t = I Δω ÷ τ = 3.5e-4 × 34 ÷ 0.13 = **0.09 s ideal**.
- v1's measured time constant was about 1.8× the ideal one (battery sag under load, friction, the hub controller), so
  plan on **about 0.16 s**. v1 by the same method: 0.16 s ideal, 0.29 s derated.
- So the time between balls is set by recovery (about 0.16 s) plus the time for the next ball to reach the wheel.
  **0.25–0.30 s per ball** is the plan.

**Spin-up from rest** (full power).
- τ = I ω_f ÷ (n τ_s) = 3.5e-4 × 628 ÷ 0.288 = 0.76 s ideal, about 1.4 s derated. To reach ω_f/2: τ ln 2 = **0.5 s
  ideal, about 1.0 s real (est.)**; on a tired battery up to about 1.4 s.
- Conclusion: **the flywheel is never started at the shot pose.** In AUTO it runs from START. In TeleOp it runs while
  we hold a ball (section 12).

**Balance (safety).** Each 0.1 mm of disc eccentricity is m e ω² = 0.182 × 1e-4 × 314² = **1.8 N** of shaking at
1400 t/s. Centre the disc, and do the hand-turn test before powering it.

### 4.4 What the physics means for the design

1. **One controller, both motors at the same power.** Two hub velocity loops (RUN_USING_ENCODER) on two belted motors
   fight each other. v2 runs both in RUN_WITHOUT_ENCODER and computes one power in our loop.
2. **Feedforward + P + full power when low.**
   `u = kV · ω_target · (12 / V_batt) + kS + kP · (ω_target − ω)`, clipped to [0, 1]; full power (1.0) while
   ω < ω_target − band_hi.
   - kV and kS come from the characterization OpMode (section 15).
   - Battery voltage is read every 0.5 s (it costs a hub command) and filtered.
   - Power never goes negative while shooting: no braking. Coming down to a lower setpoint is a coast.
3. **Velocity measurement.** The hub reports about 20 t/s steps (seen in v1 logs), which is as big as our ±20 t/s band.
   - Average both encoders. They are belted 1:1, so the sign is fixed by the calibrated directions.
   - Also compute a position-difference velocity over ≥ 40 ms.
   - Use the smoother of the two for READY, and the raw one for dip detection.
   - Open question 17.3.
4. **Encoder health.** The two encoder speeds disagree by more than 10 % for more than 0.2 s while spinning → FLYWHEEL
   FAULT: no firing, and a message on screen. This is the same idea as `pairStopped()` in v2 Hardware Test.
5. **READY band:** ±1.5 % of target (±21 t/s at 1400), held for 2 consecutive filtered samples (≈ 40 ms), not v1's
   fixed 0.1 s. The scoring window is ±4.2 %; the band leaves room for the dip spread and the set-speed error.

---

## 5. Feed: queue and gate cadence

- **Port from `indexer-v2`:** QueueFeeder (ticks per ball, G407 push-back), the gate back-off, IntakeGuard and the FULL
  latch, with their unit tests. Calibrated values come from V2Calibration (`queue.ticksPerBall`, `gate.open/closed`,
  beams, jam current).
- **Volley state machine** (pure Java, fake clock, unit-tested):
  `IDLE → ARMED (flywheel READY, ShotGate true) → OPEN_GATE → FEED_ONE (queue advances one ball) → WAIT_DIP (a dip ≥
  AUTO_SHOT_DIP_TPS = ball left) → WAIT_READY → FEED_ONE … → EMPTY (no staged beam, no dip within a timeout) → CLOSE_GATE → IDLE`.
- **Rule for releasing each ball, phase 1 (safe):** feed ball k+1 only when the flywheel is READY again and ShotGate is
  still true. That alone should give 0.25–0.30 s per ball (section 4.3).
- **Phase 2 (later, only if phase 1 is accurate):** start ball k+1 early by the measured gate-to-wheel travel time, so
  it reaches the wheel just as the speed comes back into the band. Use it only if labelled shots show no loss.
- **Does the queue push the ball into the wheel, or stop just before?** v1 showed that a ball grabbed while the feeder
  was still pushing did worse (33 % vs 59 %, p = 0.09). Test both: A = the queue stops once the staged ball has been
  taken (beam clears); B = the queue keeps running at a lower power. Decide from 40 labelled shots each. Open question 17.4.
- **Faults:**
  - queue stall (no ticks for 0.15 s at power) → back off ¼ ball and retry, up to 3 times, then JAMMED;
  - a ball that doesn't leave (no dip within 0.6 s of feeding) → count it as a miss-feed and retry once;
  - FLYWHEEL FAULT, or the CELL not UP → stop at once and close the gate.

## 6. Shot gate: every ball must pass all of these

| # | Condition | Value |
|---|---|---|
| 1 | Flywheel READY | within ±1.5 % of target, 2 samples |
| 2 | No flywheel fault | encoders agree within 10 % |
| 3 | Robot still | Pedro not busy, speed < 2 in/s, turn < 5°/s |
| 4 | Heading on target | \|heading − aim heading\| ≤ 1.5° (computed per pose, section 8) |
| 5 | **Our CELL is UP** | section 9; UNKNOWN = NO |
| 6 | Pose trusted | Pinpoint READY, and camera vs odometry within 2° when the camera sees the CELL |
| 7 | Feed OK | not JAMMED, a ball staged |
| 8 | Legal | in a launch area if the manual requires one; **to confirm in the Competition Manual** |

Each failure has a short reason code (`NOT_READY`, `CELL_DOWN`, `CELL_UNKNOWN`, `MOVING`, `AIM`, `POSE`, `JAM`, …)
shown on telemetry and logged. That way a slow volley can be explained afterwards.

---

## 7. Pedro Pathing: paths we can trust

### 7.1 Re-tune for v2
The robot is heavier with indexer v2, so the 4 Nov Foresight values in `pedro/Constants.java` are out of date.
1. Check the drive directions and the pod offsets (tape measure versus the AutoTune Pinpoint procedure). The pods may
   not have moved; check anyway.
2. Run AutoTune in order: Mecanum → Pinpoint → **Foresight** → Tests. Paste the new values into a **v2 copy**
   (`v2/V2Pedro.java`), so v1's Constants stay untouched.
3. `FORESIGHT_TUNED`-style guard: v2 paths refuse to run until the v2 values are pasted in.

### 7.2 One field file: `V2Field`
- Every pose lives in **one** class: start poses (CELL wall, alliance wall), shot poses, GARDEN pickup, LOADING ZONE
  slots A/B, park. RED is written out; BLUE = `mirror()` (180° about the centre).
- Poses are stored in v1's centre frame (field CAD numbers, matching AutoField), with **one** conversion to Pedro:
  `x_p = x + 72`, `y_p = y + 72`, `heading_p = radians(heading)`. Unit tests check:
  - every pose converts both ways;
  - the mirror of a mirror is the same pose;
  - known CAD points land where expected;
  - no pose is outside the field or overlaps a FLOWER or the HIVE frame (robot half-size 9 in, plus a margin).
- The CELL centre positions for aiming also live in V2Field (from the field CAD; v1: RED AUDIENCE CELL x −12.7,
  y −1.6…−20.1).

### 7.3 Paths
- Built once at INIT. No path is created during a match loop.
- **The end heading points at the CELL**, computed as atan2 from the end pose to the CELL centre plus the launcher's
  direction on the robot (launcher at the back).
- Heading interpolation: linear to the end heading over the last part of the path. The robot arrives already aimed.
- Speed limits per path segment. Slower on the last segment before a shot pose, so it settles fast.

### 7.4 "Settled" instead of a fixed wait
Settled = `!follower.isBusy()` and translational speed < 2 in/s and turn < 5°/s for 2 loops. Pedro keeps holding the
end pose (hold point) while we shoot. Target ≤ 0.2 s; v1 always waited 0.5 s.

### 7.5 Path Check OpMode (acceptance before any shooting relies on Pedro)
- Runs each AUTO path **10 times** from a taped start mark. It logs the end pose, the settle time and the peak
  following error, and a person checks the end position against tape marks.
- **Pass:** end error ≤ 1.0 in and ≤ 1.0°, settle ≤ 0.3 s, 10/10 runs. After a full AUTO route the Pinpoint drift is
  ≤ 2 in (tape).
- **Independent check:** at each shot pose the camera's CELL bearing (and range) is compared with what odometry
  predicts. They should agree within 2° and 2 in.
- **Watchdog** (also used in matches): following error > 6 in, or Pedro busy with speed ≈ 0 for 0.5 s (a robot or the
  wall is in the way) → stop the path, don't fire, go to the fallback (section 11).
- Optional: dry-run the routes in Beta8397/virtual_robot before field time.

## 8. Aim: odometry first, camera as the check

- **Aim heading** = bearing from the robot pose to the CELL centre (V2Field) + the measured launcher trim.
- **Camera check:** when our CELL is in view, compare the camera bearing, corrected for the camera-to-launcher offset,
  with the odometry bearing:
  - ≤ 2°: shoot on the odometry heading;
  - \> 2°: correct the heading to the camera's bearing (one correction per volley) and log `AIM_CORRECTED`;
  - \> 6°: don't fire (`POSE`). Odometry or the camera mount is wrong, and someone must look.
- **Calibrate the camera mount on v2:** the camera-to-launcher offset (x, y, yaw) from CAD, then checked by shooting
  10 balls from a known pose. The v1 −11° problem was a mount or trim problem; v2 measures it explicitly.
- **Lens calibration** for the v2 webcam at the chosen resolution, so range and height are right (the CELL check
  depends on height).
- **Heading tolerance:** v1 used 1.5°. The real limit comes from the CELL opening width, the ball size and the
  distance. Compute it per shot pose from the field CAD and store it with the pose in V2Field.

## 9. Camera: is our CELL UP?

- **Port `auto/CellCheck`:**
  - it uses the camera **height** of the CELL cluster centre; v1 at the S1 spot read UP 46.1–46.3 in and DOWN
    29.3–32.1 in, with the threshold at 39 in;
  - the verdict is the median of the last 5 frames, **and** the latest frame must also be UP (a latest DOWN vetoes);
  - **re-measure both heights for the v2 camera position** before trusting the threshold.
- **Three states:** UP, DOWN, UNKNOWN. UNKNOWN covers no frame, a frame older than 0.25 s, a cluster that is not our
  CELL, or a CELL further than the "front" distance (v1 `FRONT_MAX_FLOOR_IN` = 47). **Only UP fires.**
- **Second cue (to evaluate):** the cluster's orientation. The tags are on the CELL's bottom; a tipped CELL turns them,
  so the pose pitch should differ a lot between UP and DOWN. If logs show a clean split, require height **and**
  orientation to agree. Open question 17.6.
- **Timing:** the camera runs during the last part of every shot path, so the verdict is ready on arrival. It is
  checked again **before each ball**: if the CELL tips mid-volley, the rest of the volley stops.
- **Camera settings:** a resolution and decimation that keep ≥ 25 fps, low fixed exposure and gain so tags stay sharp
  while turning, and only the tag processor running.
- **No CELL override in AUTO.** In TeleOp: open question 17.7.

## 10. Shot speed table

- **Input:** distance from **odometry** (pose to CELL opening, horizontal), which is steadier than the camera. The
  camera range is logged as a cross-check.
- v2 changes the ball-speed factor (two motors, a stiffer flywheel, a new hood interaction), so **the v1 table is not
  reused**. The v1 table (1400 t/s at 30–40 in) is only the starting guess.
- **Fit:** shoot from the V2Field shot poses first, then 4 distances in 32–44 in, 20 labelled shots each
  (IN / SHORT / LONG / SIDE), and fit with `tools/shot_results.py`. **Only after the disc is fitted.**
- Measure the real ball exit speed with 240 fps phone video against a ruler, at 3 setpoints. That re-fits η and checks
  the model's scoring window.
- **Battery:** shots are labelled with battery voltage. If accuracy drops below about 12.0 V, the feedforward scaling
  is wrong; the table is not changed.

## 11. AUTO sequence

1. **INIT:** choose alliance, start, slot; build the paths; camera on; check Pinpoint READY and the v2 Pedro values.
2. **START (t = 0):** flywheel to the shot speed; path to shot pose 1 (from the CELL-wall start this is the 2 in
   move-off). CELL check during the last part of the path.
3. **Settled** → volley (section 5), up to 4 balls, ShotGate before each.
4. Path to the GARDEN with the intake on; queue loading (G407 push-back).
5. Path back to a shot pose, spinning up on the way → volley 2.
6. **Park** before the deadline (v1: 20 s, a value set at INIT).
- **Fallbacks:**
  - CELL DOWN or UNKNOWN at the shot pose → wait there up to the time limit, then skip to the next step and keep the
    balls;
  - Pedro watchdog fires → stop, then park if possible;
  - flywheel FAULT → don't shoot, collect and park;
  - without Pedro or Pinpoint READY → v1-style LEAVE on wheel encoders.

## 12. TeleOp assisted shooting (both modes; the driver picks)

- **Mode A, "go to shot":** a button starts a Pedro path from the current pose to the nearest legal V2Field shot pose.
  Any stick movement cancels it. On arrival the volley runs while the trigger is held.
- **Mode B, "heading lock":** the driver translates with the stick, and the robot turns to the aim heading from
  odometry (camera-checked). The flywheel speed follows the odometry distance, and while the trigger is held each ball
  fires when ShotGate is true. Moving shots are not allowed in phase 1: ShotGate needs the robot still. Moving shots
  are a later phase.
- **Pre-spin:** the flywheel spins at the table speed whenever we hold ≥ 1 ball, plus a driver toggle. Measure the
  battery cost (hold power ≈ u_hold at 1400 t/s) in section 15.
- Telemetry and rumble: READY, CELL UP/DOWN/UNKNOWN, and the first failing ShotGate reason.

## 13. Logging

- **Per loop (≤ 20 Hz to file):** pose, follower state, flywheel ω for both encoders, power, battery, CELL state and
  height, camera bearing and range, gate, queue ticks, beams, ShotGate reason.
- **Per shot event:** speed before, dip minimum and time, recovery time, odometry distance, camera distance, heading
  error, CELL state, time since the volley started, and a label (driver LB + D-pad as in v1).
- CSV in `/sdcard/BioBuzzLogs`, pulled with `tools/pull_robot_logs.ps1`; an AdvantageScope converter later.

## 14. Code layout (all in `teamcode.v2`, no v1 imports)

| Class | Kind | Tests |
|---|---|---|
| `FlywheelController` (FF + P + bang, READY band, fault) | pure Java | plant simulation: inertia, motor curve, injected dips, battery sag, a dead encoder |
| `FlywheelModel` (the plant used by the tests) | pure Java | spin-up and recovery match section 4 |
| `VolleyStateMachine` | pure Java, fake clock | every transition, every fault, CELL tipping mid-volley |
| `ShotGate` | pure Java | truth table, reason codes |
| `CellCheck` (v2) | pure Java | v1 tests + UNKNOWN / stale frames / second cue |
| `V2Field` + `PedroFrame` | pure Java | conversion, mirror, poses legal, aim headings |
| `ShotTable` | pure Java | interpolation, clamping, hysteresis |
| `V2Shooter` (hardware wrapper) | OpMode-side | System Check |
| `V2FlywheelCharacterize`, `V2PathCheck`, `V2AimTest` | OpModes | robot procedures, docv2 |

## 15. Test plan

| Stage | What | Pass |
|---|---|---|
| Unit (laptop) | all pure classes above | all green, in CI with `testDebugUnitTest` |
| Bench, no disc | **Flywheel characterization:** kS and kV ramp; spin-up; hold power at 1200/1400/1600; dip and recovery with hand-fed balls; both encoders; logged at about 100 Hz | the controller holds ±1.5 %; recovery is measured; the model is updated |
| Bench, with disc | the same again + measure the real inertia (coast-down from 1600 t/s with the motors unpowered) | recovery ≤ 0.2 s; spin-up time recorded |
| Feed | volleys of 4 into the flywheel, robot on the box: time between balls, double feeds, jams; A vs B (section 5) | ≤ 0.30 s per ball, 0 double feeds in 20 volleys |
| Pedro | v2 AutoTune, then Path Check (section 7.5) | 10/10 within 1 in / 1° |
| Aim | 20 balls from each shot pose, with the camera vs odometry check | ≥ 90 % within the CELL width; median bearing error ≤ 1° |
| CELL | 20 UP and 20 DOWN checks at each shot pose, both alliances | 40/40 right; DOWN is never called UP |
| Speed table | section 10 | ≥ 80 % IN at each distance |
| AUTO | 10 full runs | volley ≤ 1.2 s, ≥ 80 % IN, park every time |
| TeleOp | drivers, timed, both modes | the drivers pick the default mode |

## 16. Order of work

| # | Step | Needs |
|---|---|---|
| 1 | v2 Hardware Test, V2Config, V2Calibration | **done** |
| 2 | `V2Hardware` | 1 |
| 3 | v2 Setup (calibration OpMode) + `02_SETUP.md` | 2 |
| 4 | v2 System Check + `03_SYSTEM_CHECK.md` | 3 |
| 5 | FlywheelModel + FlywheelController + characterization OpMode | 2 (bench without the disc now, **again with the disc**) |
| 6 | Port the queue/gate/intake logic + VolleyStateMachine + ShotGate | 3, 5 |
| 7 | v2 Pedro re-tune, V2Field/PedroFrame, Path Check | 2 |
| 8 | CellCheck v2, aim (odometry + camera check), camera/lens calibration | 7 |
| 9 | Shot speed table | **disc fitted**, 5–8 |
| 10 | AUTO sequence | 6–9 |
| 11 | TeleOp modes A and B | 6–9 |
| 12 | Logs → AdvantageScope | any time after 6 |
| later | Shooting while moving (lead the CELL by robot speed × flight time) | all of the above proven |

Each step: unit tests → external review → robot doc in `docv2` → robot test → commit.

## 17. Questions for the reviewer

1. **The physics in section 4:** is the derating (×1.8 from v1's measured time constant) a fair way to predict v2's
   recovery? Is there a better simple model of the capture than the sticking-impulse one?
2. **Our own flywheel loop vs the hub's:** is RUN_WITHOUT_ENCODER with one power for both motors right for a belted
   pair, or would one motor in velocity mode with the other copying its power be better?
3. **Velocity resolution:** the hub's 20 t/s steps are about the size of the ±21 t/s band. Is averaging both encoders
   plus a ≥ 40 ms position difference good enough, or should the band be wider?
4. **Feed into the wheel:** should the queue stop as soon as the wheel takes the ball (A) or keep pushing gently (B)?
   Any physical reason to expect a difference?
5. **Pedro 3:** is "end heading = aim heading" with linear heading interpolation the right way to arrive aimed, and is
   `!isBusy()` plus speed thresholds a sound settle test with Foresight? Is a 1 in / 1° acceptance realistic?
6. **CELL check:** is the cluster orientation a reliable second cue for UP vs DOWN, or is height alone enough?
7. **TeleOp CELL override:** G417 says never launch into a down CELL. Should the driver have an override button (they
   can see the CELL) when the camera fails, or must the robot always refuse? Our default: **no override**.
8. **Anything missing** that would make a volley slow or inaccurate: battery sag across a match, the ball hitting the
   hood edge, ball-to-ball spacing in the lane, or camera latency during the end of a path.

## 18. Decisions already made (team, 21 Nov)

- **Stop and shoot first;** shooting while moving comes later.
- TeleOp: **both** assist modes; the driver picks.
- Camera: **checks the odometry aim** and corrects it when they differ by > 2°.
- Steel disc: **ordered**. The speed table waits for it; the controller is characterized again once it is fitted.
- v1 code stays untouched (tag `biobuzz-v1`); v2 never imports v1 classes.

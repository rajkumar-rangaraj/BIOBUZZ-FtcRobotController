# v2 shooting plan (for review): fast and accurate, built on Pedro Pathing

Status: **plan only, no code yet**, revision 4 (21 Nov 2026). Branch `v2`, folder `C:\repo\FtcRobotController-v2`.
This document is written so a reviewer who has not seen our chat can check it. Numbers marked **(est.)** are calculated,
not measured; section 15 says how each one gets measured. Section 17 lists what is still open.

### Changes in revision 2 (after the first review)

| # | Change | Sections |
|---|---|---|
| R1 | The speed reading: a 40 ms position difference does **not** beat the hub's 20 t/s steps. Now: the ticks of both encoders summed over 80 ms, and the READY band is set only after the noise is measured | 4.4, 6, 15 |
| R2 | Feed: phase 1 **requires** option A (the queue stops when the wheel has the ball). Option B lets ball 2 follow ball 1 into the wheel during recovery, which bypasses READY. B moves to phase 2 | 5 |
| R3 | The gate servo time is in the budget (0.15–0.25 s for 90°, est.), and the gate opens **during the end of the path**, before settle | 3, 5 |
| R4 | Settle: `!isBusy()` is not proof of arrival (a Pedro path can end on a timeout). The pose and heading errors are added. The aim heading is computed from the **settled pose**, not the planned end pose | 6, 7.4, 8 |
| R5 | The camera correction rule is defined once; ShotGate row 6 no longer contradicts section 8 | 6, 8 |
| R6 | The Pedro frame conversion is checked against an independent source (Pedro's visualizer and a pose read on a field mark), not only against its own formula | 7.2 |
| R7 | START current: two motors spinning up from rest draw close to stall current while the drive accelerates. The lowest battery voltage is logged, with a stagger or a cap if it dips | 4.3, 11, 13 |
| R8 | Smaller: encoder-health check has an absolute floor; all timings are in seconds, not loop counts; CELL orientation is a veto only; CELL frames count only once the robot is slow; Path Check pass is 9/10 with no run over 1.5 in | 4.4, 7.5, 9 |

### Changes in revision 3 (second review + fixes to revision 2)

| # | Change | Sections |
|---|---|---|
| S1 | **Feed stop rule fixed:** the queue no longer stops when `beam_staged` clears (ball 1 still has ~40 mm to the wheel then). It stops on the dip **or** after one ball pitch, whichever is first. **Replaced by T1** (revision 4) | 5 |
| S2 | **Mode A conflict fixed:** AUTO paths are built at INIT; a TeleOp mode A path is built when the button is pressed. The whole path's footprint is checked, not only its end pose | 7.3 |
| S3 | **Inertia measurement fixed:** coast-down alone can't separate inertia from friction. Powered acceleration with current + coast-down at the same speed | 4.3, 15 |
| S4 | READY must be **settled** (slope limit), not just passing through the band. The ball-left dip threshold is re-calibrated for v2, and a ball only counts as gone when the dip **recovers** (a jam also makes a dip) | 4.4, 5 |
| S5 | Section 1 geometry facts answered from CAD: lane uphill, gate servo model and speed, paddle sweep (the check is ball **1**, not ball 2) | 1, 5, 15 |
| S6 | **HIVE tipping:** our own volley can tip the HIVE, and 2–3 balls can be in the air. New TIPPING state; capture-to-stop delay measured; find what triggers a tip | 9, 13, 14, 17 |
| S7 | Targets as milestones (≤ 2.4 / 1.6 / 1.2 s), **accuracy before speed**, ≥ 100-shot accuracy test | 3, 15 |
| S8 | G407: the intake stops at 4 balls (a 5th is not collected); push-back is only the backup | 1, 11 |
| S9 | Aim and shot distance from the **ball release point**, not the robot centre | 8, 10 |
| S10 | TeleOp: mode B first; pose at TeleOp start (from AUTO, or re-zero on a field mark); pre-spin is a setting | 12 |
| S11 | Disc safety steps; full-match electrical endurance test; park deadline from measured travel time | 4.3, 11, 15 |
| S12 | Disc inertia hand check (ring with the Ø32 bore ≈ 1.4e-4 with screws; a solid-disc 1.07e-4 does not apply). The frame "rotation" worry is noted as unlikely (v1's Pedro AUTO ran with the plain +72 shift), but the independent check stays | 4.1, 7.2 |

### Changes in revision 4

| # | Change | Sections |
|---|---|---|
| T1 | **Feed rule (replaces S1):** FEED_ONE moves **exactly one pitch** and stops. Stopping on the dip left ball 2 only ~40–45 mm along its 73 mm pitch, short of the staged spot, so ShotGate row 7 failed and the volley stalled after ball 1; and the dip is only seen 20–40 ms after the wheel has the ball, so it never prevented pushing during capture anyway. Position keeps ball k+1 out of the wheel; the dip only **confirms** the shot. A vs B becomes a phase 2 question | 5, 14, 15 |
| T2 | **Cadence budget:** the READY logic's own delays (80 ms reading lag, 40 ms hold, slope test) and ball k+1's 40 mm travel are added to the time per ball. Phase 1: **0.30–0.40 s per ball** (fits the 1.6 s milestone); 0.25–0.30 s is the phase 2 target. The simulation reports time between balls with the full READY logic | 3, 4.3, 14 |
| T3 | **READY slope limit** is set from the same noise measurement as the band; 1 % per 100 ms is only a starting value | 4.4 |
| T4 | **Phase 2 needs no Pedro:** it shoots from a hand-placed robot on a taped pose. Step 8 needs V2Field/PedroFrame and the camera/lens calibration, not the Pedro re-tune or Path Check | 16 |
| T5 | **TIPPING false alarms:** threshold from the height noise of a still CELL, and the change must show on **2 consecutive unique frames**. New test: false TIPPING events with nothing moving, pass = 0 | 9, 15 |
| T6 | **Mode B heading control:** the two options are named, with a recommended default; the choice is open question 17.2.9 | 12, 17 |

---

## 1. The robot and the game (what matters for shooting)

- **Game pieces:** POLLEN balls, about Ø72 mm (2.8 in), about 25 g. They are launched into our alliance's **HIVE CELL**.
  The HIVE tips, so a CELL can face **up** (it can score) or **down**.
- **Rule G417:** never launch into a downward CELL. Our code must **fail safe**: if it doesn't know, it doesn't fire.
- **Indexer v2** (new this season, CAD in goBildaStarter `indexer-v2`):
  - a 40° single-file ball lane (the **queue**: belts plus Gecko wheels, one motor, `indexer`);
  - a **gate** servo paddle at the end of the lane. It opens +90° toward the intake; turning it the other way hits the hood;
  - two break beams: `beam_staged`, with a ball waiting at the gate, and `beam_full`, with the lane full. The robot
    holds up to 4 balls (G407). The intake stops as soon as `beam_full` shows 4, so a 5th ball is not collected; the
    push-back from the `indexer-v2` branch is only the backup (S8);
  - the **flywheel**: a 96 mm Hogback wheel with a Ø73 × 6.35 mm steel disc (**ordered, not fitted yet**), on a shaft
    driven by **two goBILDA 5203 6000 rpm motors** (`launcher`, `launcher2`, 28 ticks/rev, 1:1), belted together;
  - the printed hood: 74° exit, ball leaves 9 in above the floor. The ball rolls between the tread and the hood, so it
    leaves at about half the tread speed;
  - the camera (`Webcam 1`) on the camera bridge above the launcher, looking at the CELL.
- **Geometry facts for section 5** (R2; answered from the CAD in revision 3, S5):
  - **The lane runs uphill** toward the wheel (40°; the belts and Geckos drive the balls up). When the queue stops,
    gravity holds ball 2 back.
  - **Where ball 2 stops:** the queue moves one ball pitch per ball (`queue.ticksPerBall`), so after ball 1 is taken,
    ball 2 sits at the staged spot behind the gate, about 40 mm short of the wheel. Confirm on the robot (section 15).
  - **The paddle** (two prongs between the bottom belts, 3 mm in front of ball 1) opens by tipping toward the intake
    and under the lane floor, through the swing slots. It never reaches ball 2. The check is **ball 1**: the prong tips
    must not push it back or lift it while they swing (section 15).
  - **Gate servo:** goBILDA 2000-0025-0003 (Speed), about 0.09 s per 60° rated (6 V, no load), so **about 0.15–0.2 s
    for 90°** on the hub's servo power (est.; measure it).
- **Drive:** mecanum, goBILDA Pinpoint odometry (4-bar pods), **Pedro Pathing 3.0.2** (`com.pedropathing:revhub`) with
  the AutoTune page at `http://192.168.43.1:10158`. Pedro 3's API is new; don't use Pedro 1.x/2.x examples.
- **Field frame used by v1 (`auto/AutoField`, `fieldtour/FieldMap`):** origin at the field centre, +X toward the BLUE
  wall, +Y toward the rear wall, heading in degrees CCW from +X. Everything is written for RED; BLUE is the 180° turn
  about the centre. **Pedro's frame is different:** origin at the bottom-left corner of a 144 in field, radians. v1's
  Pedro AUTO (`BioBuzzPedroAutoBase`) ran on the field with a plain `+72` shift, so a rotation is unlikely, but it
  is still checked independently (R6, S12). The conversion goes in one place and gets its own tests (7.2).

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
| **Volley time** (arrive at the shot pose → last of 4 balls out) | milestones (S7): **≤ 2.4 s** first working, **≤ 1.6 s** competition-ready, **≤ 1.2 s** stretch (v1: about 5–8 s) | log timestamps |
| **Time between balls** | **0.30–0.40 s** phase 1; ≤ 0.30 s phase 2 / stretch (T2) | log |
| **Accuracy** | **≥ 80 % IN** first working, **≥ 85 %** competition-ready, over **≥ 100 shots** (several batteries, balls and shot poses) | labelled shots |
| **Never fire into a down CELL** | 0 shots | log + video |
| **Path end error** | ≤ 1.0 in, ≤ 1.0° | Path Check (section 7.5) |

**Accuracy comes before speed.** A milestone's volley time only counts once its accuracy target is met; the volley is
made faster only after that. A robot that reliably scores 4 in 2 s beats one that sometimes scores 4 in 1.2 s.

**Phase 1 volley (est., T2):** settle ≤ 0.2 s + ball 1 ~0.1 s + 3 × **0.30–0.40 s** ≈ **1.2–1.5 s**, inside the
1.6 s competition-ready milestone. The time per ball includes the READY logic's own delays (section 4.3).

Time budget for the stretch volley (phase 2, est.):

| Phase | Time | Why it is short |
|---|---|---|
| Spin-up | 0 s on the clock | starts at the beginning of the path (section 4.3) |
| Aim | 0 s on the clock | the path's end heading already points at the CELL; a small re-aim from the settled pose happens during settle (section 8) |
| CELL check | 0 s on the clock | done during the last part of the path (section 9) |
| Gate open | 0 s on the clock | opened during the end of the path; the queue holds the balls (section 5, R3) |
| Settle | ≤ 0.2 s | measured stillness and pose error, not a fixed 0.5 s (section 7.4) |
| Ball 1 | ~0.1 s | queue travel from the gate to the wheel (measure) |
| Balls 2–4 | 3 × 0.25–0.30 s | phase 2: the next pitch starts before READY, timed so the ball arrives as the speed returns (sections 4.3, 5) |
| **Total** | **≈ 1.0–1.2 s** | |

If the gate cannot open early (for example, the opening paddle disturbs ball 1), add the gate time
(0.15–0.25 s, est.) to ball 1 and the budget becomes **≈ 1.2–1.4 s**. Measure it in the feed test (section 15).

---

## 4. Flywheel physics (v2)

### 4.1 Inputs

| Quantity | Value | Note |
|---|---|---|
| Wheel | Hogback 96 mm, tread radius r = 0.048 m | |
| Inertia, v1 (one motor, no disc) | 1.66e-4 kg·m² | CAD: wheel 1.31e-4, hub 0.05e-4, rotor ~0.3e-4 |
| + steel disc | +1.50e-4 (CAD study) | Ø73 × 6.35 mm, 161 g (182 g with screws). Hand check (S12): a ring ½m(R² + r²) = ½ × 0.161 × (0.0365² + 0.016²) = 1.28e-4, + 8 screws at r 24 mm ≈ 0.12e-4 → **≈ 1.4e-4**. (A solid disc gives 1.07e-4, but the Ø32 bore moves the same mass outward.) Inside the ±15 % below; the bench measurement decides |
| + second motor rotor, pulleys, belt | ~+0.35e-4 (est.) | **measure** (section 15) |
| **Inertia, v2** | **I ≈ 3.5e-4 kg·m² (est., ±15 %)** | about 2.1× v1 |
| Motor (each) | free 6000 rpm (ω_f = 628 rad/s), stall torque τ_s ≈ 0.144 N·m, stall current ≈ 9.2 A at 12 V | goBILDA 5203 1:1 |
| Ball | m = 0.025 kg, thin shell (c = 2/3) | |
| Ball speed factor | η = 0.71 with one wheel in v1 (fitted) | **v2 must be re-fitted** |

### 4.2 Speeds and energy at the usual setpoint (1400 t/s)

- ω = 1400 ÷ 28 × 2π = **314 rad/s** (3000 rpm, half of free speed)
- tread speed = 50 rev/s × π × 0.096 = **15.1 m/s**
- ball speed = η × tread ÷ 2 = 0.71 × 7.54 = **5.35 m/s** (v1 needed 5.52 m/s at about 36 in)
- stored energy = ½ I ω² = **17 J** (v1: 8.8 J)
- ball energy out, translation + spin = ½mv² + ⅓mv² ≈ 0.6 J, about 3.5 % of stored energy.

### 4.3 The dip and the recovery

**Dip.** The sticking-impulse model gives ω_after = ω ÷ (1 + (1 + c) m r² ÷ (4 I)).
- v2: (5/3 × 0.025 × 0.048²) ÷ (4 × 3.5e-4) = 0.069, so the model dip is **6.4 %**.
- v1 measured 20.5 % where the model said 12.6 %, a ratio of 1.63. Scaled the same way, **v2 ≈ 10–11 % ≈ 145 t/s**.
- Why the scaling is trustworthy even though the model is not: v1's measured dip means about three times the ball's
  energy is lost per shot (hood rubbing, ball squash), not the one-for-one loss of the sticking model. That loss is set
  by the ball and the hood, not by the wheel, so it is roughly the same energy per shot. For small dips that makes the
  dip scale with **1/I** whatever the mechanism. The α-formula D' = D ÷ (α(1 − D) + D) = 0.205 ÷ 1.88 = **10.9 %** is
  the same 1/I scaling written another way, **not an independent check**; the bench test is the check.
- Expected dip spread (p10–p90): about 9.5–14.4 %, so about ±2.5 % ball speed (v1: ±4.4 %). The speed-only IN rate goes
  from about 60–70 % to **about 84–90 %** (est.).

**Recovery after a ball** (both motors at full power).
- At ω = ω_f/2 the motor torque at full power is τ_s(1 − ω/ω_f). Holding the speed needs a power of u_hold, about
  0.55 (est.; it's the measured F term). The net torque is n · τ_s · (1 − u_hold) = 2 × 0.144 × 0.45 ≈ **0.13 N·m**.
- Δω = 0.109 × 314 = 34 rad/s, so t = I Δω ÷ τ = 3.5e-4 × 34 ÷ 0.13 = **0.09 s ideal**.
- v1's measured time constant was about 1.8× the ideal one (battery sag under load, friction, the hub controller).
  Expect v2 to derate **a little more**: two motors draw twice the recovery current from the same battery, so the sag
  part grows. Plan on **0.16–0.20 s**.
- **Time between balls, phase 1 (T2)**, ball k taken → ball k+1 taken, all est.:

  | Part | Time |
  |---|---|
  | Recovery to the band | 0.16–0.20 s |
  | READY reading lag (80 ms window) | ~0.04 s |
  | Hold in the band | 0.04 s |
  | Slope test (settled) | usually inside the hold; more if the speed overshoots |
  | Ball k+1's ~40 mm to the tread | ~0.07 s at full queue power, ~0.17 s at 0.4 |
  | **Total** | **0.30–0.40 s** (more at low queue power) |

  In phase 1 the next pitch only starts after READY, so these add up. **0.25–0.30 s per ball is the phase 2 target**,
  where the next pitch starts early (section 5). The `FlywheelController` simulation must report the time between
  balls **with the full READY logic** (reading window, hold, slope), not just the recovery time.

**Spin-up from rest** (full power).
- τ = I ω_f ÷ (n τ_s) = 3.5e-4 × 628 ÷ 0.288 = 0.76 s ideal, about 1.4 s derated. To reach ω_f/2: τ ln 2 = **0.5 s
  ideal, about 1.0 s real (est.)**; on a tired battery up to about 1.4 s.
- Conclusion: **the flywheel is never started at the shot pose.** In AUTO it runs from START. In TeleOp it runs while
  we hold a ball (section 12).
- **Current at START (R7):** from rest the two motors draw close to stall current, about **18 A together** (est.), at the
  same moment the drivetrain accelerates off the start. Log the lowest battery voltage in the first 0.5 s of AUTO
  (section 13). If it sags enough to brown out or slow the drive, start the flywheel 0.2 s after the path, or cap its
  power at 0.7 for the first 0.3 s. Either costs well under 0.1 s of spin-up, because the path to shot pose 1 is longer
  than the spin-up.

**Holding speed is cheap.** At a steady 1400 t/s the motors only overcome friction: about (u_hold − ω/ω_f) × 9.2 A
≈ 0.5 A each (est.). Pre-spinning whenever we hold a ball (section 12) costs little battery. Confirm it in the
characterization (section 15).

**Safety (S11).** Each 0.1 mm of disc eccentricity is m e ω² = 0.182 × 1e-4 × 314² = **1.8 N** of shaking at
1400 t/s. Centre the disc and do the hand-turn test before powering it. The disc needs secure retention (8 screws +
washers), the guard must cover it, and the first runs step the speed up gradually (600 → 1000 → 1400 t/s) with
everyone clear of the plane of the wheel. 17 J is a lot of energy to come loose.

**Measuring the inertia (S3).** Coast-down alone can't do it: the deceleration depends on friction as well as inertia.
At the same speed, measure the acceleration under power α_up with the total motor current i, and the deceleration while
coasting α_coast (motors unpowered). Then I = k_t · i ÷ (α_up + |α_coast|), with k_t ≈ 0.144 ÷ 9.2 = 0.0157 N·m/A
per motor (spec; check it with a short stall test). Repeat at 3 speeds; the answers should agree within about 10 %.

### 4.4 What the physics means for the design

1. **One controller, both motors at the same power.** Two hub velocity loops (RUN_USING_ENCODER) on two belted motors
   fight each other. "One motor in velocity mode, the other copying its power" does not work either: `getPower()`
   returns the commanded value, not what the hub's loop applies. v2 runs both in RUN_WITHOUT_ENCODER and computes one
   power in our loop.
2. **Feedforward + P + full power when low.**
   `u = kV · ω_target · (12 / V_batt) + kS + kP · (ω_target − ω)`, clipped to [0, 1]; full power (1.0) while
   ω < ω_target − band_hi.
   - kV and kS come from the characterization OpMode (section 15).
   - Battery voltage is read every 0.5 s (it costs a hub command) and filtered. During recovery the motors are at full
     power anyway, so the slow voltage read only matters while holding.
   - Power never goes negative while shooting: no braking. Coming down to a lower setpoint is a coast.
3. **Velocity measurement (R1).** With 28 ticks/rev, a position difference over a window Δt can only resolve
   1 tick ÷ Δt: 25 t/s at 40 ms, which is no better than the hub's 20 t/s steps. Both are about the size of a ±1.5 %
   band (±21 t/s at 1400). So:
   - **READY speed** = the ticks of **both** encoders summed over an **80 ms** window (directions from V2Calibration),
     divided by 2 × the window: about **6 t/s (0.45 %)** resolution, at the cost of about 40 ms of lag. That is
     acceptable against a 0.16–0.20 s recovery.
   - **Dip detection** uses the hub's raw velocity on both motors (a 145 t/s dip is 7 steps; resolution is not the
     problem there, speed of response is).
   - **The READY band is set from the measured noise**, not chosen now: the characterization logs the READY speed at a
     steady 1200/1400/1600 t/s. Band = max(±1.5 %, 3 × the measured standard deviation). If that comes out above about
     ±2.5 %, fit a REV Through Bore encoder on the flywheel shaft and use only its **position** difference (check
     first: the hub's own velocity value may overflow at those count rates; position does not).
4. **Encoder health.** The two encoder speeds disagree by **more than 10 % and more than 60 t/s**, for more than
   0.2 s, while the target is above 0 → FLYWHEEL FAULT: no firing, and a message on screen. The absolute floor stops
   false faults near zero speed. This is the same idea as `pairStopped()` in v2 Hardware Test.
5. **READY:** the READY speed inside the band (point 3) **for 40 ms** (a time, not a sample count: the loop time
   changes when Pedro and the camera run), **and settled (S4):** the READY speed's slope over the last 80 ms is below a
   limit, so a speed still rising through the band after a dip doesn't count. **The limit is set from the same noise
   measurement as the band (T3).** 1 % of target per 100 ms is only a starting value: at 6 t/s resolution it is about
   2 steps per 80 ms window, so noise alone could fail it. The scoring window is ±4.2 %; the band leaves room for the
   dip spread and the set-speed error.

---

## 5. Feed: queue and gate cadence

- **Port from `indexer-v2`:** QueueFeeder (ticks per ball, G407 push-back), the gate back-off, IntakeGuard and the FULL
  latch, with their unit tests. Calibrated values come from V2Calibration (`queue.ticksPerBall`, `gate.open/closed`,
  beams, jam current).
- **Gate opens early (R3):** the gate opens when the robot is on the last segment of a shot path **and** the CELL is UP
  **and** the flywheel is READY or within 0.3 s of it. Opening the gate launches nothing as long as the queue is
  stopped and holds the balls (confirm the geometry facts in section 1 first). If the CELL leaves UP or the path is
  aborted, the gate closes again. If the geometry rules this out, the gate opens at settle and the budget grows
  (section 3).
- **Volley state machine** (pure Java, fake clock, unit-tested):
  `IDLE → GATE_OPENING (early, above) → ARMED (ShotGate true) → FEED_ONE (exactly one pitch, T1) → CONFIRM (dip seen
  during the pitch, then recovering) → WAIT_READY → FEED_ONE … → EMPTY (no ball under beam_staged after a pitch) →
  CLOSE_GATE → IDLE`.
- **"A ball left" (S4):** a dip of at least the v2 threshold **that then recovers**. v1's 35 t/s threshold is not
  reused: v2's expected dip is about 145 t/s, so the threshold is re-calibrated from the characterization (start
  ~0.5 × the measured median dip). A ball jammed against the wheel also makes a dip, but the speed stays down and the
  current stays high; that is a JAM, not a shot. Option for later: a third break beam at the hood exit (4 beams were
  bought, 2 are used) to see each ball leave; CAD space to be checked (17.2).
- **Phase 1 feed rule (T1, replaces S1):** FEED_ONE moves the queue **exactly one pitch** (`queue.ticksPerBall`,
  position-controlled: a move to a target tick count, then hold with BRAKE) and stops.
  - Ball k is taken by the wheel partway through the move (after about 40 mm). Ball k+1 ends the move at the staged
    spot, under `beam_staged`, about 40 mm short of the tread. **Position, not the dip, keeps ball k+1 out of the
    wheel.** The lane is uphill, so it can't roll on by itself.
  - Why not stop on the dip (S1): the dip almost always comes after about 40 mm, so the queue and ball k+1 would have
    moved only about 40–45 mm of the 73 mm pitch. Ball k+1 then sits short of the staged spot, ShotGate row 7 ("a ball
    staged") fails and the volley stalls after ball 1. And the dip is only detected 20–40 ms after the wheel has the
    ball, so stopping on it never prevented pushing during capture.
  - **The dip confirms the shot; it doesn't stop the queue.** No dip by the end of the pitch (plus a short grace) =
    miss-feed; a dip that doesn't recover = JAM (as above).
  - The next pitch starts only when the flywheel is READY again and ShotGate is still true: 0.30–0.40 s per ball
    (section 4.3, T2).
  - The balls sit touching in a single-file lane, so a queue that ran continuously would send ball k+1 into the wheel
    during recovery and READY would control nothing. One pitch at a time prevents that.
- **Phase 2 (later, only if phase 1 is accurate):** option A vs B is now **only** a phase 2 question: whether to start
  the next pitch **before** READY, timed by the measured queue travel so the ball reaches the wheel just as the speed
  comes back into the band. Judged by labelled shots (40 each), kept only if they show no loss.
- **Phase 2 experiment, gentle arrival:** slow the queue over the last ~20 mm before the wheel, so the ball reaches the
  tread gently. That is what v1's 33 % vs 59 % pointed at (a ball still being pushed hard while the wheel grabs it gets
  extra, uneven friction). Judged by labelled shots.
- **Faults:**
  - queue stall (no ticks for 0.15 s at power) → back off ¼ ball and retry, up to 3 times, then JAMMED;
  - a ball that doesn't leave (no dip by the end of the pitch, plus a short grace) → count it as a miss-feed and
    retry once;
  - FLYWHEEL FAULT, or the CELL not UP → stop the queue at once and close the gate.

## 6. Shot gate: every ball must pass all of these

| # | Condition | Value |
|---|---|---|
| 1 | Flywheel READY | READY speed inside the band for 40 ms (4.4.5) |
| 2 | No flywheel fault | encoders agree (4.4.4) |
| 3 | Robot still and arrived (R4) | speed < 2 in/s, turn < 5°/s, **and** distance to the planned end pose ≤ 1.5 in |
| 4 | Heading on target | \|heading − aim heading\| ≤ the pose's tolerance (≈ 1.5°, section 8), where the aim heading is computed from the **current settled pose** |
| 5 | **Our CELL is UP** | section 9; UNKNOWN = NO |
| 6 | Pose trusted (R5) | Pinpoint READY, **and** when the camera sees the CELL: \|camera bearing − odometry bearing\| ≤ 6° (raw, before any correction) |
| 7 | Feed OK | not JAMMED, a ball staged |
| 8 | Legal | in a launch area if the manual requires one; **to confirm in the Competition Manual** |

Each failure has a short reason code (`NOT_READY`, `FLY_FAULT`, `MOVING`, `NOT_ARRIVED`, `AIM`, `CELL_DOWN`,
`CELL_UNKNOWN`, `POSE`, `JAM`, `NOT_LEGAL`) shown on telemetry and logged. That way a slow volley can be explained
afterwards. Row 3 no longer uses `!isBusy()`: see 7.4.

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
- Poses are stored in v1's centre frame (field CAD numbers, matching AutoField), with **one** conversion to Pedro in
  `PedroFrame`. The working assumption is `x_p = x + 72`, `y_p = y + 72`, `heading_p = radians(heading)`. **That only
  holds if Pedro's +x points the same way as v1's +X and heading 0 is the same direction (R6).** Before any test is
  written against it, confirm it two independent ways:
  - place 3 named points (e.g. a corner of the RED start tile, the field centre, a GARDEN mark) in **Pedro's
    visualizer / field image** and read their Pedro coordinates;
  - put the robot on a known field mark, reset Pinpoint to that pose through PedroFrame, drive a short known path, and
    check it ends where the tape says.
  If either disagrees, the conversion includes a rotation; fix it in PedroFrame only.
- Unit tests check:
  - every pose converts both ways;
  - the mirror of a mirror is the same pose;
  - **the 3 points above land on the Pedro coordinates read from the visualizer** (an independent source, not the
    formula repeated);
  - no pose is outside the field or overlaps a FLOWER or the HIVE frame (robot half-size 9 in, plus a margin).
- The CELL centre positions for aiming also live in V2Field (from the field CAD; v1: RED AUDIENCE CELL x −12.7,
  y −1.6…−20.1).

### 7.3 Paths
- **AUTO paths are built once at INIT.** A **TeleOp mode A** path starts from wherever the robot is, so it is built
  when the button is pressed: one short path, never every loop (S2).
- **The whole path is checked, not only its end pose:** the robot footprint (half-size 9 in + margin) is swept along
  it against the HIVE frame, the FLOWERs and the walls. A unit test for every AUTO path; a quick check before a mode A
  path starts (if it fails, mode A refuses and says why).
- **The end heading points at the CELL**, computed as atan2 from the end pose to the CELL centre plus the launcher's
  direction on the robot (launcher at the back).
- Heading interpolation: linear to the end heading over the last part of the path. The robot arrives already aimed,
  to within the small re-aim of section 8.
- Speed limits per path segment. Slower on the last segment before a shot pose, so it settles fast.

### 7.4 "Settled" instead of a fixed wait (R4)
- A Pedro path can finish (`isBusy()` false) on its **timeout** end-constraint while still off the target. Check
  the exact end constraints in the Pedro 3 API and log which one ended each path.
- Settled = translational speed < 2 in/s **and** turn < 5°/s **and** distance to the planned end pose ≤ 1.5 in, all
  held for **40 ms**. `isBusy()` is logged but not required.
- Pedro keeps holding the end pose (hold point) while we shoot. After settle, the aim heading is recomputed from the
  **actual** pose (section 8) and the hold point's heading is updated to it.
- Target ≤ 0.2 s; v1 always waited 0.5 s.

### 7.5 Path Check OpMode (acceptance before any shooting relies on Pedro)
- Runs each AUTO path **10 times** from a taped start mark. It logs the end pose, the settle time, the peak following
  error and which end-constraint finished the path, and a person checks the end position against tape marks.
- **Pass:** at least **9/10 runs** within 1.0 in and 1.0°, **no run** over 1.5 in or 1.5°, settle ≤ 0.3 s. (Tape
  measurement itself has about ±0.25 in error, so 10/10 at 1.0 in would test the tape as much as the robot.) After a
  full AUTO route the Pinpoint drift is ≤ 2 in (tape).
- **Independent check:** at each shot pose the camera's CELL bearing (and range) is compared with what odometry
  predicts. They should agree within 2° and 2 in.
- **Watchdog** (also used in matches): following error > 6 in, or Pedro busy with speed ≈ 0 for 0.5 s (a robot or the
  wall is in the way) → stop the path, close the gate, don't fire, go to the fallback (section 11).
- Optional: dry-run the routes in Beta8397/virtual_robot before field time.

## 8. Aim: odometry first, camera as the check

- **Aim heading** = bearing from the **ball release point** (the hood exit, offset from the robot centre, from CAD,
  placed using the **current settled pose**) to the CELL centre (V2Field) + the launcher trim + the camera correction
  (below) (S9). Recomputing from the actual pose matters: at 36 in, a 1 in sideways error is
  about **1.6°**, already more than a 1.5° tolerance.
- **Camera check (R5), once per volley, after settle:**
  - raw difference d = camera bearing (corrected for the camera-to-launcher offset) − odometry bearing;
  - \|d\| ≤ 2°: no correction; shoot on the odometry aim;
  - 2° < \|d\| ≤ 6°: correction = d for the rest of this volley (the aim heading moves to the camera's bearing);
    log `AIM_CORRECTED`;
  - \|d\| > 6°: ShotGate row 6 fails (`POSE`). Odometry or the camera mount is wrong, and someone must look.
  - ShotGate row 4 uses the corrected aim heading; row 6 compares the **raw** d against 6°. They no longer conflict.
  - The correction is reset at the next volley; it never writes back into the odometry.
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
- **Four states:** UP, TIPPING, DOWN, UNKNOWN. UNKNOWN covers no frame, a frame older than 0.25 s, a cluster that is
  not our CELL, or a CELL further than the "front" distance (v1 `FRONT_MAX_FLOOR_IN` = 47). **Only UP fires.**
- **HIVE tipping (S6).** A HIVE tip scores, and only launching into the upward CELL may cause it (G417); the manual
  advises pausing while a HIVE is tipping, because balls arriving then may disrupt the tip. **Our own volley can tip
  it.** Balls already launched can't be recalled: at 0.3 s between balls and about 0.5–0.7 s of flight (to measure),
  2–3 balls can be in the air at once.
  - TIPPING = the CELL height (or orientation) changing faster than a threshold, seen on **2 consecutive unique
    frames** (capture timestamps, not loop count), so one odd frame can't raise it. The threshold is set from the
    **height noise measured on a still CELL** (T5). The queue stops and the gate closes at once.
  - Measure the delay from a frame's capture to the queue stopping (logged, section 13).
  - Find out what triggers a tip (Competition Manual section 10: a ball count, a weight, a timer?). If a tip can be
    predicted, slow the cadence, or hold the last balls, when one is near (17.2).
- **Only slow frames count (R8):** a frame enters the 5-frame window only if the robot was below 6 in/s and 30°/s when
  it was captured (frame timestamp vs the pose log), so motion blur at the end of a path cannot vote.
- **Orientation as a second cue: veto only (R8).** The tags are on the CELL's bottom, so a tipped CELL turns them.
  But small flat tags at about 40 in have a known pose ambiguity: the tilt can flip sign. Clusters of several tags
  reduce it but don't remove it. So orientation may turn UP into DOWN or UNKNOWN; it can **never** turn a non-UP into
  UP. Adopt it only if logs show a clean split; otherwise height alone decides.
- **Timing:** the camera runs during the last part of every shot path, so the verdict is ready on arrival. It is
  checked again **before each ball**: if the CELL tips mid-volley, the rest of the volley stops and the gate closes.
- **Camera settings:** a resolution and decimation that keep ≥ 25 fps, low fixed exposure and gain so tags stay sharp
  while turning, and only the tag processor running.
- **No CELL override in AUTO.** TeleOp: section 12.

## 10. Shot speed table

- **Input:** distance from **odometry** (ball release point to CELL opening, horizontal, S9), which is steadier than
  the camera. The
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
2. **START (t = 0):** path to shot pose 1 (from the CELL-wall start this is the 2 in move-off); flywheel to the shot
   speed, staggered or capped only if the START current test says so (4.3, R7). CELL check during the last part of the
   path; gate opens early (section 5).
3. **Settled** → re-aim from the actual pose → volley (section 5), up to 4 balls, ShotGate before each.
4. Path to the GARDEN with the intake on; queue loading. The intake stops at 4 balls (`beam_full`, G407, S8).
5. Path back to a shot pose, spinning up on the way → volley 2.
6. **Park** before the deadline. AUTO is 30 s; the deadline is computed from the **measured** travel time to park from each point of the route, plus a margin (v1 used a fixed 20 s) (S11).
- **Fallbacks:**
  - CELL DOWN or UNKNOWN at the shot pose → gate closed, wait there up to the time limit, then skip to the next step
    and keep the balls;
  - Pedro watchdog fires → stop, close the gate, then park if possible;
  - flywheel FAULT → don't shoot, collect and park;
  - without Pedro or Pinpoint READY → v1-style LEAVE on wheel encoders.

## 12. TeleOp assisted shooting (both modes; the driver picks)

- **Mode B is built first (S10).** It is the competition mode: the drivers keep collecting and avoiding robots while the
  software aims, which gives much of a turret's benefit with no extra mechanism. Mode A comes after it.
- **Pose at the start of TeleOp (S10):** AUTO's last pose if it is fresh and AUTO ended normally; otherwise, or if
  localization is lost during the match, the drivers re-zero on a known field mark (a button in INIT and during the
  match). Until the pose is trusted, modes A and B are off and the drivers aim by hand.
- **Mode A, "go to shot":** a button starts a Pedro path from the current pose to the nearest legal V2Field shot pose.
  Any stick movement cancels it. On arrival the volley runs while the trigger is held.
- **Mode B, "heading lock":** the driver translates with the stick, and the robot turns to the aim heading from
  odometry (camera-checked). The flywheel speed follows the odometry distance, and while the trigger is held each ball
  fires when ShotGate is true. Moving shots are not allowed in phase 1: ShotGate needs the robot still. In mode B, row
  3's "distance to the planned end pose" is replaced by "stick released for 40 ms". Moving shots are a later phase.
- **Who turns the robot in mode B (T6, not decided: 17.2.9).** Two options:
  - **Pedro's follower in TeleOp drive** with a heading target: one controller for AUTO and TeleOp, using the tuned
    Foresight heading terms. Needs a check that Pedro 3.0.2's TeleOp drive accepts a heading target while the driver
    controls translation, and how it behaves when the target jumps.
  - **Our own heading controller:** a PD on the heading error (+ a small static term) that sets the turn power, mixed
    with the driver's translation in field-centric mecanum. Simple and fully under our control, but it needs its own
    tuning and tests.
  - Recommended default until 17.2.9 is answered: **our own controller**, because it can be unit-tested now; switch
    to Pedro's if the API check shows it supports this cleanly. Either way the heading tolerance and "settled" use the
    same ShotGate rows 3 and 4.
- **Pre-spin is a setting (S10).** Default: spin at the table speed when we hold ≥ 1 ball **and** are within a set
  distance of our HIVE, plus a driver toggle. The hold current is small (4.3); confirm it in section 15.
- Telemetry and rumble: READY, CELL UP/TIPPING/DOWN/UNKNOWN, and the first failing ShotGate reason.
- **CELL override: team decision pending (17.2).** Two options:
  - **No override** (current default): the robot always refuses unless UP. Simplest to defend under G417; one camera
    failure means no shooting for the rest of the match.
  - **UNKNOWN-only override:** a deliberate two-button hold lets the driver fire while the state is **UNKNOWN** (the
    camera can't tell), never while it is **DOWN** or **TIPPING**. Every overridden shot is logged with a reason code. The driver
    takes responsibility for what they can see.
  Whichever is chosen, the reason is written here.

## 13. Logging

- **Per loop (≤ 20 Hz to file):** pose, follower state and which end-constraint finished the path, flywheel ω for both
  encoders (raw and the 80 ms READY speed), power, battery, CELL state and height, frame age and robot speed at
  capture, camera bearing and range, gate, queue ticks, beams, ShotGate reason.
- **Per shot event:** speed before, dip minimum and time, recovery time, odometry distance, camera distance, heading
  error, aim correction d, CELL state, time since the volley started, gate open time, and a label (driver LB + D-pad
  as in v1).
- **Per AUTO:** the lowest battery voltage in the first 0.5 s (R7), and the volley time split into the section 3
  phases.
- **CELL events:** every state change with its frame capture time, and for TIPPING the delay from that capture to the
  queue stopping (S6).
- CSV in `/sdcard/BioBuzzLogs`, pulled with `tools/pull_robot_logs.ps1`; an AdvantageScope converter later.

## 14. Code layout (all in `teamcode.v2`, no v1 imports)

| Class | Kind | Tests |
|---|---|---|
| `FlywheelController` (FF + P + bang, READY speed, READY band, fault) | pure Java | plant simulation: inertia, motor curve, injected dips, battery sag, a dead encoder, **28-tick quantization**; reports the **time between balls with the full READY logic** (T2) |
| `FlywheelModel` (the plant used by the tests) | pure Java | spin-up and recovery match section 4 |
| `VolleyStateMachine` | pure Java, fake clock | every transition, every fault, CELL tipping mid-volley, early gate open and close-on-abort, **exactly one pitch per ball; ball k+1 staged afterwards; no dip by pitch end = miss-feed** (T1), dip without recovery = JAM |
| `ShotGate` | pure Java | truth table, reason codes, raw vs corrected camera difference |
| `CellCheck` (v2) | pure Java | v1 tests + UNKNOWN / stale frames / slow-frame filter / orientation veto never yields UP / TIPPING from changing heights, using capture times |
| `V2Field` + `PedroFrame` | pure Java | conversion, mirror, visualizer points, poses legal, swept footprint of every AUTO path, release-point aim headings |
| `ShotTable` | pure Java | interpolation, clamping, hysteresis |
| `V2Shooter` (hardware wrapper) | OpMode-side | System Check |
| `V2FlywheelCharacterize`, `V2PathCheck`, `V2AimTest` | OpModes | robot procedures, docv2 |

## 15. Test plan

| Stage | What | Pass |
|---|---|---|
| Unit (laptop) | all pure classes above | all green, in CI with `testDebugUnitTest` |
| Bench, no disc | **Flywheel characterization:** kS and kV ramp; spin-up; hold power and current at 1200/1400/1600; **READY-speed noise at each** (sets the band, 4.4.3); dip and recovery with hand-fed balls; both encoders; logged at about 100 Hz | the controller holds the band; band ≤ ±2.5 % (else fit the Through Bore); recovery is measured; the model is updated |
| Bench, with disc | safety steps first (4.3); the same again + the inertia from powered acceleration with current + coast-down (4.3, S3) | 3 inertia results within ~10 %; recovery ≤ 0.2 s; spin-up time recorded |
| START current | AUTO start on the field with the flywheel from rest | lowest battery logged; no brownout; decide stagger / cap (4.3) |
| Feed geometry | gate servo time for 90°; with the queue stopped and the gate open, ball 2 stays at the staged spot; the opening paddle doesn't push or lift ball 1; one pitch puts ball 1 into the wheel; **after one pitch, ball 2 reads under `beam_staged`** (T1) | all five true, or sections 3 and 5 updated |
| Feed | volleys of 4 into the flywheel, robot on the box: time between balls, double feeds, jams; phase 1 = option A | ≤ 0.30 s per ball, 0 double feeds in 20 volleys |
| Pedro | frame check (7.2), v2 AutoTune, then Path Check (7.5) | visualizer points agree; 9/10 within 1 in / 1°, none over 1.5 in / 1.5° |
| Aim | 20 balls from each shot pose, with the camera vs odometry check | ≥ 90 % within the CELL width; median bearing error ≤ 1° |
| CELL | 20 UP and 20 DOWN checks at each shot pose, both alliances | 40/40 right; DOWN is never called UP |
| Speed table | section 10 (20 shots per distance to tune) | then **≥ 100 shots** over several batteries, balls and shot poses: ≥ 80 % (first working), ≥ 85 % (competition-ready) |
| AUTO | 10 full runs | volley within the current milestone (section 3), ≥ 80 % IN, park every time |
| Electrical (S11) | full-match endurance: 2:30 of driving, collecting and shooting, with a full and a part-used battery | battery voltage under load, motor currents and temperatures, recovery time and loop time (Pedro + camera + logging running) all logged and within limits; the flywheel stops on any fault, STOP or OpMode end |
| CELL tipping (S6, T5) | (a) tip the HIVE by hand while a volley runs (robot on the box, balls into a net); (b) a few minutes with nothing moving, counting false TIPPING events | (a) queue stops, capture-to-stop delay measured; (b) **0 false TIPPING events** |
| TeleOp | drivers, timed, both modes | the drivers pick the default mode |

## 16. Order of work

| # | Step | Needs |
|---|---|---|
| 1 | v2 Hardware Test, V2Config, V2Calibration | **done** |
| 2 | `V2Hardware` | 1 |
| 3 | v2 Setup (calibration OpMode) + `02_SETUP.md` | 2 |
| 4 | v2 System Check + `03_SYSTEM_CHECK.md` | 3 |
| 5 | FlywheelModel + FlywheelController + characterization OpMode; **READY-speed noise measured and band set** | 2 (bench without the disc now, **again with the disc**) |
| 6 | **Feed geometry checks (section 1)**, then port the queue/gate/intake logic + VolleyStateMachine + ShotGate | 3, 5 |
| 7 | **Pedro frame check**, v2 Pedro re-tune, V2Field/PedroFrame, Path Check | 2 |
| 8 | CellCheck v2, aim (odometry + camera check), camera/lens calibration | 2 + **V2Field/PedroFrame** from step 7 (not the re-tune or Path Check, T4) |
| 9 | Shot speed table | **disc fitted**, 5–8 |
| 10 | AUTO sequence + START current test | 6–9 |
| 11 | TeleOp modes A and B (+ the override decision, 17.2) | 6–9 |
| 12 | Logs → AdvantageScope | any time after 6 |
| later | Shooting while moving (lead the CELL by robot speed × flight time); phase 2 feed timing | all of the above proven |

Each step: unit tests → external review → robot doc in `docv2` → robot test → commit.

**Grouped into phases** (each phase leaves the robot able to do something useful; the Pedro and vision work can run in
parallel with the shooter characterization):

| Phase | Steps | Exit |
|---|---|---|
| 1. Reliable mechanical shooter | 2–6 (+ the disc) | 20 volleys with no jam or double feed; flywheel characterized |
| 2. Accurate stationary shooting | 8–9, from a **hand-placed robot on a taped pose, no Pedro** (T4): Pinpoint is set to the taped pose, so the aim and the distance still come from odometry | ≥ 80 % IN over ≥ 100 shots; CELL check right every time |
| 3. Pedro AUTO | 7, 10 | 10 good AUTO runs |
| 4. TeleOp assistance | 11 (mode B, then A) | drivers prefer it |
| 5. Speed | phase 2 feed timing, the 1.2 s volley, then shooting while moving | milestones in section 3 |

## 17. Open questions

### 17.1 Answered in review 1 (kept for the record)
1. **Derating ×1.8:** fair as a start; expect a bit worse with two motors (more sag). Plan 0.16–0.20 s (4.3).
2. **Own loop vs the hub's:** own loop, one power for both motors; copying the hub's power isn't possible (4.4.1).
3. **Velocity resolution:** a 40 ms difference doesn't help; sum both encoders over 80 ms, set the band from measured
   noise, Through Bore as the fallback (4.4.3).
4. **Feed A vs B:** A for phase 1, because B bypasses READY; B is a phase 2 experiment (5). Revision 4 (T1): phase 1 moves exactly one pitch per ball; A vs B is now only whether phase 2 starts the next pitch before READY.
5. **Pedro settle:** `!isBusy()` isn't proof of arrival; add the pose error, re-aim from the settled pose, 9/10
   acceptance (7.4, 7.5, 8).
6. **CELL orientation:** veto only (9).

### 17.2 Still open
1. **Competition Manual:** is there a launch-area rule (ShotGate row 8)? Anything beyond G417 about launching?
2. **TeleOp CELL override:** no override, or UNKNOWN-only override (section 12)? Team decision.
3. **Pedro 3 end constraints:** which constraints end a path in 3.0.2, and their defaults (timeout especially)?
4. **Feed geometry:** answered from CAD in revision 3 (section 1); the robot checks are in section 15.
5. **Through Bore:** only if the measured READY band exceeds ±2.5 %. If it's needed, check the hub's encoder count
   rate at 3000 rpm × 8192 counts/rev.
6. **Anything else that makes a volley slow or inaccurate** that we haven't seen yet: the ball hitting the hood edge,
   ball-to-ball spacing in the lane, camera latency.
7. **HIVE tipping:** what triggers a tip, and how should the cadence change when one is near (section 9, S6)?
8. **Third break beam** at the hood exit: worth the CAD change (section 5)?
9. **Mode B heading control (T6):** Pedro's follower in TeleOp drive with a heading target, or our own heading
   controller (section 12)? Check what Pedro 3.0.2's TeleOp drive supports first.

## 18. Decisions already made (team, 21 Nov)

- **Stop and shoot first;** shooting while moving comes later.
- TeleOp: **both** assist modes; the driver picks.
- Camera: **checks the odometry aim** and corrects it when they differ by 2–6° (section 8); more than 6° stops the shot.
- Steel disc: **ordered**. The speed table waits for it; the controller is characterized again once it is fitted.
- v1 code stays untouched (tag `biobuzz-v1`); v2 never imports v1 classes.
- Feed phase 1: **exactly one pitch per ball**, the next pitch only after READY (revision 4, T1; replaces the revision 2/3 wording); **team to confirm.**
- **Accuracy before speed** (revision 3): the volley-time milestones only count once their accuracy target is met.

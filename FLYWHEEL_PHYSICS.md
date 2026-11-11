# Flywheel physics check (10 Nov 2026)

How the shot speed is worked out (`tools/shot_model.py`, `ShotSpeed`), checked step by step against physics and the diag logs.

## 1. The formula, step by step

| Step | Formula | Check |
|---|---|---|
| Encoder → wheel speed | rev/s = t/s ÷ 28 (goBILDA 6000 rpm motor, 28 ticks per rev, 1:1 to the wheel) | ✅ 1450 t/s = 51.8 rev/s = 3107 rpm. Free speed 6000 rpm ≈ 2800 t/s, so `LAUNCHER_MAX_VELOCITY` 2600 fits |
| Tread speed | rev/s × π × 0.096 m (96 mm Hogback wheel) | ✅ 15.6 m/s at 1450 t/s |
| Ball speed | The ball rolls between the moving tread and the still hood, so its centre moves at **half** the tread speed (no slip), × η | ✅ Physics is right. η is a fitted factor (slip, squeeze, the dip), **0.71 with one wheel** (the drivers' 1440 t/s at ~36 in ↔ 5.52 m/s needed), not the 0.85–0.93 of the two-wheel days the model's notes quoted. The speed table is set from real shots, so the code was not affected; `shot_model.py` now prints a 0.71 column |
| Spin | Rolling on the hood gives spin = ball speed ÷ radius; the hood is on the outside of the ball's curve → **topspin**, Magnus pushes the ball forward / down | ✅ Matches the hood CAD |
| Flight | Gravity + drag (Cd 0.5) + Magnus (CL = S / (2.022 S + 0.981), S = 1 → CL 0.33). Ball 25 g, Ø72 mm | ✅ Drag is ~24 % of the ball's weight at 7 m/s, so both drag and Magnus matter; both are in the model |
| Target | CELL lips from the field CAD (53.5 / 65.1 in), ball exit 9.0 in high at 74° (hood CAD) | ✅ The launch angle hardly matters (72–76°: ±1 % speed) |
| Scoring window | Ball speeds that clear both lips by ball radius + 5 mm | **Only about ±4.2–4.7 %** of the ball speed at 36–44 in (Cd, the Magnus fit, the spin and the 5 mm margin are not measured, so treat the tenths loosely) |

## 2. What the formula leaves out: the dip

- Each ball is squeezed (7 mm) against the tread for ~30 ms and takes energy from the flywheel: the speed drops (the "dip"). The ball's speed is set by the tread speed **when it leaves**, not the speed before the shot: v_ball = κ · (1 − D_exit) · tread speed before the shot ÷ 2, with κ = slip at the exit and D_exit = the slowdown at that moment. η (0.71) lumps both, so it is a fitted factor, not an efficiency.
- A rigid "sticking impulse" model (a stationary ball captured by the tread; hood friction acts *forward*: wheel impulse 5mv/6, hood mv/6) gives ω_after = ω_before ÷ (1 + (1 + c)·m·r² ÷ (4 I)), with c = 2/3 (thin shell), m = 25 g, r = 48 mm (tread), I = flywheel inertia (CAD: Hogback wheel 1.31e-4 kg·m², hub 0.05e-4, motor rotor ~0.3e-4).
  - That model's dip: **12.6 % with one wheel** (I 1.66e-4) vs **7.5 % with two** (I 2.97e-4). (A pure energy balance would give 6.5 % / 3.8 %; the capture itself dissipates energy.)
- Logged dips are bigger: 10 Nov one wheel p10 / median / p90 ≈ **16.7 / 20.5 / 24.3 %**; the two-wheel days (4–6 Nov) were lower, about 10–18 %, but those logs are coarser and mixed, and wheel 2 also touched the ball, so they are not a clean comparison. Squeeze and slip losses make up the difference, which is also why η fell from 0.89 to 0.71.
- **Caveat on the numbers:** the logs sample the flywheel every ~60 ms (p90 73 ms) in 20 t/s steps, while a ball is in contact for ~30 ms. So a logged minimum is not exactly the exit speed: sampling phase, the hub's velocity filter and the motor's torque all add to the spread.
- **If** the logged dip spread maps one-for-one to the ball speed, the ball speed spreads about ±4.4 % (1 σ). Against the model's scoring window (±4.2–4.7 % at 36–44 in), the speed spread alone would then limit the IN rate to about **60–70 %**, even with perfect aim and the right set speed. This is a likely explanation for IN, SHORT and LONG from the same spot at the same speed, **not a measured ceiling**.
- The 10 Nov labels point the same way: SHORT balls had deeper dips (median 340 t/s, n 7) than LONG balls (290, n 9). That is an association with small numbers (at 1440–1455 t/s and 35–43 in: SHORT 4 vs LONG 6), not proof.
- **How to measure it properly:** film shots with a phone in slow motion (240 fps) beside a ruler or tape marks to get the real ball speed, and log the flywheel faster around a shot.

## 3. What would help most

- **More flywheel inertia, from a mass that doesn't touch the ball.** It shrinks the part of the spread that comes from the dip; it can't fix ball-to-ball slip or squeeze.
  - Example: ~170 g of steel at up to 38 mm radius (two or three 3-inch fender washers, Ø76 mm) on the motor shaft where wheel 2 used to be. Inertia 1.66e-4 → 2.9e-4 kg·m² (**1.74×**, back to about the two-wheel value; ~230 g would double it).
  - Dip D' = D ÷ (α(1 − D) + D) for an inertia ratio α: 15–24 % becomes ~9–15 %. Under the same assumptions as above, the speed-only IN rate goes from ~60–70 % to **~84–90 %**.
  - **Clearance must be checked properly:** a 38 mm disc is only ~5 mm from the squeezed ball at one plane. Check the full washer and hub stack against the squeezed ball's swept shape in CAD, then turn the flywheel slowly by hand with a ball in the hood before running it. Keep it centred (balance) and clamped tight.
  - **Side effects:** stored energy at 1450 t/s goes from 8.8 to 15.3 J, so spin-up takes longer (pre-spin with Y). Check that READY still settles (flywheel P 100).
- The second Hogback wheel came off on 7 Nov because it touched the ball's side and pushed it sideways; a non-contacting mass avoids that.
- **A lead from the 10 Nov labels:** balls the flywheel took **after** the windmill had stopped went IN 59 % (16 / 27); balls taken **while** the windmill was still pushing went IN 33 % (8 / 24, more SHORT). Same direction in both evening blocks, but p = 0.09, so it needs more shots before changing the feeder.
- Software can only get the set speed right (labels + `tools/shot_results.py`) and the aim right (RB aims by itself since 10 Nov). It can't see a ball's grip before the shot.

## 4. Speed table vs physics

- The model's ball speed rises ~0.5 %/in beyond ~34 in.
- The labelled shots (10 Nov) rise more slowly:
  - 1440 t/s is right at 36–40 in (the table was lowered there on 10 Nov);
  - at 48–51 in, 1450 gave IN and SHORT; the one shot at 1525 from 48 in went LONG.
- The table above 44 in (1525–1640, shaped by the model's slope) is probably 30–50 t/s too fast. **Shoot from 36–44 in** (save the SHOT SPOT there) until shots at 48 in and further have been labelled at AUTO speed.

## 5. The windmill speed (item 3)

- **It did not change.** Feed power is 0.40 on both nights, and a pocket takes ~0.2 s: 9 Nov 2 → 116 ticks in 0.20 s, 10 Nov 7 → 108 ticks in 0.18 s. In AUTO it was ~400–450 ticks/s (median) both nights.
- What looks different: since 9 Nov afternoon the windmill skips the pause after an **empty** pocket, so it turns more often.
- **Physics:** the windmill pushes a ball at ≲ 0.8 m/s into a tread moving ~15 m/s; by momentum alone that changes the ball's exit speed by only ~1–2 %. The feeder also waits until the flywheel is back within ±30 t/s before every ball (logs: within ±20 t/s at launch).
  - The feed can still matter through how the ball enters the squeeze (see the lead in §3: balls taken while the windmill was still pushing did worse), and for jams, double feeds and where balls come to rest (the front-left gap, now being filled).

## Sources

- `tools/shot_model.py` (run it for the window table)
- Diag logs `robot_logs/20261110_224811` (dips by day, AUTO dips, windmill)
- 10 Nov labels (`tools/shot_results.py`, GRAB events)
- Inertia from the CAD (MEMORY §3)

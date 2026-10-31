# BIOBUZZ robot test plan

The laptop tests (`.\gradlew :TeamCode:testDebugUnitTest`) prove the drive math, the control loops and the AUTO route logic in a simulation. This plan proves the rest **on the real field**: camera, shots, real AUTO runs, failure handling and full matches.

Do the stages in order. A stage counts as done only when every pass target is met. Record the date and result in the table at the end.

## Getting the logs

Logs are written on the Control Hub to `/sdcard/BioBuzzLogs/`:

| File | Written by |
|---|---|
| `vision_*.csv` | `BioBuzz Vision Test` |
| `shots_*.csv` | `BioBuzz Shot Log` |
| `auto_RED_*.csv`, `auto_BLUE_*.csv` | every AUTO run, practice or match |

Copy them to the laptop (on the Control Hub Wi-Fi) and open them in Excel:

```powershell
adb connect 192.168.43.1:5555
adb pull /sdcard/BioBuzzLogs .\logs
```

## Stage 0: Hardware and calibration

0. Run `BioBuzz Wiring Check` with the robot on a box. **Pass:** "WIRING OK". Fix every FAIL first: the line says which cable.
1. Run `BioBuzz System Check`, all three parts. **Pass:** every line says PASS.
2. Run `BioBuzz Calibration` and copy the values into `BioBuzzConstants.java`:

| Check | Pass |
|---|---|
| Push the robot 48 in along a tape measure | "Forward in" reads 48 ± 0.5 |
| Slide 48 in | "Strafe right in" reads 48 ± 1 |
| Turn the robot one full turn by hand and return it to its start mark | Heading back within ±2° |
| Arm deploy and stow (run `BioBuzz Arm Setup`; again after any cradle, arm or horn change) | Both stops reached, no buzzing (servo fighting a stop) |
| Arm deployed, robot on the field floor | Wedge does not touch the floor: a sheet of paper slides under it. Drive slowly with the arm out: no scraping. |
| Measure the camera lens height and tilt | Enter them as `CAMERA_LENS_HEIGHT_IN` and `CAMERA_TILT_DEG` |
| Camera mount check | Camera on the robot's centre line (left/right), facing the launcher side, right side up. Off-centre aims the camera, not the launcher, at the CELL. Upside down makes auto-aim turn the wrong way. |

3. Rebuild, then run System Check again.

## Stage 1: Camera (`BioBuzz Vision Test`)

Use the real HIVE. Mark spots with tape and measure the **floor distance from the camera lens to the point directly under the CELL tags** (use a plumb line or a square).

For each spot, set the truth with the D-pad and press A to record a 3-second capture:

| Spots | Angles |
|---|---|
| Planned AUTO shooting spot | 0°, ±10°, ±20° |
| Every TeleOp shooting spot the drivers want | 0° |
| From 24 in out to the farthest distance you would shoot from, every 12 in | 0° |

| Measure | Pass |
|---|---|
| Detection rate, robot still | ≥ 90 % of frames at every planned shooting spot |
| Detection rate, slow driving (precision speed) | ≥ 50 % |
| Bearing error (mean − truth) | Within ±2° |
| Bearing spread | ≤ 1° |
| Floor distance error | Within ±10 % (if worse, fix `CAMERA_TILT_DEG` and `CAMERA_LENS_HEIGHT_IN` first) |
| Tag height | The same at every spot within ±1 in (a big change means the tilt value is wrong) |
| Hold X from 20° off target, 10 times | Settles within ±2° in ≤ 1.5 s, 10 of 10 |

Also note the **farthest reliable distance**. Drivers should not count on auto-aim beyond it.

## Stage 2: Shots (`BioBuzz Shot Log`)

For each shooting spot (AUTO spot first):

1. Aim with X, then shoot **20 single balls** at a speed.
2. Mark every shot A (IN) or B (MISS).
3. Repeat at speeds ±50 and ±100 t/s around the best one.
4. Repeat the best speed with a battery at about 12.0 V.

| Measure | Pass |
|---|---|
| Best speed at the AUTO spot | ≥ 16 of 20 in (80 %), then set it as `AUTO_LAUNCH_VELOCITY` |
| Same speed at 12.0 V battery | ≥ 14 of 20 in, otherwise retune `LAUNCHER_PIDF` (the flywheel speed controller) |
| Recovery time (log `recovery_s`) | Note it. If continuous feeding (hold RB) hits less often than single shots, lower `INDEXER_FEED_POWER` |
| HIVE TIP | Record how many balls in the upward CELL make the HIVE tip |

Copy the near and far speeds into `LAUNCHER_NEAR_VELOCITY` and `LAUNCHER_FAR_VELOCITY`.

## Stage 3: AUTO repeatability

Mark the start position with tape for **each** alliance: intake side touching the ALLIANCE wall, about 1 in outside the LOADING ZONE tape (see `BioBuzzAutoBase.java`).

1. Run AUTO 3 times with shooting OFF (X in INIT). Adjust `AUTO_PARK_STRAFE_IN` until the robot ends well inside the LOADING ZONE.
2. Run AUTO 3 times with shooting ON. Check where the camera turns the robot; if the IMU turn is far off, adjust `AUTO_AIM_HEADING_DEG`.
3. Then run **10 timed runs on each alliance** with 4 POLLEN and nothing changed between runs.

| Measure (from `auto_*.csv` and a tape measure) | Pass |
|---|---|
| LEAVE: robot not touching the wall at 0:00 | 10 of 10 |
| AUTO PARK: robot partly in the LOADING ZONE | 10 of 10 |
| `done_s` in the RESULT row | ≤ 25 s in every run |
| Final heading | Within ±3° |
| Final position vs the average of all runs | Within ±2 in |
| POLLEN in the CELL | Average ≥ 3 of 4 |
| Never crosses to the opponent side, never touches a FLOWER or the HIVE | 10 of 10 |
| Nothing moves during INIT | 10 of 10 |

## Stage 4: Failure drills

Each drill must end safely. If a drill fails, fix it before continuing.

| Drill | How | Pass |
|---|---|---|
| Camera off | AUTO INIT, press Y (camera OFF) | Aims with the IMU only and still gets LEAVE + PARK |
| Camera unplugged | Unplug the USB before INIT | AUTO parks; TeleOp drives normally, aim says "camera not found" |
| Launcher encoder unplugged | Unplug the launcher encoder cable | AUTO never feeds and parks before 0:00; TeleOp shows the encoder WARNING |
| Windmill jammed | Wedge a ball so the windmill cannot turn | AUTO gives up firing and parks before 0:00 |
| Longest start delay | AUTO INIT D-pad up to 10 s | Still LEAVE + PARK before 0:00 |
| Low battery | Battery at about 12.0 V | AUTO still LEAVE + PARK; note the shot result |
| Arm deploy while moving | Hold the stick and press A | Arm does not deploy, controller buzzes |
| AUTO to TeleOp transition | Let AUTO end, INIT TeleOp | Nothing moves during INIT (rule G403) |

## Stage 5: Full-match rehearsal

Run at least **5 full matches**: 30 s AUTO, 8 s transition, 2:00 TeleOp. Use a phone timer and, if possible, a partner robot.

**Before each match:**
- [ ] Battery ≥ 12.8 V, robot configuration active, latest code deployed
- [ ] Robot fits the 18 × 18 × 18 in sizing box, flower arm stowed
- [ ] Exactly 4 POLLEN preloaded
- [ ] Correct `BioBuzz AUTO RED` / `BLUE` selected; start delay agreed with the partner
- [ ] At TeleOp INIT, the alliance on the screen is ours (B = RED, A = BLUE); it resets to RED if the app restarted
- [ ] Robot on the start tape, touching our wall, outside the LOADING ZONE, not touching a FLOWER
- [ ] Camera plugged in, INIT screen shows camera ok and the plan

**Score and check during the match:**
- [ ] AUTO: LEAVE, PARK, POLLEN in the CELL, HIVE TIP
- [ ] TeleOp: cycles completed, shots in or out, never more than 4 POLLEN held (G407)
- [ ] No HIVE contact (G417); flower arm used only as allowed (G415 / G418)
- [ ] Rumble at 20 s left; PARKED in the LOADING ZONE at 0:00
- [ ] Pull the AUTO log after the match

**Pass:** 5 of 5 matches with LEAVE + AUTO PARK + TeleOp PARK and no rule problems, and drivers comfortable with every control.

## Results

| Date | Stage | Result | Notes / values changed |
|---|---|---|---|
| | | | |

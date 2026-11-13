# MEMORY.md: project memory for AI agents

Read this first. It records what was built, why, what was measured on the robot, and what is still open, so a new agent session can continue without starting over. Keep it up to date when you change something important.

Last updated: 2026-11-03 (branch `field-tour`).

---

## 1. Team, robot, goals

- **FTC team 37102**, season **BIOBUZZ 2026-27**. The users are a coach/parent plus students. Keep explanations **short and simple**. Students find long documents boring, so give them visual cards and numbered steps.
- **Robot:** goBILDA StarterBot with REV **Control Hub + Expansion Hub**.
  - Mecanum drive with 4 × goBILDA 312 RPM motors (537.7 ticks/rev, 104 mm wheels).
  - **Flywheel launcher:** 1 motor at 1:1 ratio (28 ticks/rev, 6000 RPM), goBILDA Hogback 96 mm wheels. It now has **two wheels side by side** (added 31 Oct): wheel 2 touches wheel 1 on the side away from the motor (robot photos, 3 Nov).
  - **Windmill indexer:** polycarbonate rotor, 4 pockets, 435 RPM motor (384.5 ticks/rev, 96.1 ticks per pocket).
  - **Intake:** motor + 2 corner CR servos.
  - **Flower-arm** servo with a wedge.
  - **Rear C270 camera** on the launcher side, tilted up 30°, about 12.2 in high.
  - **goBILDA Pinpoint** odometry, 4-bar pods.
- **Game pieces:** balls ("pollen") are shot into the alliance **HIVE CELL**. The HIVE **tips/rotates** during the game ("upward-facing CELL"). There are 2 HIVEs (Blue and Red) hanging about 31 in above an A-frame in the middle of the field.
- **Robot "front" = intake side.** The launcher and camera are at the back.

## 2. Repositories and environment

| Item | Value |
|---|---|
| Robot code | `C:\repo\FtcRobotController` (fork `rajkumar-rangaraj/BIOBUZZ-FtcRobotController`, `upstream` = FIRST). Our code is in `TeamCode/.../teamcode/biobuzz/` |
| CAD and print repo | `C:\repo\goBildaStarter` (`rajkumar-rangaraj/ftc-biobuzz-starterbot`): Fusion via FusionBridge (`fusion.py`), Python/manifold3d generators, Bambu P2S print jobs in `bambu_p2s/` |
| SDK | **FTC SDK 12.0** (merged 31 Oct; the Driver Hub was also updated to 12.0). 12.0 changed AprilTags: detections are `AprilTagSingleDetection` or `AprilTagClusterDetection` |
| Branches | `master` = competition code, checked out in `C:\repo\FtcRobotController` (4 Nov). `field-tour` = Pedro Pathing test (see §8), checked out in the **worktree `C:\repo\FtcRobotController-field-tour`** (own VS Code window; `local.properties` copied there). **`master` is merged into `field-tour`** (5 Nov `198208d`: printed-hood flywheel code; `08aff9d`: windmill feed fix); merge again after new `master` commits (they touch different files, no conflicts so far; `field-tour` does NOT have `b1cee74` yet). **`auto-work`** = the user's AUTO work (7 Nov, from `master` `b1cee74`), **worktree `C:\repo\FtcRobotController-auto-work`** (`local.properties` copied, builds). **MEMORY.md exists only on `field-tour`** (and `auto-work`, whose copy got its own §8b on 9 Nov: merge the two copies by hand when the branches meet). **9 Nov: `master`, `auto-work` and `field-tour` pushed to `origin`** (the user's fork). Ask before pushing |
| Build | `$env:JAVA_HOME='C:\Program Files\Eclipse Adoptium\jdk-17.0.20.101-hotspot'; .\gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug -q` (exit code 1 can just be a stderr deprecation note: check for `BUILD SUCCESSFUL` / `error:`) |
| Tests | JUnit in `TeamCode/src/test/...` (98 tests on `field-tour`, 93 on `master`, 5 Nov night). Results in `TeamCode/build/test-results/testDebugUnitTest/*.xml` |
| adb | `%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe`. Hub at `192.168.43.1:5555`. **Always use `-s 192.168.43.1:5555`**, because a stale "offline" device sometimes appears |
| Wi-Fi | The laptop has 2 adapters: `Wi-Fi` = home (Sweethome 2.4), `Wi-Fi 5` = robot. Robot SSID **`37102-RC`**. Reconnect with `netsh wlan connect name="37102-RC" interface="Wi-Fi 5"`. "not available to connect" = the robot is off (the SSID shows ~1 min after power-on). Disconnect afterwards (`netsh wlan disconnect interface="Wi-Fi 5"`) so the Driver Hub can connect |
| Install | `adb -s 192.168.43.1:5555 install -r TeamCode\build\outputs\apk\debug\TeamCode-debug.apk`. **This restarts the robot app (about 30 s), so ask before installing while the team is driving.** Check the op mode list with `logcat -d \| grep "registered {BioBuzz"` |
| Logs | `powershell -ExecutionPolicy Bypass -File tools\pull_robot_logs.ps1` puts them in `robot_logs\<timestamp>\`: `BioBuzzLogs/*.csv` (from `/sdcard/BioBuzzLogs`), `biobuzz_*.txt` settings, robotControllerLog, matchlogs |
| Commits | Write the message to a file and use `git commit -F` (PowerShell here-strings are fragile; when a command fails part-way, the message file may not have been rewritten). Add the trailer `Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>` |
| Docs | docx via node `docx` in `%TEMP%\guide_build` (copy the generator there; node resolves modules from the script folder). PDF via Word COM. The HTML driver card goes to PDF via `msedge --headless --print-to-pdf`. **If a PDF is open in a viewer the copy fails**: write to `%TEMP%` and copy later |
| Hub clock | It can reset to 1969 after power loss. That's harmless, but timestamps in logs/`lastUpdateTime` then look odd |

## 3. Hardware configuration (Driver Hub config `testbotnew.xml`)

- Control Hub motors: 0 `left_front_drive`, 1 `left_back_drive`, 2 `launcher`. Expansion Hub motors: 0 `right_front_drive`, 1 `right_back_drive`, 2 `intake`, 3 `indexer`. Servos: `left_intake_servo`, `right_intake_servo`, `flower_arm`.
- Drive directions: left side REVERSE, right side FORWARD (`BioBuzzRobot`).
- **I2C bus 0 has two devices:** `<ControlHubImuBHI260AP name="imu" port="0" bus="0"/>` and `<goBildaPinpoint name="pinpoint" port="1" bus="0"/>`. Editing the config on the Driver Hub once **replaced** the `imu` entry; it was fixed by pushing the XML over adb. A backup is in `robot_logs/config_backup/`.
- Pinpoint measured on 30 Oct (track 410 mm, wheelbase 270 mm):
  - forward pod `PINPOINT_X_POD_LEFT_MM = -40`
  - strafe pod `PINPOINT_Y_POD_FORWARD_MM = -175`
  - both not reversed (the plugs were swapped once, then swapped back)
  - goBILDA sign convention
- Webcam name: see `BioBuzzConstants.WEBCAM`.

## 4. Competition code (`master`), how it works

Key files are in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/`:

| File | Role |
|---|---|
| `BioBuzzConstants` | All tunables (launcher presets NEAR 1450 / FAR 1570 t/s for ONE flywheel wheel since 7 Nov (two wheels: 1150 / 1240), auto shot speed table `AUTO_SPEED_*`, flywheel P 100, trim step 5 (was 15 until 9 Nov), windmill power 0.40, AUTO route, aim and aim-gate, ball-count dip, Pinpoint …) |
| `ShotSpeed` | TeleOp auto shot speed (`b1cee74`, 7 Nov): base speed from the camera floor distance by the `AUTO_SPEED_*` table (1440 t/s at 30 in … 1640 at 64 in), rounded to 5, hysteresis 10 t/s, latched while FIRE is held (a new target resets `LauncherGate`), kept 3 s after the CELL is lost then the preset, follows ONE CELL (switches only after it is out of view 0.5 s). MANUAL (D-pad ↑/↓) ignores the camera until □. Trim on top. Diag event `SPEED`. Unit-tested |
| `ShotResults` + `tools/shot_results.py` | TeleOp shot labels → speed table (9 Nov); see "Shot results" below the table |
| `BioBuzzRobot` | Hardware wrapper (drive, launcher PIDF velocity, `updateLauncherReady()` once per loop / `launcherReady()` = `LauncherGate`, intake, indexer, arm) |
| `LauncherGate` | FIRE rule: speed within ±30 t/s of the target for 0.1 s; fallback after 1 s only inside target −60…+30 (never long). Unit-tested |
| `AimGate` | TeleOp aim gate: with our CELL in view, a ready ball waits for aim ≤ 2.0° and no turning (and, while X drives to a saved spot, the right distance); 0.75 s hold through a lost CELL; every hold gives way 1.5 s after it first held a ready ball; X in INIT = off. Unit-tested |
| `BallCounter` | Balls from flywheel dips (AUTO, Tune): 35 t/s below the learned speed, ≥ 60 ms, only within 0.6 s of a fed pocket (AUTO: 1 per pocket). Unit-tested |
| `BioBuzzTeleOp` | Driver TeleOp, one gamepad (button list in the file header) |
| `BioBuzzAutoBase` + `AutoRoute` + `MotionController` | AUTO (RED/BLUE) |
| `ShotFeeder` | Pocket-by-pocket windmill feed: turns one pocket on a fixed grid (power ramps 0.40 → 0.15 over the pocket), then waits `minInterval` ("pause") and for the launcher to be ready. Since 5 Nov night (`8967f09`): a pocket cut short (jam, unjam) is finished first; never aims > 2 pockets ahead (then the first boundary ahead); stuck 0.15 s → +0.1 power steps, faded over the last quarter pocket; jam = 1 s timeout → back-off ≤ ¼ pocket (≈ ½ with braking), retry the same boundary; 3 jams → JAMMED until FIRE is released. Events `SHOT n`, `STUCK …`, `JAM n …`, `JAMMED …`. **Since 9 Nov (TeleOp + Tune only, `skipEmptyPockets`; AUTO unchanged):** a pocket with no flywheel dip while it turned and for the empty wait after it (`feed_empty_wait_sec`, default 0.10) launched nothing → the next pocket starts without the pause (`NO PAUSE`). A pocket started earlier than the plain pause after the last pocket's end is protected: any dip with ticks to go stops the windmill (`HOLD`) until ready + pause since the dip; if nothing dips after the resume, the pause counts from the dip (next pocket at once, protected again). FIRE let go during a hold → IDLE there (`RELEASED`, like a cut-short pocket; the next FIRE finishes it after the pause and ready). A resumed pocket turns in the same loop (review fix: a stopped loop counted toward the jam timeout). `update` has a 5-arg form: flywheel speed (dip detection) separate from TeleOp's aim gate |
| `ContinuousFeeder` | Experimental continuous feed. **Not used** (too fast: 8 pockets in 1.2 s, only 1–2 balls launched) |
| `HiveVision` | Camera. **SDK 12 cluster aiming:** uses `AprilTagGameDatabase.getBioBuzzTagLibrary()`. Each CELL = a cluster (RED SCORING 30-33, RED AUDIENCE 34-37, BLUE AUDIENCE 38-41, BLUE SCORING 42-45). The pose = centre of the CELL **opening**, even with only 2 of 4 tags visible (averaging single tags aimed at a corner). `bearingDeg` includes the **aim trim** (+ = robot/shots turn LEFT); `rawBearingDeg` doesn't |
| `TeleOpSettings` | Robot-side settings file `/sdcard/biobuzz_teleop.txt` (see below) |
| `BioBuzzTune` | Live tuning of aim trim, windmill power and pause; counts balls per pocket (`BallCounter`); SHARE saves |
| `BioBuzzShotLog`, `BioBuzzVisionTest`, `BioBuzzSystemCheck`, `BioBuzzWiringCheck`, `BioBuzzCalibration`, `BioBuzzArmSetup`, `BioBuzzPinpointSetup` | Test and setup tools |
| `BioBuzzDiag` | TeleOp diagnostics CSV (`diag_teleop_*`, `events_teleop_*`) |
| `ArmCalibration`, `PinpointCalibration` | Saved calibrations `/sdcard/biobuzz_arm.txt` (deploy 0.16, dir +1), `/sdcard/biobuzz_pinpoint.txt` |

**Shot results** (9 Nov; the user asked for "trim ±5, an option to tell what made it vs not, in the diag, so the speed can be set automatically; it's off by ±10"):
- **TeleOp logs every launched ball** (event `BALL`), found as a flywheel dip by `BallCounter.forTuning()`. Each entry has:
  - the camera floor distance, range, bearing, CELL and tags (from the last sighting, at most 1 s old);
  - the speed source, base, trim, target and the table value at that distance;
  - the actual speed at launch (max of the last 0.35 s) and the dip;
  - ready/fallback, the aim gate, battery, the IMU heading since INIT and the Pinpoint pose.
- **LB + D-pad labels the oldest unlabelled ball of the last 10 s** (`SHOT_LABEL_WINDOW_SEC`): ↑ IN, ↓ SHORT, → LONG, ← SIDE. Event `SHOT_RESULT`; with nothing waiting, `SHOT_RESULT_IGNORED`. The screen's `Balls` line shows the counts.
- **`tools/shot_results.py robot_logs [since] [--write]`** (Shot Log IN shots count too):
  - For each table point it uses the balls within 3 in: offset = target − table(d).
  - The IN median sets the centre, kept above the SHORTs and below the LONGs. A point needs ≥ 3 labelled balls; the table is kept rising; values are rounded to 5.
  - `--write` puts the result into `AUTO_SPEED_TPS` (then rebuild and install). Tested on synthetic logs.
- Note: `AUTO_SPEED_HYSTERESIS` 10 can by itself leave the auto speed up to 10 off the table; the BALL log shows both values.
- **GRAB event** (`GrabTracker`, `master` `64152dd`, merged into `auto-work` `2c9b56e`; the user asked to log the moment the flywheel takes the ball):
  - Written when a counted ball's dip is over, linked to BALL by ball number: `pre` (real speed just before the grab: median of 0.2 s, or the last reading while still climbing), `launch_err`, `min`, `depth`, `energy` = 1 − (min/pre)², `recovery_s` (back within ±30), `below_s`, `drops` (2+ = two balls in one long dip), `overlapped`, `target_changed`, `windmill_ticks_left`, `feeding`, `since_pocket_end_s`, `amps_base` / `amps_peak`.
  - The launcher current (`BioBuzzRobot.launcherCurrentAmps()`, not in the bulk read, ~2–3 ms) is read only while feeding, for `GRAB_WATCH_SEC` 0.8 s after, and during a dip. Diag column `launcher_amps`.
  - BALL now also has `move` (max |wheel power|), `turn_cmd` and `aim_x`.
  - **Replay of the 7 Nov + 9 Nov logs** (`files/grab_replay.py`): 108 grabs, no false second drops. Energy median 0.40–0.45 (p10 0.23–0.39), recovery 0.16–0.28 s. **Launch error at the grab: about +15 on a good battery (p90 +30), −22 median on the weak battery (p10 −50).** The ±10 the drivers trim is largely the flywheel's own speed error (no I term, F not voltage-compensated). A real fix later: an I term or battery-compensated F in the launcher PIDF.
  - `tools/shot_results.py` fits on pre − median launch error (Shot Log: `vel_at_feed`), and leaves out (listing them) two balls in one dip, a target change, move or turn > 0.15, |err| > 60 or fallback, and weak grabs (energy < 0.75 × the IN median).

### `/sdcard/biobuzz_teleop.txt` (read by TeleOp, AUTO and Tune; survives reinstalls)

```
alliance=RED|BLUE              # TeleOp INIT: Circle/B = RED, Cross/A = BLUE; AUTO sets it too
spot_range_in / spot_speed     # SHOT SPOT: camera range to CELL opening + flywheel t/s (TeleOp SHARE)
spot_cell=RED_REAR|...         # (10 Nov night) the CELL that range was measured to; TeleOp/Tune drive to the spot only on it or
                               # its point-symmetric twin (Cell.otherAlliance); missing = old spot: aim only, "press Share again"
spot_ref=cluster               # spots saved before the cluster change are ignored
aim_trim_deg                   # + = shots LEFT (seen from behind the launcher)
feed_power / feed_pause_sec    # windmill power while turning a pocket / pause after each pocket
feed_empty_wait_sec            # TeleOp/Tune: after a pocket that launched nothing, the next turns after this (default 0.10; >= pause = off)
shoot_fwd_in / shoot_left_in / shoot_heading_deg   # taught AUTO shooting position (TeleOp LB+SHARE)
```

An agent can edit it directly over adb, with no reinstall:

```
adb -s 192.168.43.1:5555 shell "sed -i 's/^aim_trim_deg=.*/aim_trim_deg=0.0/' /sdcard/biobuzz_teleop.txt"
```

Each program reads the file when it is INIT-ed. The last known values (2 Nov) were:
- RED, spot 54.1 in at 1250
- pose −22.5 / −21.3 / −42.4°
- trim 0
- power 0.40, pause 0.25

### TeleOp controls (REV gamepad = PlayStation layout)

| Button | Action |
|---|---|
| Left stick | drive. **Robot-centric by default again since 10 Nov night** (`FIELD_CENTRIC_DEFAULT = false`, the drivers' choice; field-centric was the default 10 Nov, `master` `b2c4020` + review fixes up to `fcd4a8b`; `FieldHeading`). Field-centric (optional): stick up = away from the drivers; robot-centric while X / RB aims at a CELL in view (and 0.5 s after it drops out of view); L1 in INIT = on/off (static `BioBuzzMatchState.fieldCentric`, until app restart); Options tap = "intake points away now" (on release), hold 1 s = field-centric on/off |
| Right stick | turn |
| L1 | slow |
| R2 / L2 | intake in / out |
| **R1 hold** | **AIM + FIRE (10 Nov night, `master` `fa1f1a9`)**: with our CELL in view it aims and drives to the saved spot like □, at the camera shot speed (a D-pad preset/trim the driver chose is kept; only □ goes back to the camera speed), then fires. The speed follows the distance until the first ball of the burst, then is frozen until R1 is let go; the spot drive stops after the first ball. Aim/drive/speed all use the CELL `ShotSpeed` has locked (`aimTarget`). Without the CELL it fires as before. Also boosts the intake. Waits for READY (`LauncherGate`) and, with the CELL in view, for the aim (`AimGate`, shows FIRE HELD + why) |
| **□ hold** | aim at our CELL without firing; with a saved spot (on the same CELL or its twin) it also drives to the spot distance, then rumbles "ON SPOT - FIRE". The □ press (rising edge) also resets MANUAL → AUTO shot speed and the trim. The spot speed is used only with the auto speed off (and by AUTO) |
| △ | flywheel pre-spin |
| ✕ | flower arm |
| ○ hold | unjam |
| D-pad ↑ / ↓ | MANUAL FAR / NEAR (camera speed off until □) |
| D-pad → / ← | trim ±10 t/s since 10 Nov evening (±5 on 9–10 Nov, ±15 before; stays on top of AUTO) |
| LB + D-pad | shot result for the diag log (`ShotResults`, 9 Nov): ↑ IN, ↓ SHORT, → LONG, ← SIDE; goes to the oldest unlabelled ball of the last 10 s |
| **SHARE** (Back) | save the SHOT SPOT (distance + the CELL `ShotSpeed` has locked + speed; refuses without that CELL in view) |
| **L1 + SHARE** | also teach AUTO this exact position (Pinpoint zeroed at TeleOp INIT, so INIT at the AUTO start) |
| INIT: ○ / ✕ | RED / BLUE |
| INIT: SHARE | diagnostics on/off |
| INIT: □ | aim gate on/off (off = FIRE never waits for the camera) |
| INIT: △ | auto shot speed on/off (off = NEAR / FAR presets only) |

### AUTO (`AutoRoute`)

**Route:** DELAY → LEAVE (back 12 in) → [GOTO the taught pose, square to the start] → AIM (turn to `AUTO_AIM_HEADING_DEG` = −50 or the taught heading) → FIRE → SPOT BACK → RETURN (to 4 in from the wall) → PARK (strafe 14 in right).

**FIRE:**
1. Camera aim (tolerance 1.5°). If there's a spot but no taught pose, it also drives up to 24 in to the spot range.
2. 0.5 s settle while **holding the aimed heading with the IMU**.
3. `ShotFeeder` from settings, intake boost, up to 12 pockets. It stops at 4 balls, counted from flywheel dips of ≥ 60 t/s.

**Heading** is the **hub IMU zeroed at START** (no `resetYaw`: it applied late, and once gave −30° at the start). The Pinpoint is used **only for distance**. If its heading differs from the IMU by more than 15°, it's dropped and AUTO falls back to the wheel encoders. That happens when the Pinpoint gyro calibration is bad because the robot was moved after INIT.

**INIT options:** Square/X = shooting off (LEAVE + PARK only), D-pad ↑/↓ = start delay (0–10 s; the wait happens *before* LEAVE), Y = camera off.

**Team rule:** place the robot, then INIT, then don't touch it until START.

## 5. What we learned from robot logs (most important findings)

1. **Flywheel speed is fine.** At feed it is 1240–1280 t/s, with dips of 40–130 that recover quickly (two wheels).
2. **Aim was the main cause of the 5 % match success (1 Nov).** At the shot, the camera bearing had a median of **−11°** (−5…−20°), almost always to the same side. □ auto-aim was held on only 12 of about 470 shots. Distance was consistent (53–58 in) and the robot wasn't moving.
   - **Open question:** are the drivers aiming off, or is the camera mounted about 10° off from the launcher? The students' Tune result of +9° trim hints at the camera.
   - **Suggested test:** from the spot, hold □ and fire 10 balls.
   - One AUTO run with +9° trim missed LEFT, so the trim was set back to 0.
3. **Windmill feed:** in matches 70–90 % of pockets launched a ball (earlier about 50 %).
   - Stopping exactly on the pocket boundary left balls pressed against the flywheel (the windmill was pushed back about 15 ticks).
   - A continuous feed was too fast.
   - Power 0.6 → 0.45 → **0.40** (the user wanted slower; 0.45 already showed 4 jams). **Don't go below about 0.35.**
4. **Pause 0.25 s is too short.** Balls 2–4 fire before the flywheel recovers, so they land short. **Suggested 0.6 s** (only the first ball scored in the 31 Oct AUTO). **Not applied yet.**
5. **AUTO with a taught pose worked:** the right position, 3 of 4 balls launched, the first ball scored.
6. **Driver Hub "hangs"** were caused by: the obsolete app (fixed by moving to 12.0), the laptop on the robot Wi-Fi, and a Control Hub that wasn't booted or was frozen (the 37102-RC SSID was missing; power-cycle it).
7. **SDK 12 cluster `fieldPosition` is (0,0,0) for all HIVE clusters, and there are no wall tags this season.** The camera **can't** give the robot's field position (the HIVE moves).
8. **Shot geometry review (3 Nov; field CAD + 1 Nov logs, checked by Opus 5.5 and GPT-6.1 reviewers):**
   - SDK 12 `ftcPose.range` = `hypot(x, z)` in the camera frame (no vertical part; checked in the bytecode). With the 30° camera tilt, horizontal distance = (r·cos(bearing) − 47.1·sin 30°) / cos 30°.
   - So the old spot was about 33 in out (not 28), but about **45° off the CELL's facing direction** (taught pose −42.4°). From there the ball starts only ~24 in in front of the opening.
   - CELL opening (match start): 14 in long, lips 53.5 / 65.1 in high, leans back 56° and faces the robot.
     - A 66° launch only works straight-on with the ball exit ≥ 34 in out, and the ball is still rising at the opening.
     - **74°** gives ±4.3 % speed tolerance from 30 to 42 in out, and the ball comes down into the CELL.
     - Straight-on, the robot's front at the wall puts the ball exit 38 in out. Topspin from the hood needs ~2–8 % more speed.
   - **Aim was a main cause too:** 285 of 447 logged feeds were outside the opening sideways.
   - Fix: launcher hood v2 (§7), shoot **straight-on with the front ~2 in from the wall**, and gate FIRE on aim.
9. **Flywheel physics check (4 Nov; CAD + logs + flight simulation with drag and spin):**
   - **Inertia:** Hogback wheel 1.31e-4 kg·m² (CAD); hub 0.05e-4; motor rotor about 0.3e-4. Two wheels total about 2.9e-4, consistent with the logged spin-up times.
   - **Spin:** the hood is on the outside of the curve, so the ball leaves with **topspin**, and Magnus pushes it toward the CELL and then down.
   - **Required speed at 74° from 36 in:** with drag, about 5.5 m/s (vacuum 5.3), still ±4.6 %. That is about **1100–1200 t/s**: half the tread speed, plus slip and the dip.
   - **Energy:** about 1.3 J per ball (the ball takes 0.63 J). Dip about 7–8 % with two wheels.
   - **Recovery is limited by the hub P gain, not the motor.** P = 40 adds only about 2 W, so recovery takes about 0.6 s; the motor (22.6 W max) could recover in about 0.1 s.
   - **Velocity resolution** is 20 t/s steps (about ±1.7 %), so the FIRE gate needs several readings in a row.
   - **No I term**, so the speed droops about 20 t/s as the battery goes from 13.1 to 12.4 V.
10. **Flywheel code for the printed hood (5 Nov, `master` commits e3f9692, f80911c, 35efca3, c5488e8; local only, 81 unit tests):**
    - `tools/shot_model.py` (drag Cd 0.5 + Magnus topspin, CELL lips from the field CAD): 5.52 m/s scores from about 28–42 in (ball exit to the opening, horizontal), 5.94 m/s from 46–54 in → **NEAR 1150 / FAR 1240 t/s** at a ball-speed factor η 0.89 (old-hood data said 0.86–0.89; calibrate with BioBuzz Shot Log).
    - P 40 → **100** (recovery ~0.2 s; oscillation only near P 400). Ready gate, aim gate, ball counter: see the §4 table.
    - SHOT SPOT and AUTO pose use the camera's **floor** distance (`horizontalIn`); `spot_ref=floor` in the settings file. Spots and AUTO poses saved with the old hood are dropped: **save the spot again, re-teach AUTO (LB + Share), re-check the aim trim in Tune.**
    - Reviewed in 3 rounds by Opus 5.5 (final: GO) and GPT-6.1 (final: GO WITH FIXES → the armed-noise ball count fixed in c5488e8).
    - Driver card + shooting guide updated to the new speeds and FIRE HELD (9073906).
    - Not done: a "straight-on" check from the cluster yaw (unclear for a tilted camera). `field-tour` has these changes since the merge `198208d` (5 Nov).
11. **Windmill feed and hand-over (5 Nov night; robot logs 31 Oct – 1 Nov, old hood; reviewed by Opus 5.5 + GPT-6.1, both SHIP after 2 rounds):**
    - `tools/windmill_logs.py robot_logs [since]` (de-duplicates the log copies; reads the aimed boundary from the diag note `FEEDING <ticks>` since `8967f09`, older logs: from the ramp power). Old logs: 832 pushes; **157 forward strokes > 1.5 pockets (47 > 2, longest 7)**; 38 stalls; **33 JAM, 6 JAMMED**; first dip during the push 198, ≤ 0.15 s after the stop 110, **late 40**; maybe two balls: 7 two-dip pushes, 24 dips deeper than 160 t/s (one ball: p90 120 t/s). Long dips (> 0.45 s: 96) are not comparable: the old P 40 flywheel recovered in ~0.6 s.
    - **Causes found in `ShotFeeder`:** every FEEDING start did `target += pocket` even if the last pocket was cut short (jam) or the windmill had gone back (auto back-off 0.25 s at −0.35 = **1.5–3.3 pockets**, or the driver's B), so the next push caught up 2–7 pockets (and after a jam could feed the stuck ball + the next). Stalls: the ramp is 0.15–0.2 power with 26–48 ticks left; the windmill stopped there (wedged ball / friction, not ball weight: 0.07 N needed vs 4–5 N available) and waited for the 1 s timeout.
    - **Fix `8967f09`** (see the §4 table). Pitfalls the reviews caught: a time-only or once-per-loop distance cap is ~2× longer in reality (60–76 ms loops + braking); full power or kept extra power after a stall overshoots 40–60 ticks (can feed the next ball); "stuck close to the boundary = done" can skip a ball that never left; aiming > ~2 pockets back over pockets turned past the intake can feed several balls (two balls 0.1–0.3 s apart make ONE dip: check dip depth, not just the dip count).
    - **Hand-over model** `goBildaStarter/handover_sim.py` (`d257110`): ball path from `ball_feed_model.py` with sphere-on-heightfield heights, 1-D dynamics (hollow ball k = 2/3), all four vanes as 3-D obstacles (the vane in front stops a thrown ball), continuous encoder timeline, motor fitted to the logs (2200 p − 30 t/s, 15 ms lag, 12 ms brake, 60–76 ms loops). The current ramp puts the ball into the flywheel during the push at **98–100 % of stop angles** (73 mm: 100 % even 30 % slower; 71 mm: 96 % at 30 % slower, stop angles 62–64). The vane lets go 0.1 mm (73 mm ball) / 7 mm (71 mm) before the flywheel's first touch. **Unconfirmed on the robot:** check the first practice logs for late dips and two-ball pushes.
    - TeleOp loop while feeding: median 60 ms, p90 76 ms (overall median 39 ms; bulk caching AUTO already on). At 0.40 the windmill turns ~750–850 t/s ≈ 47 ticks per loop, which is why the ramp starts slowing at the push start.
12. **The last (lone) ball (6 Nov; user video 16:37: the windmill hits it to the wall, it takes long to reach the flywheel):**
    - `goBildaStarter/lone_ball_sim.py`: 2-D dynamics on the floor STLs (sphere-on-heightfield centre map, Coulomb friction E 0.45 / MU 0.35, rolling resistance 200 mm/s², vanes as 3-D boxes on the ShotFeeder timeline, intake roller ridge across x −128..128 + kick-back). Matched the video only with Coulomb friction (per-step damping glued the ball to the vane).
    - **Cause:** the pinwheel vanes (22 mm offset) push a ball ~12–20° outward; with 3 balls the balls/walls hold each other, a lone ball is thrown (0.8–1.5 m/s) onto the open left floor (x 45–93 for its centre, out of reach: the tips reach a ball centre within 114.5 mm of the axis), rolls back down the side slope onto the tips and is thrown again. Also a dip between the roller (top z 0, back y −120) and v1's front edge (y −100, z −5.5 at x 40) where a ball can sit out of reach.
    - **What didn't help (model):** taper alone (10°: 57 % / 27 % vs 50 % / 29 %; an earlier "78 %" came from a model bug: roller only to x 64); lower windmill power (0.15–0.30); slow velocity pushes (100–300 t/s: the ball no longer reaches the flywheel); thin walls that leave a pocket behind them (intake balls at x > 47 trapped). The printed rotor PPW1 has the same pinwheel pocket walls.
    - **PPR1 v2 tried and rejected** (§7): a guide wall closing off the left floor + floor to the roller + 6° taper. Lone ball 50 % → 96 % fed ≤ 2 s, but both reviews (Opus 5.5, GPT-6.1): DON'T PRINT. Main reasons: capacity (§7) and vane-tip locks on the wall.
    - **Model corrected after the reviews:** `goBildaStarter/multi_ball_sim.py` resolves walls, vanes and balls together. The old `lone_ball_sim.trial()` corrected them one after the other: a vane could push a ball 9 mm into a wall and still count it fed. It also has friction locks (normals > 141°), BALL_R, the intake side channels (x ±104..124), 3 balls (queued at the intake) and parked capacity. Earlier pitfalls in my tests: entry starts inside the wall/channel, `lone_ball_sim.R` ignored BALL_R, firing time was 5.4 s, not 6.
    - **The pause is the cause** (v1, lone, 90 random + 45 video starts, fed ≤ 2 s / never in 6 s):

      | Pause | Lone ball | Video start (never) |
      |---|---|---|
      | 0.45 s (video) | 50 % / 37 % | 31 % |
      | 0.30 s | 54 % / 32 % | 36 % |
      | 0.10–0.20 s | 67 % / 18–19 % | 22 % |
      | none | 79 % / 6 % | 9 % |

      73 mm balls are similar (none: 76 % / 11 %). During a pause, the thrown ball comes back and rests on a vane tip; the next push flicks it out again.
    - **Code fix: DONE 9 Nov** (see §5.14 and `ShotFeeder` in §4): no pause after an empty pocket, stop on a dip. Originally proposed here as: after a pocket that launched nothing (no dip in its window), go straight into the next pocket instead of pausing. Stop as soon as a dip shows, then the normal pause + ready gate.
      - Why not just a short pause: below ~0.3 s two balls can share one dip and the second ball is fed into a dipping flywheel (the ready gate is only checked between pushes).
      - Quick field check meanwhile: Tune pause 0.1 s, one ball, FIRE; then set it back to ≥ 0.3 (don't SHARE).
    - Reset windmill power to 0.40 if lowered.
    - **Printed windmill PPW1 reviewed (7 Nov): not better; don't print as is.**
      - **Lid clash:** its top plate (z 66–70, 6.5 mm lower than the polycarbonate rotor's 72.5) overlaps a ball at the PPL1 hood entry by 0.2 mm (71 mm) to 2.2 mm (73 mm). PPW1 predates the printed hood. Fix before any print: `ROTOR_Z1` ≥ 74 or trim the lid.
      - **Same ball handling:** paddles on the same lines. Model (`ROTOR=ppw1`): pause 0.45 gives 52 % / 26 % (polycarbonate 50 % / 37 %), video start 29 % / 44 % (38 % / 31 %); no pause 68 % / 18 %; capacity the same.
      - **Stiff PETG:** wedges stall (JAM) where polycarbonate tips bend. The home sensor PPW3 needs code (none exists).
      - Warnings added to `POLYCARB_REPLACEMENT_PLAN.md` Phase 2 and `print_master/README.md` (Job1).
    - **Capacity depends on the rest angle** (both rotors, model): 4 balls at one angle per 90°, 2–3 elsewhere. The ShotFeeder grid origin is wherever the rotor is at INIT (`nextTarget` steps from the initial `targetTicks`). Find and mark the 4-ball angle on the robot and set it before INIT; or add homing.
    - **Front-left gap (9 Nov, user photo + CAD circle):**
      - **Where:** a lone ball sits still in the 16 mm gap between the intake's lower roller (Ø16, axis y −128, top z 0, back y −120) and PPR1's front edge (y −104). It is at the left end, against the roller's end hub: centre ≈ (68, −116), 156 mm from the windmill axis (reach ≈ 115), so the vanes can't touch it.
      - **Why:** the ball rests on two edges; along the roller its resting line is level (C 33.8 flat for x 20–80). PPR1's front edge is at or above the roller top for x ≥ 80 (z −2.9 at x 60, 0 at 80, +3.2 at 100), so the front-left floor drains into the gap.
      - **The model doesn't reproduce the stick.** `files/front_left_fix.py` (session folder) uses `multi_ball_sim`. Balls dropped there roll into reach even with 3–4° rolling resistance, so the real corner is deeper or stickier (hub, wiffle holes on the edge, real gap or heights). The model's roller only pushes a ball sitting on top of it (y < −118) inward; a ball behind the roller is never touched by it.
      - **Proposed (not built): a gap-filler strip** on PPR1's front flange (3 mm, `FRONT_FLANGE`). It carries the floor forward to about 2 mm from the roller for x ≈ 15–100 on the floor's own side slope, with a fillet at the hub. Model checks: capacity at 60° stays 4; single-ball entry from x 30–66 still ends in reach; lone-ball feeding unchanged (27/36 starts fed within 6 s). A 3 mm higher front (`FILL_RAISE=3`) made no difference in the model.
      - **First:** tape-test a card strip in the gap. The no-pause `ShotFeeder` change addresses the other lone-ball problem (vane throws), not this one.
      - **Done (10 Nov night): the tape worked** (user photo: duct tape over a small polycarb strip, "ball does not stay there like earlier") → **PPR4 printed filler designed** (goBildaStarter `main` `c978bb7`, rail-screw mount; 11 Nov `c8b796f` tape mount; 11 Nov `f1c37c2`/`76d0001` **3 countersunk screws from above into PPR1** (current); `stl_floor/README.md` PPR4; details §7).
13. **One flywheel wheel + printed windmill (7 Nov noon; logs `robot_logs/20261107_193100`, script `files/flywheel_logs.py` in the session folder):**
    - The user fitted the printed windmill anyway ("did great": balls reach the flywheel much faster, median gap 0.6–3 s vs 5–8 s) and removed one flywheel wheel. Accuracy ~75 %.
    - Jams unchanged: 36 JAM in 329 pockets (11 %), 8 JAMMED; session 13:06 had 20 manual unjams.
    - Drivers trimmed every session. Good battery (12:51–13:12, 78 balls), target ≈ actual: 28–36 in ~1440, 36–44 in ~1450–1480, 52–60 in ~1585–1600 t/s (two wheels: 1150–1240). One wheel ≈ 25 % more t/s (η ≈ 0.70 vs 0.89). 2 balls at 66 in at 1525 contradict the model: the table above ~57 in is unverified.
    - **Weak battery** (11.9 V falling to ~6 V, 12:40/12:43): 36 READY_FALLBACK, 24 of 56 balls left > 30 t/s slow. Steady error +15..+30 t/s at 13 V, −20..−40 at ≤ 11 V (no I term). Dips 300–350 t/s per ball, recovery 0.1–0.4 s.
    - **Auto shot speed** (`b1cee74`, see `ShotSpeed` in §4): drivers lost time trimming by hand.
      - Replay of all logged camera distances: two of our CELLs in view (6 Nov 15:40/15:43: REAR 30.6 in, 4 tags, and AUDIENCE 65.8 in, 3 tags, both ~1° bearing) made `HiveVision.target()` (most tags wins) flip every few frames: 54–114 speed changes per session without the one-CELL lock, 5–7 with it. Total over 36 min of launcher-on: 5.8/min → 1.5/min. The aim and the SHOT SPOT distance still use the flipping best CELL (unchanged).
14. **"Windmill speed is low" (9 Nov, drivers; same 7 Nov logs):**
    - FIRE-hold time: windmill turning 27 %, paused while the flywheel was ready 42 %, waiting for flywheel recovery 23 %, jams 7 %. 2.7 pockets per ball (367 pushes, ~136 balls); push median 0.20 s; ball-to-ball median 2.07 s.
    - **Don't raise the windmill power:** < 0.1 s per ball, and more jams and two-ball feeds (power was cut from 0.6 for that).
    - **Dip timing** (`files/dip_timing.py` in the session folder; first loop with target − vel > 30): 58 % of balls dip the flywheel while their own pocket turns (detected at a median 2/3 of the travel, 0.10 s before its end), 19 % at the end, 13 % 0.01–0.08 s after, 10 % 0.08–0.2 s after the windmill stopped.
    - **Done: skip the pause after an empty pocket** (see `ShotFeeder` in §4; robot repo `master` `1ff7495`, 9 Nov; installed 9 Nov ~16:22 as part of `auto-work` `b06abaf`; 118 tests; rubber-duck review found FIRE-release-during-HOLD and the resume-loop jam timer, both fixed). `RealPocketSim` in `ShotFeederTest` (real motor model, 60–76 ms loops, this dip timing) gives the trade-off for the empty wait (final code, 200 runs, 2897 balls). 0.05 s: 18.7 % less time per ball, 1.6 % of balls go into a slowed flywheel (> 200 t/s low, sure misses). 0.07 s: 13.7 %, 1.1 %. **0.10 s (default): 10.6 %, 0.45 %.** 0.15 s: 2.6 %, 0. Plain: 0 slow. The slow balls come from a pocket's own ball going in while a late ball from the pocket before is still on its way; HOLD can't stop that. Holding on a pocket's own dip costs the restart of the rest of the pocket (~0.1 s).
    - Tunable on the robot: BioBuzz Tune LB + D-pad ←/→ (0.05 steps, 0.05–0.5; ≥ pause = off), saved as `feed_empty_wait_sec`. TeleOp's CONFIG log shows `feed_empty_wait`. `tools/windmill_logs.py` counts `NO PAUSE` / `HOLD` (a held pocket shows there as two pushes).
15. **"TeleOp moves by itself, hard to control" (9 Nov evening; logs `robot_logs/20261109_194432`; scripts `files/teleop_motion_check.py` and `files/teleop_motion_drill.py` in the session folder):**
    - Only BioBuzz TeleOp and the Pedro BioBuzz AUTO BLUE ran (RC log; no Field Tour or Pedro tuning). Logged wheel powers matched `MecanumMath` of the sticks on every row, except while X (aim) was held. The code drove nothing by itself.
    - **The gamepad's left stick no longer centred.** It rested at ly +0.031..+0.055 and lx ~0.008. On 7 Nov the sticks read exactly 0 on 99–100 % of near-centre rows; on 9 Nov only 7–21 %. TeleOp has no dead zone, so the wheels got about ±0.04 instead of 0: no BRAKE, and stops after release took 0.25–0.34 s instead of 0.20 s.
    - **Drivers used full-deflection flicks**, mostly full-speed spins (4–15 % of rows vs 0–5 % on 7 Nov). Each flick fell between 70 ms log samples, and the wheels spun down over ~0.4 s.
    - **Correction (10 Nov): not glitches.** Those were 0.5 s IMU samples (`DIAG_SLOW_SAMPLE_MS` = 500) during fast turns, plus **the IMU yaw flipping by 180°**. The wheel encoders (logged every 50 ms) show each big jump = the real turn ± 180° (fit 0.02155°/tick, median error 4°). The yaw mostly stays within ±90° and flips where it crosses: ~1290 flips in the diag logs 30 Oct – 9 Nov (32 on 9 Nov), on every practice day; 31 Oct still had some smooth crossings past ±95°. Cause not proven: maybe a mounting/orientation mismatch near an Euler singularity (a constant mounting error alone only adds an offset). **Check on the robot:** BioBuzz Calibration, turn the robot slowly by hand past 90°; if the heading jumps 180°, check the hub's real logo/USB directions against `HUB_LOGO`/`HUB_USB` (and recheck after the electronics v2 move). **Affects:** OLD AUTO (IMU heading, and it drops the Pinpoint if the two differ by > 15°), LB + SHARE "teach AUTO position" (refuses when Pinpoint and IMU differ by > 5°), BioBuzz System Check turn test. Field-centric TeleOp corrects the flips (`FieldHeading`).
    - **Offered:** an 8 % stick dead zone (+ optional gentler turning near centre). The user said "TeleOp worked, no need to tweak now", so nothing was changed. If it comes back: swap or re-pair the gamepad (don't touch the sticks while pairing), then add the dead zone.

## 6. Docs and team artifacts (robot repo)

- `BioBuzz_Driver_Card.pdf` (`tools/driver_card.html`): a 2-page visual card for students (gamepad picture, 5 scoring steps, pit-crew SHOT SPOT steps, fix-it table).
- `BioBuzz_Shooting_Guide.pdf/docx` (`tools/make_shooting_guide.js`): the long coach guide (5 pages).
- `BioBuzz_Wiring_Sheet.pdf/docx` (`tools/make_wiring_sheet.js`).
- `README_2.md`: setup, deploy, op modes, SHOT SPOT, Tune, teaching AUTO.
- `BIOBUZZ_TEST_PLAN.md`.

## 7. CAD / printing (goBildaStarter)

- Flower arm v3.2–3.4 (arms v3.4: 5 mm shorter, hub Ø33). Camera mount, windmill parts, REV battery sleeve PPB1 v2, robot signs.
- **Robot signs** (`stl_sign/`, `make_robot_sign.py`, `make_sign_ams_jobs.py`):
  - blue and red, with white 2.25 in numbers (R401–R403)
  - jobs `bambu_p2s/Robot_Sign_37102_{BLUE,RED}_x2_P2S_PETG_AMS_*.gcode.3mf`
  - **Send dialog:** filament 1 = background colour slot, filament 2 = white
  - The red signs were printed on 31 Oct (one white copy, painted red)
- **Bambu P2S:** sign settings are 3 walls, 100 % infill (**Rectilinear**; zig-zag doesn't allow 100 %), PETG 250/70 °C, Textured PEI, no supports.
- **Bambu Studio project rule (found 4 Nov, checked in the 02.09.01.70 source):**
  - When the GUI opens a 3MF, every setting that is **not** listed in `different_settings_to_system` resets to the system preset (`PresetCollection::load_external_preset` → `update_non_diff_values_to_base_config`).
  - Older jobs made from a template with a custom `print_settings_id` and an empty list show 0.20 mm / seam Aligned and so on when opened. Only the CLI-sliced G-code inside them has the right settings.
  - Correct way (`make_launcher_hood_job.py`):
    1. Resolve the system presets from `C:\Program Files\Bambu Studio\resources\profiles\BBL\{process,filament,machine}`: parent chain, then `include` templates, then the preset's own keys.
    2. Write full JSONs, adding the changes and `different_settings_to_system` to the process JSON.
    3. Run `bambu-studio.exe --load-settings "machine.json;process.json" --load-filaments filament.json --curr-bed-type "Textured PEI Plate" [--slice 0] --export-3mf X STL`.
    4. `--curr_bed_type` (underscore) fails with exit −2.
  - Clicking in the GUI from PowerShell: call `SetProcessDPIAware` first (the display is at 125 %).
- **Field CAD:** `field_cad/x/field-cad-step.step` (35 MB). It can be read in Python with `cadquery-ocp` (installed: `from OCP.STEPCAFControl import STEPCAFControl_Reader`). Units are mm, **Y up**, floor = X–Z.
- **Launcher hood v2 (PPL1), 3 Nov:** `launcher_hood_v2_build.py`, `stl_launcher/README.md`.
  - **Print job (redone 4 Nov):**
    - project `bambu_p2s/Launcher_Hood_v2_PPL1_P2S_PETG.3mf` (open, slice, print) plus the sliced `.gcode.3mf`;
    - base 0.16mm Standard @BBL P2S plus 5 walls, 40 % gyroid, tree supports from the plate only, outer brim 6 mm, seam Back;
    - 10 h 10 min, 232 g;
    - verified in the GUI tabs.
  - **Print standing**, not on its side. On its side about 7,000 mm² of supports would stand on the inner wall (ball path). Standing, the supports are only under the floor and the side blocks: 11.8 g, at most about 20 mm tall (from the G-code).
  - **Replaces:** the polycarbonate hood G, the side plates C and the zip ties.
  - **Ball path:**
    - old floor shape (cubic fit, rms 0.47 mm) up to −80°;
    - then a smooth Hermite ramp to a 7 mm squeeze at −50°, about 11° convergence where the ball first meets the wheel;
    - held to −42°, then a gradual release on R 210, then a 74° lip.
    - Exit at y 149.5, z 175.5 (228 mm above the tiles). Ball plane x −2. Channel 77 mm.
  - **Mounting:**
    - 8 M4 heat-set insert holes at (80,8), (96,8), (96,16), (104,16) per side. **Found 5 Nov after printing: (96,16) faces a solid bar of the tower's inner side wall** (between the Ø14 bore at (88,16) and the slot y 98–106) → only **3 screws per side** (80/8, 96/8, 104/16); guide + README updated. Cause: hole positions were taken from cylinder-face centroids, which are off-centre for arcs of merged holes. **Always verify hole positions with rays through the meshes (Ø3.8 ring test)**;
    - 0.35 mm gaps to the towers and the cross channel;
    - a front tab on plate F under the windmill-zone post.
  - **The towers' 4 base screws** (2800-0004-0010:63/64/67/68 at y 72/104, z 0, into the quad blocks 1201-0043-0002:4/5) **stay**. The hood has Ø8.5 pockets for their heads, with 45–55° relief slots.
  - **The script also adds flywheel wheel 2 to the CAD.**
  - **Fitting:** diagonally from the back at 45–55°, with the hub covers, hub brackets and front post off.
    - Checked with a mesh sweep and with `hood_fit_hardware_check.py`, which includes the screws.
  - **Reviewed before printing (3 Nov) by Opus 5.5 and GPT-6.1:**
    - base screws, gaps, a too-steep nip (was 31°), and knife edges were all fixed;
    - still to test: wheel 2 pushes the ball toward the +x wall, and the stronger squeeze load on the motor shaft.
  - **Printed 4 Nov** (10 h 9 min, 231.8 g). Mounting guide: `Launcher_Hood_Step_by_Step.docx/.pdf` (15 pages; `guide_hood_steps.py` → `guide_hood_annotate.py` → `make_hood_steps_guide.js`, run with `NODE_PATH=%TEMP%\guide_build\node_modules`). **PDF export:** Word COM `Documents.OpenNoRepairDialog(path, False, True, False)` + `ExportAsFixedFormat(pdf, 17)` takes seconds; plain `Open` hung (hidden repair dialog).
- **Floor plates B, D, E, F → printed PPR1–PPR3 (user's decision 5 Nov: "remove all polycarbonate").** B, E, F were zip-tied to the old hood G (B loose, F hanging, E's back edge free with PPL1).
  - `make_floor_parts.py` (manifold3d, E/F surface fits + PPL1 front-floor profile embedded), `floor_parts_check.py` (robot meshes from the session folder: clearances, windmill, ball steps, screws/nuts, fitting order), `make_floor_parts_jobs.py`, `floor_parts_fusion.py` (PPR1-3 into the Fusion design as component `Floor_Printed_v1`, hides B/D/E/F + 7 zip ties + the 3 fence standoffs, hides standoff 2/3 screws and washers, moves standoff 1's screw (tab clamp) onto PPR2's underside; re-runnable via attributes `FloorPrintedV1`), `floor_parts_renders.py` (Fusion; uses `Floor_Printed_v1`; whole-robot + close-up pictures), `stl_floor/README.md` (pre-print checks, parts, print, fitting), `POLYCARB_REPLACEMENT_PLAN.md` (every polycarbonate part).
  - **Fusion saved as version 22 (5 Nov, user OK'd):** printed hood `Launcher_Hood_v2` + flywheel wheel 2 + `Floor_Printed_v1`. Before that the last save was v21 (30 Oct); the hood had only been in the open session.
  - PPR1 front floor (E + front of D): **clips onto PPL1's front floor edge** (cap on top 1.6→0.8 mm + tongue below, 0.25 mm play, x −40..31) → the ball steps 1.05 mm DOWN onto the hood and the joint can't sag; 4 × M4×12 at rail holes (y −88/−32, z −16) through a left skirt / right lip, nyloc nuts behind the rail walls reached from the rails' open bottoms (the round drive motors leave 13–17 mm there); rib under the back edge (its top follows the floor's underside: fixed 5 Nov night, see below); back edge beside PPR2 = PPR2 + 0.5 mm; 4 holes for the printed windmill post (surface kept = E there). Fits last: up from below 5 mm forward, slide back onto the hood. Stands on its front edge, diagonal: 2 h 56 min, 123 g.
  - PPR2 left entry floor (F + back of D): **all 3 fence standoffs replaced by a built-in ball fence** (5 Nov afternoon); PPL1's front tab clamped by M4×14 up through PPR2 + tab, washer + nyloc on top (standoff 1 removed too: it stood in the hood entry, and in the feed model it doubled the loose-ball pinch against a vane, 1.55 vs 0.79 mm, with 9/9 still fed without it); 2 × M4×14 + nyloc into the left rail (y 16/40, z 16); 2 inserts (Ø12 bosses) at the left tower face holes (60, 8)/(76, 8), M4×10 from inside the tower. PPR3 right entry wall (B): inserts at (−60, 8)/(−76, 8). PPR2 + PPR3: 2 h 36 min, 82 g, no supports. Order PPR2 → PPR3 → PPR1, all from below, PPL1 and the hubs stay in.
  - **Ball fence / feed model (5 Nov):** user asked why the "pillars" (fence standoffs) and what deflects the ball into the flywheel ramp. Built a quasi-static feed model (`goBildaStarter/ball_feed_model.py`: stock polycarb vanes = 4 offset flat plates (pinwheel, tips r 79), CCW from above = toward the launcher; R 35.5 ball on the real floor STLs; gravity or no-roll-back). Findings: a fed ball is pushed along PPR3 + the hood's right wall, never touches the posts, ends ~3 mm short of the flywheel crown (flywheel first touches a 71 mm ball at y ~67, 73 mm at y ~59; vane reach ends y ~52–56) → the ball must coast in (windmill speed at release matters; ShotFeeder ramps power down over the whole pocket; **checked 5 Nov night with the dynamic `handover_sim.py`: the ramp is fast enough, see §5.11**). The posts' real job: loose balls on the left floor (it slopes toward (−0.9, +0.44)) roll into the hood entrance's front-left corner (~(33, −8)) where the vane tips only shove them: no fence 2/9 fed, posts / thin V / solid fence 9/9; round r6 nose worse (extra pocket). Built into PPR2 as a solid block (front face = old posts 3→2 line at z 26, back corner (90, 63.4) on the tower face; prints back_down with no supports). **Fence reviewed by Opus 5.5 + GPT-6.1: both PRINT**; applied: front edge z 46 (a ball touching the face sits 37 mm in front on higher floor; z 40 was only ~1.5 mm above its centre; the old posts were 1.5–4 mm head-on), top sloped 15° up to the back (z 55, so no ball perches under the flywheel motor), standoff 1 → tab clamp, pre-print checks (wires / protruding screw heads on the left tower face: the hole column nearest its left edge, ~44 mm in from the left rail's inner face x 128.5, 20–50 mm above the floor; short-arm hex key for the rail screws), sand the stair-stepped front face if a ball hangs. PPF1 (old printed fence) dropped from the windmill guide (now 11 steps), print master Job1, shopping list, BOM. **Fusion saved as v23** (PPR2 with fence, standoffs + PPF1 hidden, standoff 1 screw = tab clamp). Pitfalls: gravity roll-back in the quasi-static model creates fake "pinches" at the end of the push; floor seams must not count as walls; Fusion STL export skips hidden bodies; Bambu CLI rotates parts on the plate (read the 3MF item transform to map G-code back); a post's margin depends on the approach angle, a flat wall's doesn't.
  - Reviewed by Opus 5.5 + GPT-6.1 (5 Nov): both "print after fixes" (joint registration, zip-tie slop/blind threading, seam, windmill post seat, bosses, screw lengths) → all applied → final check: **both PRINT**. Extras from the final check, also applied: a 25 min clip test print (`Floor_PPR1_Slot_Test`: 0.25 vs 0.35 mm play; if only 0.35 fits, set `SLOT_GAP = 0.35` and rerun), PPR1 screw holes slotted ±1 mm, "take PPR1 off (5 mm forward) before removing the hood", threadlocker on all 6 hood screws.
  - **goBildaStarter: nothing committed** (floor parts, hood guide fixes, plan, README). Ask the user before committing (many old untracked zips in the repo).
  - **Clip test result (5 Nov evening):** both pieces went on the hood's edge; narrow (0.25 mm play) = perfect fit, wide (0.35) a little loose → PPR1 printed as designed, `SLOT_GAP` stays 0.25. PPR2 + PPR3 sliced by the user in Bambu Studio (2 h 36 min, 81.77 g, same as the CLI slice); PPR1 opened in a second Bambu Studio window (`single_instance` is false) for after it. PPR1's 8 mm brim reaches x 0.6..255.5 on the 256 mm bed (part 8.3..247.7): fits, but the plate must be clean edge to edge.
  - **PPR2 + PPR3 printed (finished ~21:20, 5 Nov).** Printer "RoboGenie 3d..." (P2S) HMS: motor noise cancellation needs recalibration; threaded (Z) rods need lubrication now. AMS: A1 PETG natural, A2 PETG blue, A3 PETG red, A4 empty; external spool PETG.
  - **PPR1 "floating regions" fix (5 Nov night):** Bambu Studio's GUI slice warned "PPR1_Front_Floor_print.stl has floating regions" (the CLI slice doesn't report it; my earlier G-code check only looked for support features). Cause: the back rib (`BACK_RIB`, x 31 → left wall, under the back edge beside PPR2) had a flat top at the floor height of the back edge, but the floor is up to 0.88 mm higher at the rib's 45° front tip, so in the standing print (z = robot y) the rib's first layer (layer 375, y −25.2) was printed in mid-air, joining the floor only 13–37 layers later (x 31..100). Fix: the rib = old prisms (same 45° face plane and bottom, top raised) ∩ `slab(ppr1_top − T + 0.5, 40)` → its top follows the floor's underside 0.5 mm inside it. +0.29 cm³ all in the rib region, 1 body, bounds unchanged, `floor_parts_check.py` unchanged, 0 islands (checked by slicing with manifold3d: components with no overlap with the layer below grown 0.2 mm), GUI slice without the warning: 2 h 56 min, 122.73 g. Per-segment curved tops made zero-volume slivers at strip joins (decompose = 3): the slab intersection avoids that. **Island check recipe:** `M = part.rotate(...)` so print z is up, then per layer `cs.decompose()` vs `prev.offset(0.2, m3.JoinType.Round)`. **Fusion saved as v24** (fixed PPR1 reloaded by `floor_parts_fusion.py`). Bambu Studio "Slice plate" button sits in a WS_EX_TRANSPARENT panel: find it with EnumChildWindows by text "Slice plate" and post the click to it.
  - **Floor mounting guide (5 Nov evening):** `Floor_Parts_Step_by_Step.pdf/.docx` (15 pages, 11 steps: robot check + clip test, inserts, standoffs + plates out, PPR2 + tab clamp, PPR3, PPR1 up/back + screws, checks, problems table). `guide_floor_steps.py` (Fusion; STL mesh copies of the floor parts, moved copies written to `%TEMP%\floor_guide`; picture f1 needs `fence_only.stl` from `make_floor_parts.fence()` first; hides the intake/flower arm (`FRONT_HIDE`) for front close-ups), `guide_floor_annotate.py` (reuses `guide_hood_annotate.annotate`), `make_floor_steps_guide.js`. Re-render single pictures by setting `ONLY = [...]` through a one-line fusion.py script (the bridge keeps globals), then `ONLY = None`.
  - Windmill A + H: later, printed windmill PPW1–3. Clear hub covers stay (status lights).
  - **PPR1 v2 (6 Nov night, the lone ball, §5.12): designed, reviewed, NOT printed.**
    - **Where it lives:** `make_floor_parts.py` behind `PPR1_V2 = False`; with it False the output is byte-identical to the printed v1 and the v1 check report is unchanged. v2's STL and 3MF were deleted so they can't be printed by mistake.
    - **Design:**
      - guide wall (3 mm, z 50) from (43.7, −11.9) to (100, −113.5), sealing off the left floor;
      - front edge at y −113.5 (CAD: 6.05 mm behind the Ø17 roller sleeves, 1.05 mm while sliding in);
      - 6° taper capped per x at the front edge (≤ −2.5 at x ≥ 60).
    - **Reviews (Opus 5.5, GPT-6.1): both DON'T PRINT. CAD, fit and print were all OK.**
      1. **Capacity.** The left floor (x 43–81) holds the 3rd and 4th balls while the windmill is parked. The model `capacity` test (6 park angles): v1 fits 4 at 60° (2–3 at 0–45°), v2 fits 2–3. Opus's flood-fill: v2 holds 2 at 0–37.5°.
      2. **Vane-tip locks against the wall's middle**, ball centre 86–105 mm from the axis. Any converging wall within r_c 103–115 locks, whatever its angle. Moving the wall's back end out to x 75 removes the hard locks but also the gain.
      3. **The taper flattens the side slope** at the front-left (7° → 3°); 3 balls fed worse in the model.
      4. **The test starts missed the zone in front of the wall**, where v2 is no better than v1 (Opus).
    - **Smaller notes:**
      - The 3.3 mm "sweep" gap is really about 35 mm (`SWEEP_R` 112 is a bounding-box corner).
      - A sealed pocket behind the wall (x 89–93) could trap a ball.
    - **Before any revival:** tape a cardboard wall on v1 along the line, park the windmill and feed 4 balls.
    - **Better fix: software** (§5.12).
    - Kept: `make_floor_parts_jobs.py <job>` runs one job; `ball_feed_model.py` takes `PPR1_STL=<name or path>`.
  - **PPR4 front-left filler (10 Nov night `c978bb7` rail-screw version → 11 Nov `c8b796f` tape → 11 Nov `f1c37c2` / `76d0001` SCREWED FROM ABOVE, goBildaStarter `main`, pushed; not in Fusion):** printed replacement for the duct tape over the roller–PPR1 gap (§5.12).
    - **Current (`MOUNT = 'screw_top'`):** the user didn't trust tape and asked about building it into PPR1; I recommended against it (PPR1 goes in from below 5 mm forward, so its front can only reach y −113.5 = 6 mm from the roller: test surface in the ball model → 7° sticky ball stops at the old spot 2/74; plus the windmill post PPW2 bolts through PPR1's 4 holes and a 3 h reprint) → user chose "screw". Lip 12 mm, solid on PPR1: 2.2 → 1.6 mm (= the tape version's exact top surface); 3 vertical Ø3.4 holes with 90° countersinks Ø5.6 at x 40/72/104, y −95.6 (4.45 mm behind PPR1's face); heads 0.1 mm below the lowest rim point (lip slopes ~8° sideways: up to 1.0 mm deep on the uphill side); 0.31–0.37 mm of PETG under each cone. Screws: **M3 × 8 flat head 90° 5.5 mm (DIN 965 / ISO 7046) only**. Fitting: drop in with the back face against PPR1's face, dimple the 3 centres, lift off, drill 2.5 mm × 6 mm into PPR1's 3 mm plate (0.2–1 mm behind its front flange; nothing above or below in the CAD; outside the windmill sweep r ≥ 121), cut the threads with a screw first, refit, hand-tighten until seated. Print: right end down, 217 mm² foot, 2 h 4 min, 20.1 g (opened + sliced in Bambu Studio ~11:50, GUI: 2 h 4 min, 20.15 g, no warnings). Ball model = tape version: sticky 0 stuck at 2/4/7°, capacity 4 at 60°, lone 107/178 (no filler 108/179). **Don't raise the top:** a 3.2 mm lip root (+1 mm) moved the 25° entry ramp to x 50 → sticky 2/74 at the old spot, capacity 3. Reviews: tape version PRINT; screw-top version PRINT (applied: cut threads first, hand-tighten, drill with PPR4 off, 90° heads only).
    - **11 Nov test fit of the rail-screw version (20-min test piece printed):** the user: "there is a motor on the left, it's difficult to place M4 there; also the motor mount's M4 is in the same place, one off". CAD confirms: drive motor `5203-2402-0019` fills the left rail from y −139.5 to +2.6 (x 137–175), its mount block `1201-0043-0002` sits at y −120…−108 right behind the web, so no nut/fingers fit inside the rail anywhere there. **Lesson: the robot meshes have no screws or nuts; never plan a nut inside a rail channel that holds a motor; ask/check hardware access first.**
    - **Tape version (11 Nov, `MOUNT = 'tape'`, superseded):** no tab; part x 15–124 (8 mm from the rail's inner face, clear of screw heads); 12 mm lip with its underside 1.1 mm above PPR1's top for 1 mm double-sided foam tape; the whole ball surface 1.0 mm higher than the rail version (same slopes, front still capped at the roller top by the 25° ramp); 1.7 mm down-step at the lip tip (y −88). Ball model 111/184, 0 stuck, capacity 4. Reviewed PRINT (tape 1–1.5 mm short of the tip, 20 min set time).
    - **Design basics** (`make_front_left_filler.py`): front edge y −121 (−117 for x > 100), an arc 2.0 mm from the CAD roller sleeves (r 8.5; 2.5 to a nominal 16 mm sleeve); top = PPR1's surface carried forward, rising 3.5° toward the roller and keeping PPR1's ~8° side fall (a version with a level front edge failed the sticky-ball test: 18/74 stuck); 25° entry ramp where the top would be above the roller (the intake's left side channel `1121-0001-0048` at x 104–116, z ≥ 21.6 keeps ball centres right of x ~68.5).
    - **Screw version details (history):** 1 × M4 × 14 + washer + nyloc through a ±0.75 mm slot in a tab (x 127.95–131.7) into the left-rail hole (y −104, z −16); 7 mm lip resting on PPR1; one screw = a pivot (CoM 4.8 mm in front of the screw axis, 5° of turn reaches the roller). Rail hole map (ray-cast on `1120-0015-0384_1`): 4 mm holes at z −16 for y −120/−112/−104/−96, at z −8.5 for y −112/−104.
    - **Bambu Studio GUI recipe (11 Nov):** `Start-Process bambu-studio.exe '"<job>.3mf"'` with the PowerShell tool's `detach: true`; wait for the title `<job> - BambuStudio`; close an old window with `PostMessage(WM_CLOSE)` only if its title doesn't start with `*`; slice via `EnumChildWindows` → the child titled "Slice plate" → `PostMessage` WM_LBUTTONDOWN/UP at its client centre; screenshot with `PrintWindow(hwnd, dc, 2)` after `SetProcessDPIAware()`.
  - **Single flywheel wheel in the CAD (9 Nov, Fusion version 25):**
    - Deleted the script-added `3626-0014-0096:7` (attribute `LauncherHoodV2/added` = "flywheel wheel 2", x −41..−17). The stock motor-side wheel `:1` (x −17..7) stays; the hood centres the ball on its crown.
    - `launcher_hood_v2_build.py`: `FLYWHEEL_WHEELS = 1` (2 re-adds wheel 2 and its checks). `ball_feed_model.WHEEL_X = (−17, 7)`.
    - Docs: goBildaStarter `README.md` (v25 row), `stl_launcher/README.md`.
    - The document showed unsaved changes before my edit; they were saved with it.
  - **Electronics review (9 Nov; user: back vulnerable, hub mounts bad, had to drill, referee says move the switch inward, brackets block the back channels). Renders in the session folder `files/electronics_review/`.**
    - **Hole bug:**
      - `electronics_build.py` `HUB_HOLES` (−18/+38, ±48) are 0.3 mm-deep cosmetic circles in REV's STEP.
      - REV's drawing has 4 × M3 **through** holes, Ø3.4, in the corner tabs, **128 × 88 mm** (local ±64 × ±44); the hubs are 143 × 103 × 29.5. Both hubs are the same.
      - The channel M4 slots (x ±68/±108, y 184) do match the rear channel's holes (8 mm grid, x ±44..±124).
    - **Other v1 flaws:**
      - Each hub's bottom short edge (14 servo/sensor ports) sits on the 4 mm shelf.
      - The hubs and their 1.5 mm covers are the back face above the rear channel, only 5.5 mm inside it.
      - The switch toggle is 2 mm inside the back face.
      - The brackets cover the rear channel's top.
    - **Limits:**
      - The robot is 452 mm long (intake y −260 to rear channel y 192; 18 in = 457.2, **R102**), so a bumper can't go behind it.
      - A recessed switch at the rear centre hits the hood (10 mm in already does).
    - **Manual (2026-27 BIOBUZZ, TU08; TU09 published 12 Nov 2026 ~16:03 PST, `ftc.game/manual` → `ftc-resources.firstinspires.org/ftc/game/manual`, R102/R105 unchanged):**
      - **R102** (p. 122): STARTING CONFIGURATION = 18 in wide × 18 in long × 18 in **high** (the manual writes 45.70 cm), all parts stationary; pre-loaded scoring elements may stick out. **R105** (p. 123): after the start, up to 18 × 24 × **29 in tall** (height above the FIELD surface), physically limited (not by software).
      - **Height check 12 Nov (user: "is the raised Expansion Hub still allowed?"):** floor = z −55 (lowest mecanum roller). The old top was the camera cradle, z 269.6 = 324.6 mm (12.8 in). Expansion Hub top (v3) z 295.5 = **350.5 mm (13.8 in)**; v4 cover roof z 307 = **362 mm (14.25 in)**: 95–106 mm under 18 in. The hub/cover don't change the width or length (82 mm inside the wheels, back y 191).
      - **R603**: the main switch must be accessible, away from high-speed parts and pinch hazards. It "should be ... protected from ROBOT-to-ROBOT contact", and may be behind removable panels.
      - The damage rule (p. 113–114): an opponent turning off an exposed switch is our problem; doing it to a "reasonably well-protected" switch counts against them.
      - **R606**: the Control Hub lights only need to be showable at inspection; visible in matches is strongly encouraged.
    - **Proposed layout (CAD exact-interference clear):**
      - Hubs on edge behind the launcher towers: x ±96..125.5, y 77..180, z 40/50..183/193 (103 along y, 143 tall, 29.5 thick).
      - Printed plates on the towers' outer faces (x ±92.4..96), with a foot on the rear channel.
      - Cable room clear above, below and behind; on the right, 2 screw heads sit in the front cable gap.
      - Switch beside the hood on the Control Hub side: x 54..86, y 134..172, z 30..75, rocker facing back about 20 mm in, finger room behind it clear.
      - Rear channel top fully free.
    - **Open before building:**
      - Which long edge has the Control Hub's USB/HDMI (the front gap is only about 10 mm at z 94–146, the flywheel motor): pick the rotation or use a right-angle USB.
      - Bottom-edge port room (raise the hub).
      - `HUB_LOGO` / `HUB_USB` in `BioBuzzConstants` (now BACKWARD / LEFT).
      - Wi-Fi ping (metal on both sides).
      - Make the hub carrier removable (inspection, R606).
      - Not built yet: waiting for the user's choice.
    - **BUILT as Plan A (user chose A, 9 Nov):** Fusion **v26**, component `Electronics_Mounts_v2` (v1 hidden, REV models moved).
      - **Scripts:** `electronics_v2_build.py` asserts the hub placement from its own transform, because occurrences added in the same run report empty boxes. `electronics_v2_check.py` checks 31 regions and the placement; all clear. `make_electronics_v2_jobs.py` makes the print jobs.
      - **Print jobs:** `bambu_p2s/Electronics_v2_Hub_Plates_P2S_PETG` (both plates flat on the tower side, 5.3 h, 149 g since v2.1) and `Electronics_v2_Switch_Shroud_P2S_PETG` (1.2 h, 22 g). Guide: `stl_electronics/README.md`.
      - **Final geometry:** hubs x ±97.05..126.6, y 72.5..175.5, z 88.5..231.5. Plates 5 mm at x ±92.05..97.05, y 71..186 (lower back extended to 191.5), z 26..236. No removable carrier: all 8 hub screws are reachable in place.
      - **Fasteners:**
        - Hub: 4 × M3 × 16 per hub into M3 nuts in 2.6 mm hex pockets on the plate's tower side. The front holes (y 80, z 96/224) line up with the tower's 4 mm holes.
        - Plate to tower: 3 × M4 × 12 button head at (y 104, z 112/152/192), flush in counterbores on the hub side, nyloc inside the tower (open back).
        - Feet: 2 × M4 × 16 socket head + washer at x ±108/±116, y 184 (v2.1; v2 had M4 × 14 through a 4 mm foot).
        - Switch: REV-31-1387 rotated 90°, bracket plate at y 160–163, rocker tip y 171.5. Shroud PPE6 at x 50..90, y 157..191.5, z 24..82, no roof, screws at x 60/76.
      - **Orientation:** Control Hub N +x, motor edge up, servo/USB edge forward; servo 0 and webcam USB at the top, above the flywheel motor. Expansion Hub N −x, motor edge up, servo edge back; servos 0/1 at the top with 16.5 mm to y 192. Servo ports are unchanged from `testbotnew.xml`.
      - **Review (rubber-duck):** confirmed holes, lengths, print and IMU. It found the first gusset (y 183–186) capping both foot screws. I found a socket head at x 100 rubbing the plate face by 0.55 mm. Fixed: feet use x 108/116, and the gusset sits at y 188.5–191.5 on a plate extension. The STL measurements show all holes and head/tool columns open.
      - **v2.1 feet (9 Nov evening, before the first print; user asked "would any corners bend"; goBildaStarter `5c62fcd`, pushed):**
        - The v2 foot was the weak corner, and the review had missed it. It was 4 mm thick (y 180.2–191.5) with ±1.5 mm slots: 1.6 mm of plastic in front of the slots, 0.6 mm between them, and the outer slot open at the foot's end (x 119.5 limit from the existing screw head at x 120.5). An M4 head bore on ~1.3 mm strips, ~2× PETG's crush limit at ~1 N·m. The foot prints as a standing fin, so bending it pulls across its layers.
        - Now: `FOOT` y 176–191.5, z 24–30 (6 mm), round Ø4.8 holes, a 3 mm chamfer at the plate (`FOOT_CHAMFER`), and the gusset at y 189–191.5. STL measurements: 5.5 mm in front of the holes, 5.1 behind, 3.1 between, 1.0 beyond the outer hole; the washer and head columns are clear. 33 check regions are clear. The plate job is 5.3 h, 149 g.
        - Screws: M4 × 16 + washer (the shroud gets washers too). Plate corners, shroud walls, nut-pocket and counterbore floors (2.4 mm) are fine if not over-tightened.
        - **Fusion:** `doc.save()` returned True and the doc shows no changes, but after 10+ minutes the cloud still lists v26 as the latest and `dataFile.isComplete` is False. The upload may be slow or retrying on the bad Wi-Fi. Check Fusion's Job Status and keep Fusion open; the repo scripts can rebuild the component anyway.
        - Renders: a Fusion viewport capture needs `cam.isFitView = False` and `cam.viewExtents` (cm), or it fits the whole robot. Hide `Camera_FOV_preview` (normally ON) for the capture and restore it afterwards.
      - **v2.2 after the Opus 5.5 review (9 Nov evening; user: "review by opus 5.5 before the print, open Bambu Studio with proper settings, make it sturdy"):**
        - **Review findings:** no blocking issues.
          - The foot: under 1 MPa in a 10 g hit.
          - Hole play: ±0.5 mm in y, the direction that matters.
          - The 2.4 mm floors are only ever squeezed, the counterbores suit ISO 7380-1 heads, and warping risk is low.
          - Recommended: layer bonding (260 °C, fan 20–35 %), `counterbore_hole_bridging = partiallybridge`, and solid infill (100 % zig-zag, no slower than 30 % gyroid; the slicer rejects 100 % gyroid).
          - Check blind spots under the channel and in the towers, torque limits (M3 ≤ 0.4, M4 ≤ 1–1.5 N·m), press-fit back nuts (a 5.8 pocket prints ~5.65, the 5.5 nut falls out), and the optional R5 corners, 5 mm chamfer and round shroud holes. Also a 1 mm test-fit slice, and `filament_shrink` 99.7 % if the 128 mm hole spacing measures under 127.7.
        - **Applied:** all of the above. `CORNER_R`, `FOOT_CHAMFER` 5, `M3_NUT_AF_PRESS` 5.6 (back two), `M4_HOLE_FOOT` 4.8 (plates and shroud). The check now has 42 regions, all clear.
          - The x 116 foot nut sits beside goBILDA block `1205-0001-0005` inside the rear channel at x 120.0 (held by the existing x 124 screw). With a flat toward the block it ends at x 119.5; the block then stops it turning. Fallback: x 108 alone.
          - Rear channel `1121-0010-0264:2` = x ±132, y 180–192, z ±24 (web at the back, open toward the front).
        - **Jobs** (`make_electronics_v2_jobs.py`; the `J` helper now supports `FILAMENT_CHANGES`, listed for the filament in `different_settings_to_system`, and `check()` verifies them):
          - Plates: 5.3 h, 252 g. Shroud: 1.2 h, 23.5 g. Test fit `Electronics_v2_Test_Fit_PPE4_1mm`: 0.7 h, 25 g.
          - Settings: 5 walls, 5/5 shells, 100 % zig-zag, 3 mm brim (plates), 5 mm (shroud), nozzle 260 °C (first layer 245), fan 20–35 %.
        - **Opened in Bambu Studio** (three windows; `single_instance` is false): it shows Bambu Lab P2S, 0.4 nozzle, Textured PEI, `* Bambu PETG Basic` and `* 0.20mm Standard @BBL P2S` (`*` = our changes kept).
        - Fusion: saved again (v28 intended), but the cloud still lists v26 and `isComplete` is False. Check Job Status.
      - **Robot code `master` `ca1cbcf` (local, 107 tests):** `BioBuzzConstants.HUBS_ON_EDGE = false` → BACKWARD/LEFT; true → LEFT/FORWARD. Flip it ONLY after the rebuild, then check the heading in BioBuzz Calibration (CCW turn → heading increases). No install is needed until then.
    - **PPE5 v3 (12 Nov overnight; goBildaStarter `21317a4`, pushed): Expansion Hub raised above the windmill channel.** The user: "Expansion hub side is not fitting, flywheel channel blocking"; "update geometry in Fusion, prepare the 3D model, get it reviewed, open Bambu Studio".
      - **As built:** the windmill motor's horizontal channel `1121-0005-0144:1` (x −108..−96, z 96..144) and its 2 × M4 × 8 `2800-0004-0008:33/:36` sit 2 holes (+16 mm y) behind the CAD.
        - Team measurements: tower face back→front 30 mm free | 17 mm channel; aluminium 15 mm in front of the y 104 M4 holes (y ≈ 89); channel top ~118 mm above the v2 plate's bottom (z 144); stick-out 15–16 mm; front strip free above the channel.
        - `electronics_v3_ppe5_build.py` moves those 3 occurrences +16 y (set `transform2` on the root **proxy**: Fusion refuses it on nested natives). It keeps the world originals in attribute `ElectronicsV3/orig_transform`; `:33`/`:36` already had the windmill build's −16 x override, so store the proxy's matrix, not the native one.
        - It adds the measured keep-out x −108..−92, y 64..89, z 92..148 as a hidden body (`AsBuilt_Reference_v1`), hides v2's PPE5 body, moves `REV_Expansion_Hub` and builds `Electronics_Mounts_v3`.
        - The Fusion doc was **not saved**: re-run `electronics_v2_build.py`, then `electronics_v3_ppe5_build.py`, to rebuild.
      - **Options (Fusion scan):**
        - Hub 18 mm back: servo plugs 15 mm outside y 192 ✗.
        - Hub outward past the channel: hits the Yellow Jacket `5203-2402-0019` at x −182..−129.8, y 94..146, z 67..148.6 ✗.
        - **Raised: free ✓.**
        - A raised plate that kept the rear foot would be 266 mm tall, over the 256 mm bed, so the foot is dropped.
      - **Geometry:**
        - Hub y 66.7..169.7, z 152.5..295.5, same orientation as v2: 8.5 mm above the channel, servo room 22.3 mm, top 26 mm above the robot's old top (z 269.6).
        - Hub M3 holes: (74.2, 160) over the tower's 8.5 × 4 slot, so the nut is trapped and the M3 × 16 tip goes into the slot. (74.2, 288), (162.2, 160) and (162.2, 288) are press pockets AF 5.6.
        - Plate 5 mm (x −97.05..−92.05): upper body y 65.2..172, z 150..297; lower tab y 94..124, z 98..150 (5 mm behind y 89, per the review); window y 118..152, z 172..270.
        - 4 M4 × 12 button heads at y 104, z 112 (slot)/152/192/240; the z 240 nyloc space inside the tower is free.
        - Tower outer-face hole map (repeats every 24 mm): Ø14 at y 88, z 88/112/…/232; slots 8.5 × 4 at y 74.2/101.8, same z; Ø4 at y 72/104, z = big ± 8; Ø4.5 at y 79.8/96.2, z = big ± 8.2.
      - **Checks:**
        - Exact Fusion booleans: plate and hub touch nothing. Clear: motor plugs above, servo room behind, front, the bottom-port cable zone (x ≤ −110: REV's JST ports are top-entry from the logo face), the RS485 route under the plate behind the tower, and the camera FOV cone.
        - Independent manifold check on the STL: tower 0.05, channel 6, channel screws 7.5, measured line 5 (tab).
        - Rubber-duck review: print as is after moving the tab back 4 mm and proving the RS485 route. Fitting caution: keep the RS485 cable above the motor until y > 149.
      - **Job** `bambu_p2s/Electronics_v3_PPE5_Hub_Plate_P2S_PETG`: 1 h 50 min, 81.8 g, v2.2 settings, no warnings. Shots in `shots/ppe5_v3/`; guide in `stl_electronics/README.md` (top section).
      - **Cables:** every Expansion Hub connection is ~6.5 cm higher (motors and encoders, XT30, RS485, servos 0/1). The team must check their reach.
      - The moved-back test strip (`make_ppe5_v3_test.py`, `940e18e`/`5e302e3`) is removed; it was printed 11 Nov and the user measured 15 mm on it.
    - **PPE5 v4 + PPE7 cover (12 Nov morning; goBildaStarter `9628108`, pushed): a guard around the raised hub.** The user started printing v3, then: "vulnerable to defence as it's on the top; v4 with a cover on the sides".
      - **Built by** `electronics_v4_ppe5_build.py`, which redoes v3's as-built moves and hub move, hides `Electronics_Mounts_v3` and builds `Electronics_Mounts_v4` (the Fusion doc is not saved).
      - **Unchanged from v3:** tower holes, hub position and nut pockets, so v3 can be fitted now and swapped later.
      - **Plate PPE5 v4** (4 h 02 min, 180 g):
        - Upper body y 55.7..191, z 150..295.5 (top = the hub's top).
        - 4 mm walls at y 60.7..64.7 and 185..189, full depth x −92.05..−140, z 150..303.5, with 45° root fillets (front outside 5, back inside 5 / outside 2).
        - 4 screw columns at (61.2, 168/280) and (185.5, 168/280): Ø9, Ø11 outboard of x −127, with M3 nut press slots AF 5.6 at x −128..−130.6 that open into the tray.
      - **Cover PPE7** (2 h 49 min, 91 g; prints on its outside face, no bridges):
        - Side panel x −140..−143.5, y 55.7..191, z 155..307, with 32 slots 8 × 28 mm (~36 % open).
        - Solid roof z 303.5..307 spanning x −85..−143.5: a hood 7 mm past the plate's tower face. The top cables leave through the 8 mm gap over the plate's top edge into the robot, so the cover slides off without unplugging.
        - 3 roof ribs at y 79/123/166, z 299.5..303.5.
        - 4 × M3 × 16 into the column nuts. Bottom open.
      - **Fit and checks:**
        - Envelope: back y 191, top z 307 (+37 mm); the robot's right side is the wheel at x −225.9.
        - Exact Fusion checks and an independent STL pass are clean: hub to roof 8 mm, to the ribs 4 mm, to the panel 13 mm; the cable zones are free.
      - **Review (rubber-duck):** v4 had 30 × 27 windows plus roof slots 2 mm above the hub. Its blockers: direct paths to the plugs, a captive cover, PETG threads, weak wall roots. All fixed in v4.1. Re-review: **print as is** (route the XT30 flat between the roof ribs).
      - Both jobs were opened and sliced in Bambu Studio (Print plate ready). Guide: `stl_electronics/README.md` (top section). Fusion captures need a warm-up `saveAsImageFile`: the first capture keeps the old zoom.
    - **PPE8L/PPE8R servo wire hoods (12 Nov; goBildaStarter `a1ee2b9` → `8003d43`, pushed).** Built by `electronics_wire_hoods_build.py`; guide in `stl_electronics/README.md` (PPE8 section, top).
      - **`8003d43` (evening):** the user asked to open the job for printing, and Bambu Studio warned "It seems object PPE8… has floating cantilever". Causes: a back-wall strip under the roof, 1 mm legs beside the gussets, and anchors starting in mid-air (front-down print: the back wall, tab and anchor are the last layers).
        - Fix: back opening u145.5..166.5 from the tab top z29 to the roof z52, tab kept whole; anchors span the opening (left u145..167, right u145..175 under the whole 14 mm notch).
        - The right servo block's outer screw (u172) is under the anchor; its cutout stops at the tab top, so turn that screw with the hood off.
        - Right inner nylock (on the tab): anchor cut back r7.5, z29..37, to fit a 7 mm wrench (new Fusion check zone).
        - Rubber-duck: the first fix ended the right anchor at u167 under the 14 mm notch (cable over an edge) → "do not print"; with u175 → **print as is**. Job 2 h 26 min, 91.7 g; opened and sliced in the GUI with no warnings.
      - **Bambu "floating cantilever" rule** (source: TreeSupport.cpp `detect_overhangs`, PrintObject.cpp `is_support_necessary`; runs only when support is off):
        - Overhang = the layer minus the lower layer offset by 0.2/tan 30° = 0.35 mm.
        - A contour is floating if none of its vertices is within one line width (0.42) of the lower layer. Otherwise the cluster counts as a cantilever when its farthest exterior vertex is > 3 mm from the nearest vertex of the supported edge.
        - The GUI warns when the maximum is > 6 mm. Small anchors or strips that start over air trigger it.
      - **Check it without the GUI:** the CLI's `result.json` → `sliced_plates[].warning_message` holds the same text the GUI pops up. `make_electronics_v2_jobs.py` now prints it as `SLICER WARNING:`. `slice_info.config` in the `.gcode.3mf` does not hold it.
        - My Python replica (session files `servo_wire/bambu_cantilever_check.py`) matches only if slices are simplified first: Bambu's resolution is 0.012 mm and Clipper drops collinear points, otherwise mesh-diagonal vertices give false hits.
      - **GUI notes:**
        - Bambu's GUI logs are encrypted (`*_enc.log`), so they can't be grepped.
        - `CopyFromScreen` returns black while the user's full-screen Remote Desktop window (msrdc) is in front. `PrintWindow(hwnd, dc, 2)` still captures Bambu's 3D view once it has drawn.
        - A Bambu window closed itself (or was closed) at 21:55 with no crash event. Always check the window still exists before reporting "open".
        - Execution policy blocks `. script.ps1`: use `iex (Get-Content -Raw …)`.
      - **Lesson (user, 12 Nov):** "Have no clue what you are doing for past 3 hours" and "You designed it and are asking me what to do; I wanted you to answer". Post a one-line status when a simple request turns into a redesign, and make the design call myself instead of asking.

## 8. Branch `field-tour`: Pedro Pathing "Field Tour" (test, later AUTO base)

**Goal (user):** put the robot anywhere on the field; it knows its position and the whole field map, tours the field (like a Roborock), and returns to where it started. This will later be used for AUTO.

**Decisions:**
- The camera can't localize this season (§5.7), so the user chose **option A: marked start tiles picked in INIT** (tile column/row + facing).
- Distance sensors (option B) were offered for later.

**Pedro Pathing 3** (brand new). **The API is very different from Pedro 1.x/2.x, so don't use old docs.**
- `TeamCode/build.gradle`: repo `https://repo.dairy.foundation/releases/`, `com.pedropathing:revhub:3.0.2`, `com.pedropathing:tuning:1.0.3`.
- `pedro/Constants.java`:
  - `MecanumConfig`
  - `PinpointConfig` (offsets in MM from BioBuzzConstants, goBILDA convention)
  - `ForesightConfig` from the AutoTune Foresight procedure, 4 Nov (commit `a80ff5c`); it replaced the Pedro docs placeholder
  - `FORESIGHT_TUNED = true` since 4 Nov (a safety flag: the tour won't drive while it's false)
  - `create(hw)` builds the `Follower`
- `pedro/Tuning.java` (`@Tuner` methods) + `pedro/procedures/*` (copied from Pedro-Pathing/Quickstart). Tuning runs from **AutoTune at http://192.168.43.1:10158** in this order: Mecanum → Pinpoint → Foresight → Tests. Each step shows generated Java config to paste into `Constants`.
- API facts (checked in the jar):
  - `Follower(localizer, drivetrain, algorithm)`; `update()`, `follow(Path)`, `isBusy()`, `hold(Pose)`, `stop()`, `setPose()`, `pose()`, `follower.drivetrain.stop()`
  - `Pose(x, y, headingRad)`; `Paths.line(a, b).constant(h).with(config.var.at(v))`
  - Pedro coordinates: origin bottom-left of a 144 in field, x right, y up, CCW radians
  - **`Follower.stop()` does NOT zero the motors** (only a later `update()` in IDLE does), so call `follower.drivetrain.stop()` too
  - **`ForesightConfig.maxPathSpeed` is a FRACTION of top speed** (0.30 = 30 %), not in/s
- `fieldtour/FieldMap.java`, from the field CAD:
  - field ±70.5 in (141 in), 6×6 tiles of 23.5 in
  - frame: origin at field centre, **+X toward the BLUE wall**, **+Y = left as seen from the RED station**, heading CCW from +X
  - HIVE A-frame on the floor: half-size 24.75 × 19.5 in. Keep-out = A-frame + robot half-width (9) + 3 in (the tour never turns, so the footprint stays square)
  - laps stay 3 in off the walls
  - 28 of 36 tiles are valid starts
  - flowers are listed but **not avoided** (the user's field has only the HIVE)
  - Blue HIVE centre (12.8, −2.6), Red (−12.7, 2.6)
- `fieldtour/TourPlanner.java`: HOME → nearest point on the outer lap → outer lap → inner lap (around the keep-out) → back out → HOME. Straight legs. Tested from every start tile: never enters the keep-out, ends at HOME. About 800 in total (~40 s at 30 %).
- `fieldtour/BioBuzzFieldTour.java` (TeleOp list, "BioBuzz Test" group):
  - **INIT:** D-pad sets the tile, △ the facing, plus an ASCII map
  - **PLAY:** one line path per leg at 30 %, constant start heading
  - **Stops:** ○ = immediate stop (checked before the drive update; `halt()` = `follower.stop()` + `drivetrain.stop()`); STUCK if moved < 1 in for 2 s
  - at the end it holds HOME and shows "From HOME" error
  - if not tuned, it only shows the plan
- `FIELD_TOUR.md`: the team's steps.
- **Reviewed** by GPT-6.1 and Claude Opus 5.5 subagents (2 Nov). They found 2 bugs: stop not stopping, and speed cap units. Both fixed in `a5ccafe` and re-confirmed by both reviewers.

**Status (2026-11-06, morning):**
- **Installed on the robot: `master` `8967f09`** (6 Nov 09:57, over adb Wi-Fi from the laptop's second adapter on `37102-RC`; 93 tests pass; all 11 BioBuzz op modes registered, robot running). It has the printed-hood flywheel code (NEAR 1150 / FAR 1240, P 100, ready + aim gates) and the windmill feed fix. **The Pedro / Field Tour op modes are NOT on the robot now:** install `field-tour` `08aff9d` (same match code + Pedro) for those. Before this the robot ran `a80ff5c` (4 Nov 19:46) with the OLD flywheel settings (NEAR 1250 / FAR 1450, P 40).
- **Installed on the robot: `auto-work` `331978d`** (9 Nov 22:29) = the user's Pedro AUTO with the 9 Nov tuning (`8828abf`, `331978d`: drive times from the 9 Nov runs, 4 s GARDEN intake, stretch off, default slot B) + `150b183` (`master` `1f157a9`: trim 5 t/s, BALL / SHOT_RESULT logging, LB + D-pad labels) + the empty-pocket feed. Hub `base.apk` MD5 = local APK (`5a4cc5d1…`); `Robot Status: running`; crash buffer empty; 14 op modes (BioBuzz AUTO RED/BLUE = Pedro, OLD AUTO RED/BLUE, Field Tour, TeleOp, Tune, …). 139 tests pass. Robot settings file: power 0.40, pause 0.25, `feed_empty_wait_sec=0.10`.
  - **adb "offline" / "read failed" (9 Nov ~21:44–22:27):** ping and TCP 5555 worked, but the hub dropped every adb connection right after the handshake (`%TEMP%\adb.log`: "connection terminated: read failed"). Restarting the laptop's adb didn't help, and the user's first quick restart didn't either. A real robot power-cycle (off ≥ 10 s) fixed it; the first try then connected. The routing was fine: home Wi-Fi is 10.0.0.x, the robot is 192.168.43.1 via `Wi-Fi 5`.
- Before that (9 Nov ~16:22 – 22:29): **`auto-work` `b06abaf`** = the user's new Pedro AUTO (`40c5ad0`, first installed 9 Nov 11:17, see `auto-work`'s MEMORY.md §8b) + `master` `1ff7495` (no pause after an empty pocket) and `ca1cbcf` (HUBS_ON_EDGE switch, false). Hub `base.apk` MD5 = local APK (`0ed65580…`); `Robot Status: running`; crash buffer empty; registered: BioBuzz AUTO RED/BLUE (Pedro), OLD AUTO RED/BLUE, BioBuzz Field Tour, TeleOp, Tune and the other BioBuzz op modes. 134 tests pass.
  - **Lesson:** at ~16:12 `master` `1ff7495` was installed over the user's Pedro AUTO build by mistake (only the old AUTO left). Before any install, check what the robot runs (this section, `auto-work`'s MEMORY.md, hub `base.apk` MD5) and build from the branch that has everything. The Pedro build registers op modes through Sinister (`SinisterRegisteredOpModes`): look for the op mode names in logcat, not `OpmodeRegistration … registered {`.
- Before that (7 Nov ~21:05 – 9 Nov 11:17): **`master` `b1cee74`** (adb `Success`; the hub's `base.apk` MD5 = the local APK; the new app process reached `Robot Status: running` with all 11 BioBuzz op modes registered; crash buffer empty). Auto shot speed + one-wheel presets NEAR 1450 / FAR 1570; 107 tests pass. The user's AUTO work: branch `auto-work` from `b1cee74` (worktree, §2).
- Before that (6 Nov 09:57 – 7 Nov): `master` `8967f09` (printed-hood flywheel code NEAR 1150 / FAR 1240 for two wheels, windmill feed fix).
- Install recipe that worked: `adb connect 192.168.43.1:5555`, `adb -s 192.168.43.1:5555 install -r TeamCode\build\outputs\apk\debug\TeamCode-debug.apk`, then `adb -s 192.168.43.1:5555 shell am start -n com.qualcomm.ftcrobotcontroller/org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity`; check logcat for `OpmodeRegistration: registered {BioBuzz…}` and `Robot Status: running`. The hub clock reads 1969 until the Driver Hub syncs it.
  - To check the NEW app process, take its PID from `adb shell ps` (PowerShell pitfall: `(single string)[0]` is its first character, so `(pidof …)[0]` gave "1" = init) and filter `logcat -d` lines on that PID. `adb shell md5sum <pm path>` vs the local APK proves which build is installed.
- **AutoTune redo (4 Nov):**
  - Round 1 is done. Mecanum confirmed the current names and directions (no change). Pinpoint gave `-1.288 / -7.271 in`, both FORWARD; it's in `Constants` (commit `9ebd09a`). The 2 Nov run gave `-1.85 / -5.92`; the tape gives `-1.57 / -6.89`.
    - Offsets = −(displacement after a hand-turned 180° spin) / 2, so a wobbly pivot moves them by about 1 in. The tour never turns, so this hardly matters for it.
  - Round 2 is done: Foresight (commit `a80ff5c`, `FORESIGHT_TUNED = true`).
    - Top speed 61.4 in/s forward (92 % of the 312 RPM / 104 mm free speed of 66.9), 50.3 strafe (82 %); coast kV agrees (1/kV = 60.8).
    - Braking from top speed about 9.5 in forward, 6.3 in strafe.
    - **Soft spot:** natural forward deceleration is only 30 in/s² (strafe 62, docs example 85), maybe a smooth floor instead of foam tiles. If the Line test stops short or overshoots on tiles, rerun Foresight on tiles.
  - **Round 3 passed (4 Nov, AutoTune Tests):** Line and Curve came back to the exact same spot, with no overshoot, no stopping short and no wobble. Hold worked too.
  - **Next:** the Field Tour on a full field (not run yet). Tour speed 30 % = 18.4 in/s, so the tour takes about 45–55 s. The outer lap passes 3 in from the walls and the inner lap about 5 in from the HIVE frame, so the HIVE must be exactly in its official spot.
  - **AutoTune facts (checked in tuning-1.0.3):**
    - The cards have **empty titles** (pick them by their description), and their order changes when the app restarts.
    - Foresight and Tests use the **installed** `Constants`, so reinstall after Mecanum/Pinpoint changes.
    - Results live only in the browser session (Java tab → **Copy**; there's an `execCommand` fallback on http).
    - Foresight steps have no web Stop button: the Driver Hub STOP is the emergency stop.
- **AutoTune progress (2 Nov):**
  - The Pinpoint procedure gave `xPodOffset -1.847 in`, `yPodOffset -5.917 in`, 4-bar pods, both directions FORWARD. These were in `Constants.localizerConfig` (commit `7484134`) until the 4 Nov redo.
  - Foresight was **not** finished. The team only had about 5–6 ft of space, so the forward/strafe distance was set to **36 in** instead of 48.
  - **The user wants to redo AutoTune later FROM THE START:** Mecanum → Pinpoint → Foresight → Tests. Re-paste the new Pinpoint result too.
- **AutoTune page answers** (HTTP 200) at http://192.168.43.1:10158. Op modes now register through Pedro's loader ("SinisterRegisteredOpModes" in logcat, not "OpmodeRegistration").
- **What the user does in AutoTune:**
  - Mecanum: enter the 4 motor names, answer which way each wheel spun.
  - Pinpoint: pod type **goBILDA 4-Bar**, push/turn by hand.
  - Foresight: the robot drives forward and left by itself, so it needs space.
  - Copy the Java tab text for Pinpoint and Foresight into chat.
- **What the agent does then:**
  1. Paste into `Constants`.
  2. Set `FORESIGHT_TUNED = true`.
  3. Build, install, commit.
  4. The team runs BioBuzz Field Tour from a few tiles; the target is under 2 in / 2° back at HOME.

**For later AUTO reuse:**
- Move the leg-following loop into a reusable class.
- Use short routes (shoot / collect / park), not the full 40 s tour.
- Allow turning legs, which needs a rotated-footprint keep-out.

## 8c. Branch `auto-hive`: HIVE AUTO (12 Nov night; worktree `C:\repo\FtcRobotController-auto-hive`; NOT installed)

**User (12 Nov ~22:25):** "Clone the current auto. Don't change the current one, update from clone. 1. Shoot to hive, this will flip the hive and we get 20 points. 2. Then go to garden and collect 4 balls. 3. After collecting go underneath the hive to opposite end and shoot the ball. 4. Go and park. 3.5 if we have time, explore collecting balls from flower in this area. This plan is aggressive, want to check if it works."
- Answers before sleeping:
  - The robot really fits under the HIVE (also check the manual).
  - Fewer than 4 balls tip it in practice, but "shoot all 4 balls first, then whatever you collect to the other end of hive. This way we are safe."
  - FLOWER = flower arm + intake.

**Branch `auto-hive`** from `master` `c29d480`:
- `9275a15`: the clone.
- `f5761ce`: rubber-duck review fixes, `AUTO_HIVE.md` and a README_2 op-mode row.
- `9a0e381`: re-review fixes (seesaw rule, "blind far shot" switch, FLOWER dwell).
- `57a87e6` (13 Nov morning): the coach said the 9 Nov camera misses were the **dark garage at night**; a lit field should see the tags. Rework from the CAD (below): tag facing, as-built robot, far spot y 58, lane −10.2, bands 42/37, pre-loads always all 4, blind far shot OFF by default. 224 tests pass. Two reviews (code-review + rubber-duck): see below.
- `6c947c9` (13 Nov): fixes for the design review of `57a87e6`:
  - camera-gated volleys (`VolleyGate`);
  - lane back to −10.4;
  - the far aim turns on the spot to 87.25° and is limited to ±2.5°;
  - S1 only;
  - blind far shot = practice only.
  - 233 tests (HivePlannerTest 17, VolleyGateTest 7).
- New files only; **BioBuzz AUTO / AutoPlanner / AutoField / HiveVision are unchanged**.
- Op modes **BioBuzz HIVE AUTO RED/BLUE**: `auto/BioBuzzHiveAutoBase.java` (a copy of BioBuzzPedroAutoBase with the new plan), `HivePlanner`, `HiveAutoField`, `VolleyGate`, `biobuzz/HiveCells.java`, `HivePlannerTest`, `VolleyGateTest`.
- Log: `hiveauto_*.csv`, with a `tip` column.

**Manual TU09 facts used:**
- 9.6: the frame is two A-legs anchored under the TILES; pivots 43.95 in up; the HIVE has 2 CELLs 18.8 in apart.
- 10.3.1: each up CELL starts with **3 NECTAR**; 4 POLLEN in each FLOWER; "the CELL which points at a FLOWER is the one tilted down" (RED's rear CELL points at FLOWER 1).
- **HIVE TIP = 20 pts** in AUTO.
- **SWARM RP = 16 pts LEAVE + PARK**: both robots must LEAVE and PARK.
- **G409 example D: robots driving under the HIVE are allowed.**
- **G417:** only launching into an UP CELL may move the HIVE.
- **G418:** FLOWER POLLEN only out of the bottom.

**Field CAD (field-cad-step.step read with OCP 12 Nov; scripts in the session folder `files/hive_auto/`: `field_cad_probe.py` → `field_parts.json`, `frame_slices.py`, `flower_slices.py`):**
- RED HIVE lowest point **30.65 in**; frame box ±24.75 × ±19.48.
- **A-frame legs lean in:** a RED leg's inner edge is at x = −23.37 + 0.2733 h, at |y| = 18.26..19.35 − 0.4455 h.
- Floor feet / foot bars: x −24.75..−22.75, |y| ≤ 19.5, 2.15 in tall. Under-tile bars sit below the tiles. Between the legs it is open.
- FLOWER 1 at robot heights: x −26.37..−20.42, y 65.63..70.5, lower-ring front face y 65.65. The HIVE planner uses these boxes for all 4 FLOWERs (`hitsFlower`); AutoPlanner's 3.7 in circle is bigger and would block the far spot.
- **CELL AprilTag stickers** (`am-5888-red1/2`, 4 tags in a row, 17 × 4.3 in) on the CELL bottom: **UP 49.67 in high** at 12.89 in from the pivot (y 0), **DOWN 35.65 in**. The HIVE tilts 30° each way and **both stickers face down and toward the end that is UP**. So the audience-side near spot sees our CELLs only before TIP 1, and the rear-side far spot only after it. From the far spot in good light, no tag in view = most likely NOT tipped. Script `files/hive_auto/tag_view.py`.
- **Camera** (C270 640×480, SDK fx = fy = 822 → vertical half-FOV 16.3°; tilt 30°, lens 12.2 in): from the near spot the UP sticker sits ~10–15° above the camera axis. 9 Nov run 212355 read it at 46.3 in high, floor 38.9 in.

**Robot as built (Fusion, 13 Nov; `files/hive_auto/camera_probe.py`, `wheel_probe.py`):**
- Pedro pose = drivetrain centre: mecanum wheel centres at CAD y −144 / +120 mm, so 12 mm ahead of the CAD origin.
- From there: **intake front 9.76 in, back 8.03 in, sides 8.9 in, camera lens 4.63 in back on the centre line** (the `Camera_FOV_preview` body is symmetric about x 0; re-probed 13 Nov).
- The planner's 9/9 box means that with the S1 start (intake on the audience wall) the **real robot is 0.76 in rearward of Pedro's belief all AUTO** (`HiveAutoField.START_OFFSET_IN`). BioBuzz AUTO has the same offset; it works because its shot spot is relative to the start. **HIVE AUTO is S1 only** (`6c947c9`): S2 would put the offset in x.

**Robot height profile (Fusion, box-based):**
- Wheels 4.3 in at |x| > 7.5 in from the centre; end caps 8.0 in.
- Control Hub plate 11.5 in; camera cradle 12.8 in; Expansion Hub 13.8 in; **PPE7 cover 14.25 in** (right side, 3.4..5.7 in from the centre).
- The planner models the robot as two boxes: the full footprint up to 8.0 in, the middle ±5.7 in up to 14.25 in.

**Plan and geometry (RED frame):**
- **Lane under the HIVE x −10.4** (`6c947c9`; −10.2 in `57a87e6`). Low body 1.8 in from the legs (model; 2.8 in at wheel height), 1.4 in from the middle line. Heading exactly 90° under the HIVE: 3° of turn put a corner 1.5 in from a leg.
- **Far spot (−10.4, 58.0, arrive at 90°)**, the real mirror of the near spot:
  - The intake is 2 in off the REAR wall as built.
  - The camera is 41.3 in from the far sticker (near spot 41.2 in), so it gets the same view. At 56.5 the sticker would leave the top of the picture.
  - **The far aim turns on the spot to `FAR_AIM_HEADING` 87.25°**: the centre line (camera + launcher) through the far CELL opening (HiveVision's bearing is to the opening). The camera trim is then limited to **±2.5°** (`FAR_AIM_MAX_TURN_DEG`, log `AIM LIMIT`).
  - As built, the robot is 0.4 in clear of the REAR wall, FLOWER 1's CAD box and the middle line from **83.2° to 94.35°**. The test also covers ±1° of Pedro error around 90 and around 87.25 ± 2.5. At 90° / 87.25°: wall 1.98 / 1.56 in, middle 1.50 / 1.04 in, FLOWER 1 1.12 / 1.44 in.
  - A two-leg route that turned on the way (stop at y 32) cost 0.4 s and lost the slow-case far shot, so the turn is done on the spot instead.
- **Park from the far spot:** BACK OFF to y 52, then **PARK A straight across facing the REAR wall**, 3.0 s. The rear slot A is at y 48.0 (`REAR_SLOT_A_Y`), 4.9 in inside the zone as built (at 49.6 the short back left only 3.3 in). Slot B takes 5 s.
- **HivePlanner.park from anywhere:** OUT FROM UNDER along the lane, FLOWER OUT, BACK OFF near FLOWER 1, AWAY FROM HIVE before AutoPlanner.park's turn.

**Timing:**
- Near-CELL aim cap 1.5 s (BioBuzz AUTO 3.5 s). It ends as soon as the robot is lined up (~0.3 s after reaching the spot when the CELL is in view). The cap only matters in bad light: 9 Nov garage at night, 1 of 8 runs saw the CELL, and the pre-loads were done at 7.1–9.7 s.
- Adaptive GARDEN corner time 1–2 s (`gardenDwellEnd`; BioBuzz AUTO 4 s). Park-by default 27 s.
- **Timeline (model, lit field):** slot A typical: pre-loads 6.2 s, GARDEN 13.9, far spot 21.3, **3.3 s feed (~4 balls)**, parked 29.0. Slow: 1.1 s feed. With a 4 s corner: no far shot. Slot B: 1.6 s or none.

**G417 logic:**
- Per-CELL camera tracking: **UP ≥ 42 in, DOWN < 37 in** (CAD 49.7 / 35.7; UP read 46.3), in between = not sure; 3 fresh frames in a row, ≤ 0.3–0.5 s old.
- **Seesaw rule:** a CELL is DOWN when it is confirmed down OR its other end is confirmed up (`cellState`).
- **Manual G417 penalty:** VERBAL WARNING first; repeated after a warning = STRATEGIC → **MAJOR FOUL (20 pts to the opponent) + YELLOW CARD**.
- **Pre-loads: always all 4.** The design review (13 Nov) flagged that the HIVE can tip before the 4th ball and the rest then go into a CELL going down. I offered A (keep all 4) or B (stop on a TIP). The user first said B, then: *"no, in that case we won't be able to get 4 from garden and aim at the hive. Its fine to aim 4 at first shot. Later we will enhance"* → **A**. Documented as a known G417 risk; the near spot can't see a TIP anyway (the stickers turn away). Only a TIP confirmed before the first ball keeps them for the far CELL.
- TIP watch ≤ 1.2 s: NOT TIPPED (near still UP or far still DOWN ≥ 1 s after the last pre-load) → plan B, back to the near CELL with camera confirmation. Stickers gone → UNKNOWN → far CELL.
- **Far volley:**
  - Camera confirms UP → fire (evidence TARGET_UP). Not seen, but the near CELL is confirmed DOWN from the far spot right now → fire on the planned heading (evidence OTHER_END_DOWN).
  - DOWN or seen-but-unsure → keep.
  - Nothing in view → keep, unless **"blind far shot"** (INIT Circle, **OFF by default, PRACTICE ONLY**; INIT shows a warning line while ON) with 4 counted pre-loads.
- **Every volley after the pre-loads is camera-gated** (`VolleyGate`, `6c947c9`; far volleys and plan B):
  - evidence older than 0.3 s → WAIT: indexer 0 at once, even mid-pocket; ShotFeeder isn't updated and its clock is paused (`feederTime`);
  - seen again → GO on;
  - older than 0.6 s, or the CELL confirmed down → STOP.
  - Blind volleys stop only on a CELL reading down.

**G407:**
- Any GARDEN attempt counts as "may hold balls".
- The intake is off under the HIVE and on plan B's way back.
- The FLOWER only runs when a volley has proved the robot empty (4 counted or 12 pockets).

**FLOWER (3.5):**
- Line-up on row y 51.15; arm down while stopped; creep at 0.15 with the intake on to the stock stop y 57.15 (`FLOWER_STOP_Y`, as built, CALIBRATE; it also ends on contact); dwell up to 3 s (ends in time to fire from the far spot when that fits); arm up 0.4 s; back out. Every phase ends 0.4 s before the park start.
- After the far shot it almost never has time. With the GARDEN OFF (INIT Triangle) FLOWER 1 becomes the ball source.

**Rubber-duck review:**
- First verdict "not ready" for 5 issues:
  1. Far volley didn't stop on TIP 2.
  2. Blind trust 4.
  3. 46 vs 45 threshold and stale frames.
  4. G407 after a cut-short GARDEN; plan B intake.
  5. FLOWER park time; timeline missing the TIP watch.
- All fixed except the blind shot, which stays the coach's call.
- Re-review: 3 more issues, all fixed in `9a0e381`:
  - the seesaw rule;
  - switch OFF must also disable blind fire after a seen TIP;
  - FLOWER dwell vs firing time.
- Final check: see the backlog entry.

## 9. Driver Hub freezes (REV support case, 2026-11-02)

- **Symptom:** the Driver Hub becomes unresponsive and needs a restart. This happened during the 1 Nov competition match.
- **REV Support (Ezra)** asked for: (1) photos, (2) video, (3) whether REV Hardware Client sees it, (4) versions, (5) logs, (6) whether it can be reproduced, (7) when it started.
- **Done by the agent:** #4 and #5 collected over USB, read-only, as `robot_logs/driver_hub_20261102_211535.zip` (with `REV_SUMMARY.txt`). Logs aren't committed.
- **USB adb tip:** the Driver Hub first shows as "(no serial number) offline". The fix is to accept "Allow USB debugging" on the hub, then run `adb reconnect offline`; it then appears as serial `4315Q2U9R9`. The Driver Hub (192.168.43.194) has adb over Wi-Fi switched off.
- **Versions:** Driver Hub OS 1.2.0 (Android 10, build 2021-12-21), DS app 12.0, Wi-Fi chip Realtek RTL8821CU (internal USB).
- **What's on the Driver Hub:**
  - logs in `/sdcard/driverStationLog.txt(.1-.4)` (26 Sep → 2 Nov) and `/sdcard/WifiLog.txt` (from 1 Apr)
  - crash/reboot records via `dumpsys dropbox --print` (only the last 300 entries, about 1–2 days, so pull soon after a freeze)
- **Findings:**
  - 6 restarts between 13:48 and 15:27 on 1 Nov.
  - The Control Hub logged "Lost connection while running OpMode: BioBuzz TeleOp" at 13:49:46 and 14:03:04 (the Control Hub clock is about 1–2 min ahead).
  - The Driver Hub disconnected from 37102-RC on its own (reason 0) at 14:01:21.
  - A `wificond` tombstone: "Driver is in a bad state, restarting wificond".
  - Kernel logs show `RTW ... SurpriseRemoved(True)` / `USB disconnect`: the **Wi-Fi chip dropped off the internal USB bus**. It happened again on 2 Nov at 21:10.
  - The DS app's `WifiMuteStateMachine` lines are only the state list printed at app start, **not** the cause.
- **Conclusion:** most likely the Driver Hub's built-in Wi-Fi module or driver (hardware/warranty). We can't be 100 % sure because the freeze itself isn't logged.
- **Advice given to the team:**
  - take a spare Driver Hub or phone to events
  - ask REV about an OS newer than 1.2.0
  - plug the hub in for logs right after any new freeze

## 10. Open items / backlog

- [ ] Aim test (□ + 10 balls from the spot) to decide between camera offset and driver aim. Then set `aim_trim_deg` or retrain the drivers. (FIRE now waits for the aim when the CELL is in view: `AimGate`.)
- [ ] Fit launcher hood v2 (PPL1, printed 4 Nov; `Launcher_Hood_Step_by_Step.pdf`, **6 inserts / 3 screws per tower**) and the floor parts PPR1–3 (PPR2 + PPR3 printed 5 Nov, PPR1 printing 5 Nov night; `Floor_Parts_Step_by_Step.pdf`).
  - **Installed: `master` `b1cee74` (7 Nov ~21:05)**, after `8967f09` (6 Nov 09:57). The robot no longer has `a80ff5c` with the old flywheel settings.
  - Save the SHOT SPOT straight-on, front ~2 in from the wall; re-teach AUTO (LB + Share); re-check the aim trim in Tune.
  - Start at NEAR 1150, ±15 per D-pad press; run BioBuzz Shot Log to calibrate η and the ball-count dip. (Obsolete since one wheel, 7 Nov: see the next item.)
  - After the session: pull the logs and run `python tools/windmill_logs.py <logs> <date>`: late dips, two-ball pushes (two dips or one > 160 t/s), STUCK → SHOT, few JAMs. Turn a few pockets with an empty windmill: if STUCK appears without balls, raise `INDEXER_MIN_POWER`.
- [ ] **Check the auto shot speed on the robot** (`b1cee74`, installed 7 Nov ~21:05): 5 shots at ~50 in and 5 at ~60 in, trim until they score, put the speeds into `AUTO_SPEED_TPS`; check the `SPEED` events in the logs. No SHOT SPOT is saved on the robot (`spot=none`), so AUTO shoots at NEAR 1450 until one is saved.
- [ ] Retire the weak battery (sagged to ~6 V on 7 Nov). Optional: flywheel I term (steady error +30 / −40 t/s with the battery).
- [ ] **Electronics v2.2:** print (Bambu Studio windows were opened 9 Nov evening): first the 0.7 h test fit `Electronics_v2_Test_Fit_PPE4_1mm` against the left tower, then `Electronics_v2_Hub_Plates` (5.3 h, 252 g, solid) and `Electronics_v2_Switch_Shroud` (1.2 h). Fit per `stl_electronics/README.md` (M4 × 16 + washer on the feet, torque limits, press-fit back nuts). Then set `HUBS_ON_EDGE = true` (`ca1cbcf`), install, check the IMU in BioBuzz Calibration, and check the Wi-Fi ping (§7).
- [ ] **Front-left gap (lone ball, §5.12): print and fit PPR4, screw-from-above version** (`bambu_p2s/Floor_PPR4_Front_Left_Filler_P2S_PETG.gcode.3mf`, 2 h 4 min, 20 g; opened and sliced in Bambu Studio 11 Nov ~11:50). You need: 3 × M3 × 8 flat head 90° (DIN 965 / ISO 7046), 2.5 mm drill, hand screwdriver. Remove the duct tape + polycarb strip. Fitting per `stl_floor/README.md` (cut threads first, hand-tighten). After fitting: roller turns free (1.5 mm hex key passes), thumb test, AUTO 4/4 preloads launch.
- [ ] **Backup PPR1 (user preparing it 11 Nov):** PPR1 now has PPR4's 3 pilot holes built in (goBildaStarter `862d626`, `PPR4_PILOTS` in `make_floor_parts.py`: Ø2.6 at x 40/72/104, y −94.85 = PPR4's holes with its back face against PPR1; 0.95 mm behind the front flange). Job `bambu_p2s/Floor_PPR1_P2S_PETG(.gcode).3mf` re-sliced: 2.9 h, 122.7 g, no floating islands. On that PPR1: no drilling, cut the threads with a screw on the bench, then screw PPR4 on. Floor check report unchanged; PPR2/3/4 STLs unchanged.
- [ ] **Flywheel inertia disc (user decision): order** the Ø73 × 1/4" steel disc (DXF in goBildaStarter `stl_launcher/`) + 8 × 2815-0004-0016 + 8 × M4 washers; fit per `stl_launcher/README.md` (centre it, hand-turn test with the largest ball, then 500 → 1450 t/s); then check READY settling and the logged dips (expect ~10–15 %), and re-label shots (the speed table will shift: η changes).
- [x] **Installed `auto-work` `c5edfc1` on 11 Nov 16:36** (MD5 `1a85f757…`, 208 tests; was `2c38ba9` / `094f36b0…`): last night's code (R1 aims + drives to the SHOT SPOT + camera speed, robot-centric default, SHOT SPOT tied to its CELL, Tune guard) + doc-only commits (physics doc, PPR1/PPR4 notes, driver card with shot logging, driver quiz). 14 team op modes registered, Robot Status running. The laptop's Wi-Fi 5 was already on 37102-RC when I started (robot powered; profile auto-connects); disconnected after. Robot settings file then: `alliance=BLUE`, `spot_ref=floor`, aim trim 0, feed 0.40 / pause 0.25 / empty wait 0.10, **no SHOT SPOT** → save one (Share at 36–42 in with the CELL in view) before relying on R1's drive.
- [ ] **PPE5 v3 (Expansion Hub raised): printing 12 Nov 07:20** (`Electronics_v3_PPE5_Hub_Plate_P2S_PETG`, 1 h 50 min). While it prints, check that every Expansion Hub cable reaches ~6.5 cm higher. Fit per `stl_electronics/README.md` (PPE5 v3 section): nuts first, 4 × M4 × 12 before the hub, RS485 back along the hub and above the motor. No code change.
- [ ] **PPE5 v4 + PPE7 cover (guard), reviewed "print as is"; both jobs are open and sliced in Bambu Studio:** `Electronics_v4_PPE5_Hub_Plate_P2S_PETG` (4 h 02 min) and `Electronics_v4_PPE7_Hub_Cover_P2S_PETG` (2 h 49 min), or both in one job `Electronics_v4_PPE5_and_PPE7_One_Job_P2S_PETG` (7 h 32 min, cover standing on its roof: flat they need 271 mm side by side; also open and sliced). Swap from v3 per the README (v4 section); extra hardware: 4 × M3 × 16 + 4 M3 nuts for the cover.
- [ ] **PPE8L/PPE8R servo wire hoods (12 Nov, goBildaStarter `8003d43`, cantilever fix of `a1ee2b9`): print `Electronics_PPE8_Servo_Wire_Hoods_P2S_PETG`** (2 h 26 min, 92 g, both hoods standing on their front face; opened and sliced in Bambu Studio 22:14, no warnings, Print plate ready). Fit per `stl_electronics/README.md` (PPE8 section). Hardware: 4 × M4 × 14 + 4 M4 nylocks (into the rails' free 4.5 mm holes at |x| 147.8 / 164.2, y −176.2, nuts inside the rail; the right inner screw goes in from below with the nut on top, held with a 7 mm wrench). The servo blocks keep their M4 × 20. **Tighten the right servo block's outer rail screw before fitting:** the hood's anchor covers it.
  - **Why** (user: "servo wire got cut during defence attack ... both sides"): the intake servos 2000-0025-0003 hang upside down above the 72 mm intake wheels at the front corners. Their wire seal is on the case's **front** face (|x| 152.5..159.5, y −239.2, z ~38), and the wire ran outward past the mecanum wheel (8 mm from the rail). The right arm servo's seal points down at (x −155, y −230, z 69..74).
  - **Design** (`electronics_wire_hoods_build.py`, Fusion temporary BReps, exact checks):
    - Box |x| 135..178, y −253.5..−188.5, roof z 52..55 (8.5 mm over the servo top 43.5). Front wall inner face y −249.5 (10.3 mm pocket).
    - Walls down to z 11.0, 0.5 mm over the servo block plate (its top under the walls is z 10.5, not the bbox 13.9).
    - Notch: left |x| 152..160, right 160..174 (outboard of bar `1516-4008-0430:2` at x −157.7..−114.3, y −179.7..−171.9, z 47..55.7). Floor z 44.5, rounded R1.5.
    - Zip-tie anchor y −188.5..−180.5 up to z 44.5, with a tunnel. Tab y −192..−170, z 24..29, with Ø9 cutouts for the servo block's M4 × 20 heads at (|x| 140/172, y −184).
    - Back wall open at |x| 145.5..166.5, z 24..43.5; 45° corner gussets (the right outboard one only to z 43.5).
    - Right only: 8 mm roof slot open inboard + front/outboard fence to z 63.5.
  - **Rail 1120-0015-0384:** a U-channel **open at the bottom**, top wall z 21.5..24. Top-face holes near the front: Ø4 at |x| 140/172 (y −184, −176, −160, −152), Ø4.5 at |x| 147.8/164.2 (y −183.8, −176.2, −159.8, −152.2), a 4×5 slot at |x| 156 (y −180, −156), Ø14 at |x| 156 (y −168, −145).
  - **Review (rubber-duck, 3 rounds → print as is):**
    - v1 clamped its tab under the servo block's screws (PETG creep).
    - v1 had a tight bend, one 8 mm notch for 2 cables, a 2.2 mm slit and a low anchor.
    - Bridges: a 39 mm roof, then a 39–42 mm back wall right behind the servo. Now the longest bridge is 23.8 mm (tab front strip), and the job sets `thick_bridges=1`, `bridge_speed=25`.
    - Fitting note: trim any droop under the tab/anchor (22.75 mm bridge).
  - **Fusion pitfalls:**
    - `measureMinimumDistance` needs root-context proxies (`body.createForAssemblyContext(occ)`).
    - A shared face between union/cut features (anchor = notch width) made a non-manifold STL; make one 1 mm wider.
- [ ] **Next practice:** save the SHOT SPOT, 20+ shots with R1 (battery ≥ 12.5 V), label every ball (L1 + D-pad), pull the logs (`tools/pull_robot_logs.ps1`) and run `tools/shot_results.py`; give the driver quiz (`BioBuzz_Driver_Quiz.pdf`, pass 28/33 + every ★).
- [ ] **HIVE AUTO (branch `auto-hive`, §8c): finish the review round, then the first field test.** Not installed (the robot runs `c5edfc1`).
  - **13 Nov ~08:52: pushed `6c947c9`** (design-review fixes on top of `57a87e6`; 233 tests pass). Two reviews of `9a0e381..6c947c9` (code-review agent `27d9c73e…`, rubber-duck `392ef012…`) were still running when the user put the laptop to sleep to go out. **On return: read their results (or rerun both), fix the findings, rebuild, push, then report to the user.**
  - APK for `6c947c9`: `C:\repo\FtcRobotController-auto-hive\TeamCode\build\outputs\apk\debug\TeamCode-debug.apk`, MD5 `051843c2…`.
  - **Install `auto-hive` after the reviews.** It also contains everything on `master` `c29d480` (HUBS_ON_EDGE, wiggle).
  - Dry run first, no balls: watch the lane under the HIVE (~1.8 in to the legs, never past the middle line) and the ~3° turn on the spot at the far spot.
  - Then 3–5 full runs in good light. Note whether TIP 1 happens (and whether before the 4th pre-load: G417 risk, user decision A), how many GARDEN POLLEN come in with the 1–2 s corner, whether the far shots go in and TIP 2 happens, and whether it parks by 30 s.
  - Pull `hiveauto_*.csv`: check `height_in` (UP ≈ 46, DOWN ≈ 32; bands 42 / 37) and the `AIM LIMIT`, `WAIT` / `GO ON` and `STOPPED` events.
  - Blind far shot: practice only, keep it OFF in matches.
- [ ] **Install `master` = `auto-work` `81c1f31` and test the GARDEN corner + the IMU** (not installed yet; the robot runs `c5edfc1`).
  - **`81c1f31` (12 Nov, user: "I also moved the control hub to side, there was an env update needed?"):** `BioBuzzConstants.HUBS_ON_EDGE = true` (IMU: logo LEFT, USB FORWARD).
    - Both mountings stand the hub upright with the same hub axis up (logo × USB points down in both), so they differ by a fixed yaw offset only. All code uses heading changes (`imuAtInit`, `imuHeading0`, `resetYaw`, FieldHeading offsets) and nothing reads pitch/roll, so the old setting did not break driving.
    - The Pedro AUTO uses the Pinpoint only.
    - **After install:** BioBuzz Calibration, turn CCW by hand, the heading must increase. If it decreases, the hub is upside down (motor connectors at the bottom): use USB BACKWARD.
    - Also check the Driver Hub ping (metal on both sides of the hub now).
  - **12 Nov:** the team tested AUTO (`c5edfc1`) and it works. `master` was fast-forwarded to `auto-work` `c5edfc1`, so `master` now has the Pedro AUTO, Field Tour and Pedro tuning code plus AUTO.md / FIELD_TOUR.md and an older copy of MEMORY.md. **This file (`field-tour`) stays the canonical MEMORY.**
  - Then `a15ac67` (user: "after reaching the corner we need to come 1 or 2 inch front and back for successful intake"): the still dwell at the GARDEN corner (up to 4 s, cut short at the park start) is now a back-and-forth.
    - `AutoField.GARDEN_WIGGLE_IN` = 2 in back along the wall, then in to the sweep end, repeated, at `SPEED_WIGGLE` 0.25 with the intake on (`AutoPlanner.gardenWiggle()`).
    - Each move ends after 1 s (`GARDEN_WIGGLE_MOVE_SEC`) if it can't finish; no BLOCKED there.
    - Log: `EVENT GARDEN WIGGLE … / done: N moves`; steps `GARDEN WIGGLE OUT/IN`.
    - New laptop test `gardenWiggleIsShortStraightAndClear`; 209 tests pass.
  - **Check on the field:** all 4 POLLEN in, no wall hits. Tune `GARDEN_WIGGLE_IN` within 1–2 in if needed.
  - **Encoding pitfall (12 Nov):** `a1f12ab` double-encoded this file. Windows PowerShell 5's `Get-Content -Raw` reads UTF-8 without a BOM as cp1252, and writing that back as UTF-8 garbles every special character. Repaired from `9ee6811`. Edit this file with the edit tool or Python (`encoding='utf-8'`), never Get-Content → WriteAllText. Here-strings piped to `python -` also turn non-ASCII characters into `?`: use `\u` escapes in such scripts.
- [ ] Fusion trial: the window title said "Trial ends in 6 days" on 9 Nov and **"Trial ends in 3 days" on 12 Nov (≈ 15 Nov)**. Make sure the team keeps a working Fusion licence: every CAD script runs through the FusionBridge add-in, and the robot doc is open but unsaved (title `3200-2627-0004 BIOBUZZ robot*`). CAD is in Fusion v26; the v2.1 save on 9 Nov ~16:55 hadn't shown up as v27 in the cloud by 17:10: check Fusion's Job Status, or re-run `electronics_v2_build.py` and save.
- [ ] Straight-on check for the SHOT SPOT (cluster yaw), if aiming from the side keeps missing.
- [ ] Set `feed_pause_sec` to about 0.6 (BioBuzz Tune or adb sed).
- [ ] **The last ball:** PPR1 v2 was rejected (§7). The `ShotFeeder` no-pause change is done (§5.14); it also helps the vane-throw lone-ball problem (§5.12 model: no pause → never fed 6 % vs 37 %).
- [x] **Installed `auto-work` `331978d`** (9 Nov 22:29, see the install record).
- [x] **Installed `auto-work` `4c5d916` on 10 Nov 19:06** (MD5 `085de054…`, 14 team op modes, Robot Status running; was `331978d` / `5a4cc5d1…`): GRAB diagnostics + field-centric TeleOp + INIT Pinpoint gyro check + Options tap/hold + the Pedro AUTO heading handoff `b680235` + the Opus/GPT review fixes `0878a3c` and `fcd4a8b`; 203 tests.
- [x] **First drive on it (10 Nov 19:08–19:31, 21:49–22:05):** field-centric used all evening (teleop "worked"); logs pulled to `master` `robot_logs/20261110_205139` and `20261110_220809`.
- **10 Nov shooting analysis (the user: "shot making to the HIVE came down after the code change"):** the shooting code (ShotFeeder, AimGate, ShotSpeed, flywheel control, HiveVision, BallCounter) was identical to 9 Nov's `b06abaf`, and the launches looked the same (flywheel at launch − target median +5 → +8 t/s, ≥ 30 low 10 % both; |aim| at launch 0.8° → 0.2–0.6°; mid-pocket HOLDs 10 % both; loop while the windmill turns 55 → 59 ms). IN/SHORT/LONG/SIDE balls all launched at the same speed and aim; SHORT took slightly more flywheel energy (0.41 vs 0.37: ball/compression). What changed: (1) trim 15 → 5 t/s: on 9 Nov at 36–40 in the drivers shot ~15 under the table (one ← press), on 10 Nov at about the table → LONG 7 of 21 at ~40 in; 1450 at ~40 in IN 8 of 10, the table's 1475–1480 at 42 in IN 2 of 6. (2) At 48–52 in 13 of 14 balls at MANUAL NEAR 1450 (D-pad ↓; 77–92 under the table): IN 5, SHORT 4, SIDE 4. (3) Same distance + speed (35.6 in, 1445) gave IN, SHORT and LONG: ball-to-ball spread. Labels: 51 kept (IN 24, SHORT 7, LONG 9, SIDE 11); D-pad without LB 28 presses, nearly all deliberate trims/presets (a few right after unlabelled balls, ambiguous: ← / → are also the documented trims). **24 of 75 label presses were lost**: the TeleOp loop is ~55–60 ms while firing, a ball's dip gave only 2 low readings ~55 ms apart and BallCounter needed 60 ms → 6 of 34 shots (19:xx) not counted. Loop with tags in view 55 ms vs 35 ms on 9 Nov (without tags 33 vs 28: field-centric reads ~+5 ms); the rest not explained yet (not the GRAB current reads, not vision code) — profile it if it matters.
- **Fixes `master` `71b88c5` (auto-work `2c38ba9`, 206 tests; installed 10 Nov ~22:30, MD5 `094f36b0…`, 14 team op modes, Robot Status running; pushed):** AUTO_SPEED_TPS 36 in 1450 → 1440, 40 in 1465 → 1440, 44 in 1490 → 1465 (shot_results.py from the 51 labels; 48 in+ unchanged — label shots there at AUTO speed first; the Pedro AUTO uses the same table when it sees the CELL, tonight's 5 runs didn't); LAUNCHER_TRIM_STEP 10 (user's choice); BallCounter `twoSamples` (TeleOp/Tune `forTuning()` only; AUTO unchanged): two different low readings ≥ 45 ms apart = a ball (replay of 10 Nov: +17 balls, all real 250–360 t/s dips; labels kept 44 → 58 of 75). Docs: README_2, driver card (+10/−10), shooting guide.
- **Hubs:** still on the BACK of the robot (user, 10 Nov 22:0x: "did not move the controllers to the side yet") → `HUBS_ON_EDGE = false` is right.
- [x] **A/B test (10 Nov 22:37–22:46):** build A (`b06abaf`) installed, AUTO runs 22:39/22:41 → the user: "did not make a difference" → **the code is ruled out**; B (`2c38ba9`, md5 `094f36b0…`) reinstalled 22:47 (14 op modes, running). Logs `master` `robot_logs/20261110_224811`. Also ruled out: battery (A runs 12.42–12.48 V = the 9 Nov 4/4 runs' 12.3–12.5 V), flywheel speed (1440–1480 both), the flywheel's grab (AUTO dip depth median 310 t/s on 9 Nov, at 21:44 and in the A/B runs; TeleOp's 330 vs 300 was sampling/distance), pose/heading (same ±0.3 in / ±0.5°). Remaining = what the robot can't measure: AUTO start placement / field (the camera can't see the CELL from the AUTO shot spot, so 1–2° of start rotation misses), balls, hood/flywheel tread, feed path. Only measured difference: 3 of tonight's 6 AUTO runs launched just 2–3 of 4 preloads in 12 pockets (`FIRE done: 2 balls, 12 pockets`), all 7 runs of 9 Nov evening launched 4 (9 Nov afternoon had 0/3 runs too) → check preload seating / windmill / feed path.
  - (Was: prepared the A/B test.) Build A = robot-centric only, trim 15, NO labels (LB + D-pad = presets/trims there!), old table; AUTO shots identical in both (1450 when no CELL). Worktree `C:\repo\FtcRobotController-ab` (b06abaf) and `files/ab_test/` APKs kept for now. AUTO logs 9 Nov (4/4) vs 10 Nov are identical in everything measured (spot ±0.3 in, heading 90 ± 0.5° held within 0.1–0.3° while firing, flywheel 1420–1480 at target 1450, dips 1120–1340, loop 20–26 ms; CELL not seen at that spot both nights → fires on the planned heading at 1450). Battery 12.2–12.5 V (9 Nov AUTOs) vs 13.0–13.1 V (fresh); the windmill is open-loop 0.40. AUTO shooting code unchanged since `b06abaf`. Prepared: worktree `C:\repo\FtcRobotController-ab` at `b06abaf` (the exact build of all 9 Nov shooting; `331978d` was installed 22:29 and never shot), APKs in the session folder `files/ab_test/`: `A_9Nov_b06abaf.apk` (md5 `a38e7383…`), `B_10Nov_2c38ba9.apk` (md5 `094f36b0…`), both versionCode 63. Build A = robot-centric only, trim 15, NO labels (LB + D-pad = presets/trims there!), old table; AUTO shots identical in both (1450 when no CELL). Protocol: same start tape + battery; AUTO 2–3 runs each, count balls in the HIVE; TeleOp same spot, D-pad ↓ NEAR 1450 in both, 10 balls each. Put B back afterwards.
- Install from one session at a time, always the newest `auto-work`, never an older build over the AUTO session's commits. After the next practice: pull logs, `python tools/shot_results.py robot_logs <since>` (all labelled sessions), and label some shots at 48–56 in at AUTO speed (press X after any D-pad ↑/↓ preset).
- **Overnight 10–11 Nov (the user, before sleeping: 5 items, "a robot with 90 % shooting accuracy tomorrow morning"; full autonomy, reviews by subagents):**
  - **Item 4, RB auto-aim** (`master` `fa1f1a9` + Tune guard `17953fa` + docs `d80b9b0`; `auto-work` `c31a527` = code of `a0e5f4c`, 208 tests, APK md5 `4511dc3a…`; pushed; **NOT installed**, the robot still runs `2c38ba9`): see the R1 row in the controls table. Code review (code-review agent, simulation with the real `AimGate`/`ShotSpeed`/`ShotFeeder`/`LauncherGate`/`MotionController`) found 3 issues in the first version: (1) balls 2+ fired at ball 1's speed when ball 1 went before the spot (give-way, CELL hidden > 0.75 s, stick take-over) and the drive went on → the spot drive stops after the first ball (`burstStarted`); (2) with no CELL in view the speed no longer froze at the press, so ShotSpeed's 3 s hold could run out mid spin-up → freeze when `target == null` too; (3) the spot drive/aim followed the per-frame "best" CELL (REAR 31 in / AUDIENCE 66 in flipped on 6 Nov) → everything uses the CELL `ShotSpeed` locked. Re-review: then "the SHOT SPOT isn't tied to a CELL" → `spot_cell` + `Cell.otherAlliance()` + `TeleOpSettings.spotIsFor()` (also in Tune's X drive); re-review OK. **The robot's settings file has no SHOT SPOT** → save one (Share) with the CELL in view before relying on RB's drive.
  - **Item 5:** robot-centric is the default again (`FIELD_CENTRIC_DEFAULT = false`); LB in INIT / hold Options 1 s = field-centric.
  - **Item 2, physics check:** `FLYWHEEL_PHYSICS.md` (robot repo) + `tools/shot_model.py` (η 0.71 one wheel, 0.89 two). Reviewed by a rubber-duck (fixed: the accuracy ceiling is a scenario, not proven; 170 g of disc = 1.74× inertia; estimate 84–90 % with it; clearance needs a 3-D check). Dips 10 Nov p10/median/p90 ≈ 16.7/20.5/24.3 % of the speed (one wheel) vs ~10–18 % with two wheels; scoring window ±4.2–4.7 % at 36–44 in. Lead (not acted on, p = 0.09): balls taken after the windmill stopped went IN 59 % (16/27) vs 33 % (8/24) while it pushed.
    - **Inertia add-on, CAD-checked** (goBildaStarter `main` `1713f07`: `flywheel_inertia_check.py/.json`, `stl_launcher/README.md` "Flywheel inertia add-on", `stl_launcher/Flywheel_Disc_73mm_Steel.dxf`; re-run by me, numbers reproduce): wheel 1 = Hogback on a 1310 Hyper Hub clamped on the motor's 8 mm REX shaft, no outer bearing; free space on its far side (x −17 … −40, where wheel 2 was): max radius with ≥ 4 mm from a 73 mm ball squeezed 9 mm against the right wall = 36.9 mm next to the wheel, 48.5 at x −36 … −40 (only the ball limits it). **Recommended: laser-cut steel disc Ø73 × 6.35 mm (Ø32 bore, 8 radial slots 4.3 wide on the 48 mm pattern), 8 × goBILDA 2815-0004-0016 M4 × 16 thread-forming + washers, 182 g, +1.50e-4 → 1.90×, 4.4 mm from the ball, model dip median 20.5 → ~12 %.** goBILDA 3628-0032-0082 Ø82 steel flywheel flush: 0.1 mm ✗ (on a 10 mm stand-off 4.2 mm but needs M4 × 22). 2 × 3" fender washers on a printed carrier: 2.19×, carrier NOT designed. Balance: 1.9 N per 0.1 mm off-centre at 1450 t/s.
  - **Item 3, windmill speed:** unchanged (0.40 power, ~0.2 s per pocket both nights; AUTO ~400–450 ticks/s). Not the cause.
  - **Item 1, front-left gap filler PPR4** (goBildaStarter `main` `c978bb7` rail screw → 11 Nov `c8b796f` tape → `f1c37c2`/`76d0001` screws from above into PPR1, pushed; each version reviewed: PRINT): see §7 and `stl_floor/README.md` PPR4.
  - Docs rebuilt: README_2 (RB, SHOT SPOT CELL, robot-centric), driver card PDF, shooting guide docx + PDF.
- **Field-centric details (10 Nov):**
  - `FieldHeading` v4 (`master` `3e46835`): the heading relative to the drivers (0 = intake away, CCW +), from the Pinpoint (leads) and the IMU, each with its own offset. The IMU is read right after `pinpoint.update()` at the top of the TeleOp loop (back-to-back samples).
  - Each reading is judged against its own sensor's track first (rate check ≤ 450°/s·dt + 6°, dt capped 0.25 s), then the sensors are compared. IMU readings are taken as is or flipped 180° (whichever fits its last on-track reading, also while it is off its track; else nearer the Pinpoint/prediction): the IMU yaw flips at ±90° (§5 item 15).
  - Agree (≤ 10°): follow the Pinpoint however fast; an IMU branch its own track rules out agrees only with a Pinpoint move the robot can make. While agreeing, the IMU's offset follows the Pinpoint by ≤ 1°/loop (bad Pinpoint readings can't drag it).
  - Both fit but disagree: both within reach of the prediction (last rate; 3000°/s², allowing for the lagging rate estimate) → the nearer one, unless the IMU repeats its last reading exactly (frozen) → Pinpoint; only one in reach → that one; neither → Pinpoint. If the Pinpoint wins, the IMU is **apart**: it keeps its own offset for 2 s (`IMU_REALIGN_SEC`), the Pinpoint wins every contest meanwhile; the IMU shifted by how far apart they are agreeing with the Pinpoint = moving together (however fast: hits, stalls, flips); agreeing again ends it ("Pinpoint back in line"); a Pinpoint dropout/glitch hands over to the IMU's own heading, unless the IMU repeated its reading exactly 3+ times (maybe frozen: heading held); reset() while apart zeros each sensor by its own heading; after 2 s it is lined up. (Was: lined up after 2 disagreements, which let a short bad Pinpoint run corrupt the IMU for good. No quick "frozen" line-up: a still IMU repeats its reading.)
  - The turn rate comes from the leading sensor's own move: a jump to the other sensor's heading (taking over, back in line) is not a turn.
  - A sensor leading alone must read where the robot can get from the prediction (not when taking over); if the next reading only fits without the last one, the last one is dropped.
  - Off its track: back when it agrees with the other, or (no other on track) near the prediction (very near if it moved on smoothly). Moving on smoothly for 0.3 s = restart → lined up, carrying the turn it saw while nobody gave the heading. A sensor with no reading in the previous update rejoins at the current heading.
  - Options reset turns the whole frame (offsets − heading). Options in INIT also clears AUTO's handed-over heading. IMU flip events are logged at most once a second (with a count).
  - `FieldHeadingSimTest`: simulated robot (≤ 300°/s, ≤ 2500°/s², logged loop times 20–80 ms, rare 0.1–0.22 s; real logs: p50 23 ms, max 213 ms), IMU flips on/off, 11 fault types × 400 runs. Results: no fault/Pinpoint dropout/garbage/burst/IMU garbage/burst/restart/frozen/IMU only: ≤ 1.2° after the fault. Pinpoint restart: ≤ 8.4° can stay (documented limit). IMU alone + one bad reading: ≤ ~20° can stay, heading held ≤ 0.3 s (documented). Long-stall stress (0.3–0.5 s, both sensors): nothing left. `shortBadPinpointBurstsLeaveNoLastingError`: 20,000 runs of 1–5 bad Pinpoint readings (random or ramps; still or turning; IMU flipping or not; a still IMU repeating exactly 0/50/100 %): old code 2,483 off by > 2° (worst 180°), now 0 (worst 0.4°).
  - Measured from the logs for this: turn acceleration p99.9 2620°/s² (wheel encoders), max turn rate ~300°/s (p99.9 253).
  - With no reading at all, `update` returns NaN, so the drive goes robot-centric and the screen says NO HEADING.
  - AUTO handoff: `BioBuzzMatchState.recordAutoHeading` (OLD AUTO: 180 + IMU since START, every loopTick; Pedro AUTO: the start pose heading at START, then `redHere().headingDeg` every loopTick once the pose is set). TeleOp INIT reads `autoHeading()` if < 60 s old; START clears it.
  - Diag: `CONFIG` field_centric / start_heading, `DRIVE_HEADING` events, columns `drive_hdg` / `field_drive`. The IMU is read once per loop and reused by telemetry.
  - Reviews: rubber-duck #1 (4 issues, `b01f98f`), #2 (4 issues, `21c7f00`), #3 (3 issues → v3 `52feac1`), #4 (3 issues → v4 `3e46835`, plus the simulation's findings), #5 (a Pinpoint frozen with a READY status, or drifting, still leads and drags the IMU), #7 Claude Opus 5.5 + GPT-6.1 (sol) on 10 Nov → `0878a3c`: Options hold re-zeroed on the press; a short bad Pinpoint run (8, 30, 60 then back; 9, 60, −120, 180 then back = 180° for good) corrupted the IMU; X-hold frame flipping with the camera. Their check of `0878a3c` → `fcd4a8b`: a stall + IMU flip while apart (166° off), a frozen apart IMU taking over with a stale heading (~100° off), reset while apart undone by the bad sensor, an Options hold whose release a slow loop hid (`TapHold` now). Tests reproduce each (fail on `0878a3c`). Known open (pre-existing, Opus): if the hub IMU lags the Pinpoint by 20–40 ms (unverified), single Pinpoint dropouts while turning could leave large errors; check the lag in logs before worrying.
  - Review #5 handling (`eac1300`): with two gyros, "Pinpoint drifting while the robot is still" and "IMU frozen while it turns slowly" look the same. A wheel-encoder referee (wheels' turn = −0.0862°·`turnCw` ticks, fit of the 9 Nov logs) was built and run in the simulation (frozen sensors fixed, but restarts/drift kept trading failures); kept out, saved in the session folder `files/fieldheading_v5_wip/`. Instead: **`PinpointInitCheck`** in TeleOp INIT (robot still: Pinpoint turning > 1° more than the IMU while the IMU < 3° → `pinpoint.reset()`, INIT line `Pinpoint gyro`, CONFIG `pinpoint_recal`), and **hold Options 1 s = field-centric on/off** in the match (`DRIVE_MODE` event). Documented as limits in the `FieldHeading` javadoc. Review #6 (`4cbfef3`): the INIT check adds up turns loop by loop with IMU flips taken out (a yaw chattering at ±90 hid a drift), and TeleOp reads the IMU every loop (holding Options can turn field-centric on at any time; without a Pinpoint the heading wasn't tracked while off).
  - Real data used: the IMU at rest repeats its exact reading 15 % of the time (AUTO logs, 841/5498 still pairs), so "exactly repeating" alone doesn't mean frozen.
  - Commits: `master` `b2c4020`, `b01f98f`, `21c7f00`, `52feac1`, `3e46835`, `eac1300`, `4cbfef3`, `0878a3c`, `fcd4a8b`; `auto-work` `a56b08b`, `b680235`, `4827222`, `91e6207`, `60254d4`, `367e198`, `4a9e9f1`, `6410275`, `773125a`, `4c5d916`. All pushed 10 Nov.
  - Docs: README_2 field-centric section, driver card (+PDF, Options tap / hold, "don't move the robot in INIT"), shooting guide docx (PDF still stale: Word COM hangs; save by hand).
- [x] **Shooting guide PDF:** Word hung on every PDF save on 9 Nov; on 10 Nov night `ExportAsFixedFormat(path, 17)` worked (6 pages, `master` `fa1f1a9`). Recipe: `New-Object -ComObject Word.Application`, `Documents.Open(docx, $false, $true)`, export to %TEMP%, copy.
- [ ] **Empty-pocket feeder change: installed 9 Nov ~16:22** (in `auto-work` `b06abaf`). Practice with the default 0.10 s and pull the logs: count weak shots, `FEED HOLD`, `NO PAUSE` and `RELEASED` (`tools/windmill_logs.py`). If weak shots are rare, try 0.05 in Tune (LB + D-pad ←) for ~19 % less time per ball.
- [ ] Maybe a 3D-printed windmill rotor (the guide exists in goBildaStarter). Do the hand-feed test first: if balls fed by hand score well, the windmill is the problem.
- [ ] Pedro AutoTune from the start, then the Field Tour test (§8). `field-tour` now carries all match code from `master`; use `master` alone for competitions if unsure.
- [ ] Driver Hub: the user sends REV photos, video, the Hardware Client check and the log zip (§9).
- [x] Pushed to `origin` (9 Nov, user asked): `master`, `auto-work` and `field-tour` (the last two new on GitHub). Ask again before later pushes.
- [x] goBildaStarter pushed (9 Nov, `main` `0645689`; user asked): `ff0dc73` (hood PPL1, floor PPR1–3, guides, pictures, jobs, PPF1 removal, `handover_sim.py`), `d257110`, and `0645689` (electronics v2, one flywheel wheel, PPR1 v2 behind `PPR1_V2 = False`, lone-ball sims, `shots/electronics_v2/`). Left untracked on purpose: 5 old zips, `result.json`, the `.code-workspace`, `arm_work/_*`, `shots/_vp_test.png`, `shots/polycarb/` (not referenced), raw guide renders (only `shots/*_steps/final/` is committed, as before).
  - **Large pushes fail from this laptop** (home Wi-Fi corrupts long uploads: schannel `SEC_E_MESSAGE_ALTERED`, OpenSSL `bad record mac`; 94 MB never went through). What worked: push the new blobs in ~6 MB chunks as temporary commits on a temp branch (temp `GIT_INDEX_FILE` + `read-tree origin/main` + `update-index --cacheinfo` + `commit-tree`, `git push origin +<sha>:refs/heads/tmp-upload`, retry each), then `git push origin main` (retry), then delete `tmp-upload`.
- [ ] Remove the old worktree folder `C:\repo\FtcRobotController-pedro-path` (detached, was locked): `git -C C:\repo\FtcRobotController worktree remove --force C:\repo\FtcRobotController-pedro-path`.

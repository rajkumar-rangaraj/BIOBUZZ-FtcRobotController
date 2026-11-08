# MEMORY.md: project memory for AI agents

Read this first. It records what was built, why, what was measured on the robot, and what is still open, so a new agent session can continue without starting over. Keep it up to date when you change something important.

Last updated: 2026-11-03 (branch `field-tour`).

---

## 1. Team, robot, goals

- **FTC team 37102**, season **BIOBUZZ 2026-27**. The users are a coach/parent plus students. Keep explanations **short and simple**. Students find long documents boring, so give them visual cards and numbered steps.
- **Robot:** goBILDA StarterBot with REV **Control Hub + Expansion Hub**.
  - Mecanum drive with 4 × goBILDA 312 RPM motors (537.7 ticks/rev, 104 mm wheels).
  - **Flywheel launcher:** 1 motor at 1:1 ratio (28 ticks/rev, 6000 RPM). It now has **two wheels side by side** (added 31 Oct).
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
| Branches | `master` = competition code. `field-tour` = Pedro Pathing test (see §8). **The `origin` remote is behind: nothing has been pushed since `8353741`.** Ask before pushing |
| Build | `$env:JAVA_HOME='C:\Program Files\Eclipse Adoptium\jdk-17.0.20.101-hotspot'; .\gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug -q` (exit code 1 can just be a stderr deprecation note: check for `BUILD SUCCESSFUL` / `error:`) |
| Tests | JUnit in `TeamCode/src/test/...` (56 tests on `field-tour`, 51 on `master`). Results in `TeamCode/build/test-results/testDebugUnitTest/*.xml` |
| adb | `%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe`. Hub at `192.168.43.1:5555`. **Always use `-s 192.168.43.1:5555`**, because a stale "offline" device sometimes appears |
| Wi-Fi | The laptop has 2 adapters: `Wi-Fi` = home (Sweethome 2.4), `Wi-Fi 5` = robot. Robot SSID **`37102-RC`**. Reconnect with `netsh wlan connect name="37102-RC" interface="Wi-Fi 5"` |
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
| `BioBuzzConstants` | All tunables (launcher presets NEAR 1250 / FAR 1450 t/s, trim step 15, windmill power 0.40, AUTO route, aim, Pinpoint …) |
| `BioBuzzRobot` | Hardware wrapper (drive, launcher PIDF velocity, `launcherReady` = within 100 t/s, intake, indexer, arm) |
| `BioBuzzTeleOp` | Driver TeleOp, one gamepad (button list in the file header) |
| `BioBuzzAutoBase` + `AutoRoute` + `MotionController` | AUTO (RED/BLUE) |
| `ShotFeeder` | Pocket-by-pocket windmill feed: turns one pocket, then waits `minInterval` ("pause") and for the launcher to be ready. Jam detection, then backs off |
| `ContinuousFeeder` | Experimental continuous feed. **Not used** (too fast: 8 pockets in 1.2 s, only 1–2 balls launched) |
| `HiveVision` | Camera. **SDK 12 cluster aiming:** uses `AprilTagGameDatabase.getBioBuzzTagLibrary()`. Each CELL = a cluster (RED SCORING 30-33, RED AUDIENCE 34-37, BLUE AUDIENCE 38-41, BLUE SCORING 42-45). The pose = centre of the CELL **opening**, even with only 2 of 4 tags visible (averaging single tags aimed at a corner). `bearingDeg` includes the **aim trim** (+ = robot/shots turn LEFT); `rawBearingDeg` doesn't |
| `TeleOpSettings` | Robot-side settings file `/sdcard/biobuzz_teleop.txt` (see below) |
| `BioBuzzTune` | Live tuning of aim trim, windmill power and pause; counts balls per pocket from flywheel dips; SHARE saves |
| `BioBuzzShotLog`, `BioBuzzVisionTest`, `BioBuzzSystemCheck`, `BioBuzzWiringCheck`, `BioBuzzCalibration`, `BioBuzzArmSetup`, `BioBuzzPinpointSetup` | Test and setup tools |
| `BioBuzzDiag` | TeleOp diagnostics CSV (`diag_teleop_*`, `events_teleop_*`) |
| `ArmCalibration`, `PinpointCalibration` | Saved calibrations `/sdcard/biobuzz_arm.txt` (deploy 0.16, dir +1), `/sdcard/biobuzz_pinpoint.txt` |

### `/sdcard/biobuzz_teleop.txt` (read by TeleOp, AUTO and Tune; survives reinstalls)

```
alliance=RED|BLUE              # TeleOp INIT: Circle/B = RED, Cross/A = BLUE; AUTO sets it too
spot_range_in / spot_speed     # SHOT SPOT: camera range to CELL opening + flywheel t/s (TeleOp SHARE)
spot_ref=cluster               # spots saved before the cluster change are ignored
aim_trim_deg                   # + = shots LEFT (seen from behind the launcher)
feed_power / feed_pause_sec    # windmill power while turning a pocket / pause after each pocket
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
| Left stick | drive |
| Right stick | turn |
| L1 | slow |
| R2 / L2 | intake in / out |
| **R1 hold** | FIRE (also boosts the intake) |
| **□ hold** | aim at our CELL; with a saved spot it also drives to the spot distance and sets the spot speed, then rumbles "ON SPOT - FIRE" |
| △ | flywheel pre-spin |
| ✕ | flower arm |
| ○ hold | unjam |
| D-pad ↑ / ↓ | FAR / NEAR presets |
| D-pad → / ← | speed ±15 t/s |
| **SHARE** (Back) | save the SHOT SPOT |
| **L1 + SHARE** | also teach AUTO this exact position (Pinpoint zeroed at TeleOp INIT, so INIT at the AUTO start) |
| INIT: ○ / ✕ | RED / BLUE |
| INIT: SHARE | diagnostics on/off |

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
- **Bambu P2S:** sign settings are 3 walls, 100 % infill (**Rectilinear**; zig-zag doesn't allow 100 %), PETG 250/70 °C, Textured PEI, no supports. Opening a job in Bambu Studio can reset Strength settings, so check them before sending.
- **Field CAD:** `field_cad/x/field-cad-step.step` (35 MB). It can be read in Python with `cadquery-ocp` (installed: `from OCP.STEPCAFControl import STEPCAFControl_Reader`). Units are mm, **Y up**, floor = X–Z.

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
  - **placeholder `ForesightConfig`** (from the Pedro docs example, NOT this robot)
  - `FORESIGHT_TUNED = false`, a safety flag: the tour won't drive until it's true
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

**Status (2026-11-03):**
- **Installed on the robot:** the `field-tour` build (commit `7484134`, Pedro + AutoTune + Field Tour). It also contains all the match code, so TeleOp and AUTO work as on `master`. **Reinstall `master` before competitions.**
- **AutoTune progress (2 Nov):**
  - The Pinpoint procedure gave `xPodOffset -1.847 in`, `yPodOffset -5.917 in`, 4-bar pods, both directions FORWARD. This is now in `Constants.localizerConfig` (commit `7484134`).
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

## 8b. Branch `auto-work`: new Pedro AUTO (7 Nov, not on the robot yet)

`field-tour` merged into `auto-work` (Pedro + FieldMap). New package `teamcode/auto/`; team card `AUTO.md`.
- **User decisions (7 Nov):** shoot the pre-loads first, then collect our GARDEN, stretch = shoot again, PARK always (deadline 24 s, INIT LB/RB 18–27). The LOADING ZONE is split into slot **A (corner end)** / **B (middle end)**, agreed with the partner before each match. Two start spots picked in INIT. Don't hit the partner.
- **Field CAD read with OCP** (`C:\repo\goBildaStarter\field_cad\x\field-cad-step.step`; script in the session folder `cad_probe.py`). CAD x = field X, CAD z = −field Y, CAD y = up (mm). RED: LOADING ZONE x −70.5…−59.1, y 24.9…45.6; GARDEN = 4 POLLEN against the audience wall, y −69.3, x −69.3…−60.6; **the up CELL at the start (RED AUDIENCE) is at x −23.3…−2.1, y −1.6…−20.1, and its opening faces the AUDIENCE wall**, so the straight-on spot is x −12.7 with the intake against the audience wall. BLUE = RED turned 180°.
- **Rules applied:** G304 (start touching the wall, own side, not in the zone, not on a FLOWER), G407 (max 4 balls: GARDEN only after the windmill is empty), G417 (no shots into a DOWN CELL: camera `heightIn()` ≥ 45), G402 (own side only).
- `AutoPlanner`: routes, time estimates, `parkStartSec` = min(user deadline, 30 − 1 − drive time to the slot), footprint check (walls, own side, HIVE frame + 2 in, FLOWERs + 1 in, partner slot + 1.5 in). The tests park from 100+ points along every route.
- Op mode: Pedro drive (`linear` heading = the short way), aim by `follower.hold` + camera bearing, fire with ShotFeeder/BallCounter, BLOCKED 1 s → park (or wait and re-plan while parking). Without Pedro or Pinpoint READY: fallback LEAVE 8 in on the wheel encoders. Old AUTO renamed `OLD AUTO RED/BLUE`.
- Planned times (S1 start): fire done ~9 s, GARDEN ~13.5 s, back at the spot ~16 s, park 4.2–4.9 s from 24 s.
- **To check on the field:** robot half-length/half-width (9/9 assumed), intake reach in the GARDEN sweep (`GARDEN_SIDE_GAP_IN`), the CELL height threshold, parking next to a partner.

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

- [ ] Aim test (□ + 10 balls from the spot) to decide between camera offset and driver aim. Then set `aim_trim_deg` or retrain the drivers. Consider "R1 also aims and only feeds when on target".
- [ ] Set `feed_pause_sec` to about 0.6 (BioBuzz Tune or adb sed).
- [ ] Maybe a 3D-printed windmill rotor (the guide exists in goBildaStarter). Do the hand-feed test first: if balls fed by hand score well, the windmill is the problem.
- [ ] Pedro AutoTune from the start, then the Field Tour test (§8). Reinstall `master` before matches (the robot currently runs `field-tour`).
- [ ] Driver Hub: the user sends REV photos, video, the Hardware Client check and the log zip (§9).
- [ ] Push commits to `origin` (ask the user first).
- [ ] goBildaStarter has an uncommitted README change and several zip files.

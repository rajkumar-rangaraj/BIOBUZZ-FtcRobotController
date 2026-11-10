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
| Left stick | drive |
| Right stick | turn |
| L1 | slow |
| R2 / L2 | intake in / out |
| **R1 hold** | FIRE (also boosts the intake). Waits for READY (`LauncherGate`) and, with the CELL in view, for the aim (`AimGate`, shows FIRE HELD + why) |
| **□ hold** | aim at our CELL; with a saved spot it also drives to the spot distance, then rumbles "ON SPOT - FIRE". Also MANUAL → AUTO shot speed. The spot speed is used only with the auto speed off (and by AUTO) |
| △ | flywheel pre-spin |
| ✕ | flower arm |
| ○ hold | unjam |
| D-pad ↑ / ↓ | MANUAL FAR / NEAR (camera speed off until □) |
| D-pad → / ← | trim ±5 t/s since 9 Nov (was ±15; stays on top of AUTO) |
| LB + D-pad | shot result for the diag log (`ShotResults`, 9 Nov): ↑ IN, ↓ SHORT, → LONG, ← SIDE; goes to the oldest unlabelled ball of the last 10 s |
| **SHARE** (Back) | save the SHOT SPOT |
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
    - **The IMU heading glitched** for single slow samples (e.g. −58 → +89 → −68° within 0.5 s). TeleOp doesn't use it; AUTO does. Watch it.
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
    - **Manual (2026-27 BIOBUZZ, TU08):**
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
- [ ] **Front-left gap (lone ball, §5.12):** the user is testing a duct-tape filler between the lower roller and PPR1's front edge. If it works, design the printed strip.
- [ ] Fusion trial: the window title said "Trial ends in 6 days" on 9 Nov. Make sure the team keeps a working Fusion licence (CAD is in Fusion v26; the v2.1 save on 9 Nov ~16:55 hadn't shown up as v27 in the cloud by 17:10: check Fusion's Job Status, or re-run `electronics_v2_build.py` and save).
- [ ] Straight-on check for the SHOT SPOT (cluster yaw), if aiming from the side keeps missing.
- [ ] Set `feed_pause_sec` to about 0.6 (BioBuzz Tune or adb sed).
- [ ] **The last ball:** PPR1 v2 was rejected (§7). The `ShotFeeder` no-pause change is done (§5.14); it also helps the vane-throw lone-ball problem (§5.12 model: no pause → never fed 6 % vs 37 %).
- [x] **Installed `auto-work` `331978d`** (9 Nov 22:29, see the install record). Next: practice with LB + D-pad labels after each ball, pull the logs, run `python tools/shot_results.py robot_logs <since>` and `--write` when the proposals look right. Install from one session at a time, always the newest `auto-work`, never an older build over the AUTO session's commits.
- [ ] **Re-export `BioBuzz_Shooting_Guide.pdf`** from the new `.docx` (`1f157a9`). On 9 Nov ~21:20 Word's `ExportAsFixedFormat` hung behind a Windows "Enter your PIN" prompt (locked screen or Office sign-in). Open the docx in Word and Save as PDF, or re-run the Word COM export when the user is at the PC.
- [ ] **Empty-pocket feeder change: installed 9 Nov ~16:22** (in `auto-work` `b06abaf`). Practice with the default 0.10 s and pull the logs: count weak shots, `FEED HOLD`, `NO PAUSE` and `RELEASED` (`tools/windmill_logs.py`). If weak shots are rare, try 0.05 in Tune (LB + D-pad ←) for ~19 % less time per ball.
- [ ] Maybe a 3D-printed windmill rotor (the guide exists in goBildaStarter). Do the hand-feed test first: if balls fed by hand score well, the windmill is the problem.
- [ ] Pedro AutoTune from the start, then the Field Tour test (§8). `field-tour` now carries all match code from `master`; use `master` alone for competitions if unsure.
- [ ] Driver Hub: the user sends REV photos, video, the Hardware Client check and the log zip (§9).
- [x] Pushed to `origin` (9 Nov, user asked): `master`, `auto-work` and `field-tour` (the last two new on GitHub). Ask again before later pushes.
- [x] goBildaStarter pushed (9 Nov, `main` `0645689`; user asked): `ff0dc73` (hood PPL1, floor PPR1–3, guides, pictures, jobs, PPF1 removal, `handover_sim.py`), `d257110`, and `0645689` (electronics v2, one flywheel wheel, PPR1 v2 behind `PPR1_V2 = False`, lone-ball sims, `shots/electronics_v2/`). Left untracked on purpose: 5 old zips, `result.json`, the `.code-workspace`, `arm_work/_*`, `shots/_vp_test.png`, `shots/polycarb/` (not referenced), raw guide renders (only `shots/*_steps/final/` is committed, as before).
  - **Large pushes fail from this laptop** (home Wi-Fi corrupts long uploads: schannel `SEC_E_MESSAGE_ALTERED`, OpenSSL `bad record mac`; 94 MB never went through). What worked: push the new blobs in ~6 MB chunks as temporary commits on a temp branch (temp `GIT_INDEX_FILE` + `read-tree origin/main` + `update-index --cacheinfo` + `commit-tree`, `git push origin +<sha>:refs/heads/tmp-upload`, retry each), then `git push origin main` (retry), then delete `tmp-upload`.
- [ ] Remove the old worktree folder `C:\repo\FtcRobotController-pedro-path` (detached, was locked): `git -C C:\repo\FtcRobotController worktree remove --force C:\repo\FtcRobotController-pedro-path`.

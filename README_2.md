# BIOBUZZ — FTC Robot Code

Team BIOBUZZ · FIRST Tech Challenge 2026-27 season

Forked from [FIRST-Tech-Challenge/FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController).

Our code lives in `TeamCode/`. Everything under `FtcRobotController/` is FIRST's — don't edit it, so upstream season updates merge cleanly.

---

## What you need

| Thing | Version / notes |
|---|---|
| Android Studio | Quail 3 (2026.1.3) or later, **Stable** channel |
| JDK | **Temurin 17** — installed separately, not the bundled JDK |
| Hardware | REV Control Hub, REV Driver Hub, gamepad |

---

## First-time laptop setup

### 1. Install Android Studio

Stable channel only. Not Canary, not Beta.

### 2. Install JDK 17

Download Eclipse Temurin 17 from <https://adoptium.net/temurin/releases/?version=17> — pick the **JDK** package (not JRE) and use the `.msi` installer on Windows.

The JDK bundled with Android Studio is too new for this project. You must install 17 separately.

### 3. Clone this repo

```powershell
cd C:\repo
git clone https://github.com/rajkumar-rangaraj/BIOBUZZ-FtcRobotController.git
```

Keep the path short and **outside OneDrive** — sync conflicts with Gradle's `build/` directory cause locked-file errors that are painful to diagnose.

### 4. Point Gradle at JDK 17

Open the project, then:

**File → Settings → Build, Execution, Deployment → Build Tools → Gradle → Gradle JDK** → select `temurin-17`.

If it isn't listed, use **Download JDK…** → Version 17, Vendor: Eclipse Temurin.

### 5. Add platform-tools to PATH

```powershell
[Environment]::SetEnvironmentVariable(
  "Path",
  [Environment]::GetEnvironmentVariable("Path", "User") + ";$env:LOCALAPPDATA\Android\Sdk\platform-tools",
  "User"
)
```

Restart the terminal (and Android Studio) afterward. Then `adb` works without the full path.

### 6. First build

On your **normal Wi-Fi** — this needs internet:

```powershell
.\gradlew assembleDebug
```

Decline any prompt to upgrade Gradle or AGP. The project pins its versions deliberately.

---

## Deploying to the robot

The Control Hub broadcasts its own Wi-Fi with **no internet**. Gradle needs internet. So build and deploy are separate steps on different networks.

### Build (normal Wi-Fi)

```powershell
.\gradlew assembleDebug
```

Output lands at `TeamCode\build\outputs\apk\debug\TeamCode-debug.apk`.

### Deploy (Control Hub Wi-Fi)

1. Power the Control Hub from the battery. Wait for the LED to go **green**.
2. Connect the laptop to the Hub's Wi-Fi.
3. Then:

```powershell
adb connect 192.168.43.1:5555
adb devices          # must show "device", not "offline"
adb install -r TeamCode\build\outputs\apk\debug\TeamCode-debug.apk
```

Or hit **Run** in Android Studio with `TeamCode` selected and the Control Hub as the target — same result, one step.

### Then on the Driver Hub

1. Confirm it shows the Control Hub name and a connected state.
2. **Configure Robot** → select the config → **Activate**. Saving is not activating.
3. Pick your op mode from the TeleOp dropdown → **INIT** → **▶**.
4. Bind the gamepad: hold **Options** + press **○** for gamepad 1.

---

## Robot wiring and configuration

Printable version (tables, wiring rules and cut-out cable tags): `BioBuzz_Wiring_Sheet.pdf`, rebuilt with `node tools/make_wiring_sheet.js`. Keep it in step with this section.

Plug every motor and servo into the port below, and type the **name** exactly as shown when you create the robot configuration on the Driver Hub. Names are case-sensitive; a typo crashes the OpMode at INIT.

"Left", "right", "front" and "back" are as seen from **behind the robot looking toward the intake** (the intake/flower-arm side is the FRONT, the launcher/camera side is the BACK).

**Simple rule: everything on the LEFT goes to the Control Hub, everything on the RIGHT goes to the Expansion Hub.** The intake motor, the indexer and the flower arm servo sit on the right, so they go to the Expansion Hub. The launcher sits in the middle and goes to the Control Hub, which leaves one spare motor port there.

### Control Hub (left side)

| Port | Device | Configuration type | Name |
|---|---|---|---|
| Motor 0 | Front-left drive wheel | goBILDA 5202/3/4 series | `left_front_drive` |
| Motor 1 | Back-left drive wheel | goBILDA 5202/3/4 series | `left_back_drive` |
| Motor 2 | Launcher / flywheel motor, 1:1 (middle) | goBILDA 5202/3/4 series | `launcher` |
| Motor 3 | spare | — | — |
| Servo 0 | Left corner intake wheel servo | Continuous Rotation Servo | `left_intake_servo` |
| I2C bus 0 | Built-in IMU (already in the default config) | REV internal IMU (set automatically) | `imu` |
| I2C bus 1 | goBILDA Pinpoint odometry computer (**optional**, AUTO only; without it AUTO uses the wheel encoders) | goBILDA Pinpoint Odometry Computer | `pinpoint` |
| USB 2.0 port | Logitech C270 webcam | Webcam | `Webcam 1` |

### Expansion Hub (right side)

| Port | Device | Configuration type | Name |
|---|---|---|---|
| Motor 0 | Front-right drive wheel | goBILDA 5202/3/4 series | `right_front_drive` |
| Motor 1 | Back-right drive wheel | goBILDA 5202/3/4 series | `right_back_drive` |
| Motor 2 | Intake roller motor | goBILDA 5202/3/4 series | `intake` |
| Motor 3 | Windmill / indexer motor, 435 RPM | goBILDA 5202/3/4 series | `indexer` |
| Servo 0 | Right corner intake wheel servo | Continuous Rotation Servo | `right_intake_servo` |
| Servo 1 | Flower arm servo (2000-0025-0002, **servo mode**) | Servo | `flower_arm` |

### Wiring rules

- **Every motor's encoder cable goes in the encoder port with the same number as its power port** (motor in port 2 → encoder in port 2). All 4 drive motors, the launcher and the indexer need their encoders; the intake doesn't.
- Connect the Expansion Hub to the Control Hub with the RS485 cable, and give it its own power cable from the Control Hub. Its address must not be the same as the Control Hub's (the Driver Hub shows this when you scan).
- The flower arm servo must be switched to **servo (position) mode** with the goBILDA servo programmer before you install it. In continuous mode it will spin instead of stopping at the pads.
- After replacing the servo cradle (PPA5), the arms (PPA2L/PPA2R) or the servo horn, run `BioBuzz Arm Setup` again. The servo positions shift when the horn goes back on a different spline tooth.
- Plug each servo cable so its black (ground) wire lines up with the ground pin marked on the hub. A reversed servo plug doesn't move.
- After any wiring change, run `BioBuzz Wiring Check` (robot on a box), then `BioBuzz System Check`. It spins each drive motor alone so you can confirm the right wheel moves, and checks every encoder.

---

## Gotchas

**Disconnect the laptop from the Hub's Wi-Fi when you're done deploying.** The Control Hub allows a limited number of clients. If the laptop holds a slot, the Driver Hub can't connect and reports "Network: unknown, disconnected."

**Never `adb uninstall` against the Control Hub.** It removes the Robot Controller app and wipes the robot configuration. Recovery means reinstalling the APK and rebuilding the config by hand.

**Check `adb devices` before deploying.** If both hubs are connected, it's easy to install the Robot Controller onto the Driver Hub by mistake. One device at a time.

**"Signature conflict" on first install is expected.** The Hub ships with FIRST's factory-signed app; yours is debug-signed. Accept the uninstall — it only wipes robot configs, and on the first install there are none.

**Robot Controller and Driver Station versions must match.** Both come from the same SDK release. Update them in the same session, Driver Station first.

**Config names are strings.** `hardwareMap.get(DcMotor.class, "test_motor")` is not compile-time checked. A typo builds fine and crashes at INIT.

---

## Writing an op mode

Op modes go in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

```java
@TeleOp(name = "Motor Test", group = "Test")
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Runs at INIT
        DcMotor motor = hardwareMap.get(DcMotor.class, "test_motor");

        waitForStart();

        // Runs at PLAY
        while (opModeIsActive()) {
            motor.setPower(-gamepad1.left_stick_y);
            telemetry.addData("Power", motor.getPower());
            telemetry.update();
        }
    }
}
```

- `@TeleOp` / `@Autonomous` are what put it in the dropdown.
- Everything before `waitForStart()` runs at INIT; everything after runs at PLAY.
- You write your own loop. Nothing calls `loop()` for you — and it must exit when `opModeIsActive()` goes false, or a watchdog kills it.
- Stick Y is **inverted** — forward is negative, hence the minus sign.
- `telemetry.update()` is required or nothing appears on screen.

---

## Testing

**On the laptop (no robot needed):**

```powershell
.\gradlew :TeamCode:testDebugUnitTest
```

Tests are in `TeamCode/src/test/java/`. They check the mecanum wheel math and drive a simulated robot (motor lag, mecanum kinematics, encoders, IMU) forward, back, sliding left/right and turning. They also run the real AUTO route (`AutoRoute`) and check it ends off the wall and in the LOADING ZONE before 30 s. Run them before every deploy.

**On the robot:** run `BioBuzz System Check`. The simulation can't see wiring mistakes such as swapped motor names or wheels in the wrong corners, so a person confirms each wheel and each move.

**On the field:** follow [BIOBUZZ_TEST_PLAN.md](BIOBUZZ_TEST_PLAN.md). It uses `BioBuzz Vision Test`, `BioBuzz Shot Log` and the AUTO logs (all saved on the Control Hub in `/sdcard/BioBuzzLogs/`), with pass targets for each stage.

---

## Diagnostics logs (real-robot runs)

The robot saves logs on the Control Hub in `/sdcard/BioBuzzLogs/`:

| File | Written by | What's in it |
|---|---|---|
| `diag_teleop_*.csv` | `BioBuzz TeleOp` with diagnostics ON | 20 rows/s: sticks, triggers and buttons, all 4 wheel powers and encoders, distance driven, intake / indexer power and windmill ticks, flywheel target, speed and ready flag, flower arm state and command, camera target, battery, IMU heading, loop time |
| `events_teleop_*.csv` | same | One row per event: start settings, launcher on/off, FIRE press/release, launcher ready, arm deploy/stow/refused, aim press, target found/lost, unjam, warnings, 20 s warning, stop |
| `auto_RED_*.csv`, `auto_BLUE_*.csv` | every AUTO run | Step, distance, heading, flywheel, windmill, camera target over time, plus the final result |
| `systemcheck_*.csv` | `BioBuzz System Check` | Every PASS / FAIL / SKIP line with its fix hint |
| `wiring_*.csv` | `BioBuzz Wiring Check` | Per motor: port, encoder ticks, current, PASS/FAIL with the cause; hubs, IMU, camera, battery |
| `vision_*.csv`, `shots_*.csv` | Vision Test / Shot Log | See the test plan |

**Diagnostics switch:** in `BioBuzz TeleOp` INIT, press **Back / Share** to turn diagnostics ON or off; the INIT screen shows which. It's ON by default (`DIAG_DEFAULT_ON` in `BioBuzzConstants`) and stays as you set it until the robot app restarts. Logging writes at most 20 rows a second and reads the battery and IMU only twice a second, so the loop stays fast. Data is saved to storage every second, so a power cut loses at most 1 s.

**Getting the logs to the laptop:**
1. Connect the laptop to the Control Hub Wi-Fi.
2. From this folder, run `.\tools\pull_robot_logs.ps1`. Add `-Clear` to delete the CSVs from the hub afterwards.
3. The files land in `robot_logs\<date_time>\`, which git ignores. They include the Robot Controller's own `robotControllerLog.txt`, which shows crashes and configuration errors.
4. To get help checking a run, share that folder path.

---

## Pulling season SDK updates

```powershell
git fetch upstream
git merge upstream/master
```

`upstream` is FIRST's repo; `origin` is ours. When a new season SDK drops, merge it and update both hub apps to match.

---

## Shooting guide

Printable driver guide (one shooting spot, speed tuning with Shot Log, camera/AIM check, driver rule, TeleOp button card): `BioBuzz_Shooting_Guide.pdf`, rebuilt with `node tools/make_shooting_guide.js`.

## Current op modes

| Op mode | What it does |
|---|---|
| `BioBuzz AUTO RED` / `BioBuzz AUTO BLUE` | LEAVE, launch the 4 preloads into our upward CELL, then AUTO PARK in our LOADING ZONE. Selects `BioBuzz TeleOp` afterwards. |
| `BioBuzz TeleOp` | Competition driver control, one gamepad (controls listed at the top of `BioBuzzTeleOp.java`) |
| `BioBuzz Arm Setup` | Guided flower-arm setup, one step per screen: fit the arm level, move it down onto the pads, test up/down, press X to save. Saved on the Control Hub (`/sdcard/biobuzz_arm.txt`), so no code change or re-deploy is needed |
| `BioBuzz Calibration` | Measures drive distance, IMU direction, windmill direction, flower-arm servo positions and camera range/bearing |
| `BioBuzz System Check` | On-robot integration test: each wheel, forward/back/slide right/slide left/turns, intake, windmill, launcher, arm. Prints PASS/FAIL with fixes. Run it after every wiring or configuration change. |
| `BioBuzz Wiring Check` | Robot on a box: spins each motor alone and reads every encoder and the motor current. Finds missing motor cables, missing or swapped encoder cables and reversed directions; also checks both hubs, IMU and camera. Run it first after any wiring change. |
| `BioBuzz Vision Test` | Measures camera detection rate, bearing and distance against tape-measured truth; logs to CSV |
| `BioBuzz Shot Log` | Fires one ball at a time and logs speed, battery, camera distance and IN/MISS; logs to CSV |
| `TEST Indexer + Flywheel` | Tunes flywheel speed and windmill feed power |
| `HelloWorld` | Telemetry only — proves the toolchain and the DS link |
| `MotorTest` | Single motor on the left stick, with encoder position |

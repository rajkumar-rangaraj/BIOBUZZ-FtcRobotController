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

## Pulling season SDK updates

```powershell
git fetch upstream
git merge upstream/master
```

`upstream` is FIRST's repo; `origin` is ours. When a new season SDK drops, merge it and update both hub apps to match.

---

## Current op modes

| Op mode | What it does |
|---|---|
| `HelloWorld` | Telemetry only — proves the toolchain and the DS link |
| `MotorTest` | Single motor on the left stick, with encoder position |

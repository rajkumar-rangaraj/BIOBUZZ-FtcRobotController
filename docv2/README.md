# BIOBUZZ v2 robot code – start here

The v2 robot code (indexer v2: queue, gate, two flywheel motors) is on branch **`v2`**, in the folder
`C:\repo\FtcRobotController-v2`. The old robot's code is kept, untouched, under the tag **`biobuzz-v1`**.

Follow these documents **in order**. Do not skip ahead: each step checks something the next one relies on.

| # | Document | What you do | Status |
|---|---|---|---|
| **1** | [01_FIRST_TEST.md](01_FIRST_TEST.md) | Install v2, make the v2 Driver Hub configuration, run **v2 Hardware Test**, fill in the results sheet | **Do this first** |
| 2 | 02_SETUP.md (coming) | Run **v2 Setup**: confirm directions, gate and arm positions, beams, queue ticks per ball, jam current; it saves the calibration file | Not written yet |
| 3 | 03_SYSTEM_CHECK.md (coming) | Pre-match check | Not written yet |
| 4 | 04_TELEOP.md (coming) | Driving and the 4-ball volley | Not written yet |

## Plans (for review, not robot steps)

| Document | What it is |
|---|---|
| [V2_SHOOTING_PLAN.md](V2_SHOOTING_PLAN.md) | Fast, accurate shooting: flywheel physics, flywheel controller, feed cadence, Pedro paths, aim, CELL UP check, AUTO and TeleOp, tests, questions for the reviewer |

## What is on the Driver Hub with v2

Only the v2 programs. Every v1 program is still in the code, renamed **"v1 …"** and switched off with `@Disabled`
(delete that one line to bring one back).

| Program | Use |
|---|---|
| **v2 Hardware Test** | Test every motor, servo, sensor and the camera one at a time |

## Rules

- Robot **on a box** (wheels off the ground) for every test until a document says otherwise.
- One person runs the Driver Hub, one person watches the robot. Hands away from the flywheel and the gate.
- Write every result down. The sheet in [01_FIRST_TEST.md](01_FIRST_TEST.md) is what v2 Setup asks for.

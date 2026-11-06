"""Shot model for the printed launcher hood (PPL1): which flywheel speed scores in the upward HIVE CELL.

Run:  python tools/shot_model.py

Geometry (field CAD, upward CELL at match start; checked 4 Nov 2026):
  lower lip 1358 mm (53.5 in), upper lip 1654 mm (65.1 in) above the tiles, upper lip 198 mm further from the robot
  (opening leans back 56 deg); opening centre 1506 mm (59.3 in). The CELL's AprilTags are at 1262 mm (49.7 in).
  Ball exit (hood PPL1 lip): centre 227.5 mm above the tiles, launch angle 74 deg.
Ball: 71-73 mm, 25 g. Air: 1.2 kg/m3. Drag Cd 0.5. Topspin from rolling on the hood (spin = exit speed / radius),
Magnus lift CL = S / (2.022 S + 0.981), S = r w / v (sphere fit). The model is checked with drag and spin off too.
A shot scores when the ball centre crosses the lip-to-lip line from the robot side, staying ball radius + 5 mm
clear of both lips (same rule as the launcher plan).

Flywheel speed -> ball speed: the ball rolls between the moving tread and the still hood, so with no slip it leaves at
half the tread speed: v_ball = eta * (tps / 28) * pi * 0.096 / 2. eta (slip + squeeze + the dip as the ball passes)
is about 0.85-0.93; 0.89 is the value used in the hood review. Only test shots can pin it down, so the code presets
are starting points and the drivers trim from there (15 t/s per press).
"""
import math

G = 9.81
RHO = 1.2
CD = 0.5
MASS = 0.025
LOWER = (0.0, 1.358)                 # lip positions relative to the opening centre (horizontal, height), metres
UPPER = (0.198, 1.654)
CENTRE = ((LOWER[0] + UPPER[0]) / 2, (LOWER[1] + UPPER[1]) / 2)
EXIT_H = 0.2275
WHEEL_D = 0.096
TICKS_PER_REV = 28.0
IN = 0.0254


def flight(v0, theta_deg, D, ball_d=0.073, drag=True, magnus=True, dt=0.0005):
    """Ball centre path from the exit; D = horizontal distance exit -> opening centre (m).
    Returns (scores, entry_deg, min_lip_clearance_m)."""
    r = ball_d / 2
    area = math.pi * r * r
    kd = 0.5 * RHO * CD * area / MASS if drag else 0.0
    w = v0 / r                                         # topspin rad/s, from rolling on the hood
    th = math.radians(theta_deg)
    x, y = 0.0, EXIT_H
    vx, vy = v0 * math.cos(th), v0 * math.sin(th)
    off = D - CENTRE[0]
    L = (LOWER[0] + off, LOWER[1])
    U = (UPPER[0] + off, UPPER[1])
    ux, uy = U[0] - L[0], U[1] - L[1]
    side = lambda px, py: ux * (py - L[1]) - uy * (px - L[0])   # > 0 = robot side of the lip line
    clear = 9.0
    crossed = None
    prev = side(x, y)
    t = 0.0
    while t < 2.0:
        v = math.hypot(vx, vy)
        ax, ay = 0.0, -G
        if drag:
            ax -= kd * v * vx
            ay -= kd * v * vy
        if magnus and v > 1e-6:
            S = r * w / v
            cl = S / (2.022 * S + 0.981)
            km = 0.5 * RHO * cl * area / MASS
            ax += km * v * vy                          # topspin: force perpendicular to the path, forward/down
            ay += km * v * (-vx)
        vx += ax * dt
        vy += ay * dt
        x += vx * dt
        y += vy * dt
        t += dt
        if crossed is None or t < crossed[1] + 0.03:
            clear = min(clear, math.hypot(x - L[0], y - L[1]), math.hypot(x - U[0], y - U[1]))
        s = side(x, y)
        if crossed is None and prev > 0 >= s:
            f = ((x - L[0]) * ux + (y - L[1]) * uy) / (ux * ux + uy * uy)
            if not 0 < f < 1:
                return False, None, clear
            crossed = (math.degrees(math.atan2(vy, vx)), t)
        elif crossed is None and prev < 0 <= s:
            f = ((x - L[0]) * ux + (y - L[1]) * uy) / (ux * ux + uy * uy)
            if 0 < f < 1:
                return False, None, clear              # came up into the opening from underneath
        prev = s
        if y < 0 or x > D + 1.0 or (crossed and t > crossed[1] + 0.05):
            break
    if crossed is None:
        return False, None, clear
    return clear >= r + 0.005, crossed[0], clear


def window(D, theta=74.0, **kw):
    """Scoring speed range (m/s) at horizontal distance D (m): (low, high, centre, +/- %)."""
    good = [v / 200 for v in range(500, 1600) if flight(v / 200, theta, D, **kw)[0]]
    if not good:
        return None
    # largest continuous run
    runs, cur = [], [good[0]]
    for a, b in zip(good, good[1:]):
        if b - a > 0.0051:
            runs.append(cur)
            cur = [b]
        else:
            cur.append(b)
    runs.append(cur)
    best = max(runs, key=len)
    lo, hi = best[0], best[-1]
    c = (lo + hi) / 2
    return lo, hi, c, 100 * (hi - lo) / 2 / c


def tps(v_ball, eta=0.89):
    return v_ball / (eta * math.pi * WHEEL_D / 2 / TICKS_PER_REV)


if __name__ == '__main__':
    print('Upward CELL: lips %.1f / %.1f in, opening centre %.1f in; exit %.1f in high at 74 deg\n'
          % (LOWER[1] / IN, UPPER[1] / IN, CENTRE[1] / IN, EXIT_H / IN))
    print('exit->opening   vacuum          drag only       drag+topspin (used)          flywheel t/s at eta')
    print('  (in)        centre  +/-%     centre  +/-%     low-high m/s  centre  +/-%    0.85   0.89   0.93')
    for d_in in range(28, 58, 2):
        D = d_in * IN
        a = window(D, drag=False, magnus=False)
        b = window(D, magnus=False)
        c = window(D)
        fmt = lambda w: '  --    --  ' if w is None else '%5.2f  %4.1f' % (w[2], w[3])
        tail = '' if c is None else '%5.2f-%5.2f   %5.2f  %4.1f   %5.0f  %5.0f  %5.0f' % (
            c[0], c[1], c[2], c[3], tps(c[2], 0.85), tps(c[2], 0.89), tps(c[2], 0.93))
        print('  %4d       %s      %s      %s' % (d_in, fmt(a), fmt(b), tail))
    print('\nLaunch angle sensitivity at 36 in (drag+topspin):')
    for th in (72, 73, 74, 75, 76):
        w = window(36 * IN, theta=th)
        print('  %d deg: %s' % (th, 'no window' if w is None else 'centre %.2f m/s +/-%.1f%%  (%.0f t/s at 0.89)' % (w[2], w[3], tps(w[2]))))
    print('\nBall size at 36 in: 71 mm %s, 73 mm %s' % (
        '%.2f m/s' % window(36 * IN, ball_d=0.071)[2], '%.2f m/s' % window(36 * IN, ball_d=0.073)[2]))

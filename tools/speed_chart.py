"""Speed chart: flywheel speed vs. distance from the physics model, the TeleOp speed table and labelled shots.

Run:  python tools/speed_chart.py [--ref-dist 33] [--ref-tps 1400] [--out speed_chart.png]

- Physics band: tools/shot_model.py (drag + topspin) scoring window at each exit->opening distance. The ball-speed
  factor eta is calibrated so --ref-tps is the window centre at --ref-dist (the drivers' proven shot).
- Table: AUTO_SPEED_DIST_IN / AUTO_SPEED_TPS and the NEAR / FAR presets, read from BioBuzzConstants.java.
- Dots: shots labelled with LB + D-pad (SHOT_RESULT events in robot_logs/**/events_teleop_*.csv), at their camera
  floor distance (camera ~ launcher exit, both at the back) and flywheel target.
"""
import argparse
import csv
import glob
import math
import os
import re
from concurrent.futures import ProcessPoolExecutor

import shot_model as m

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONSTANTS = os.path.join(REPO, "TeamCode", "src", "main", "java", "org", "firstinspires", "ftc", "teamcode",
                         "biobuzz", "BioBuzzConstants.java")


def read_constants():
    src = open(CONSTANTS, encoding="utf-8").read()

    def arr(name):
        body = re.search(name + r"\s*=\s*\{([^}]*)\}", src).group(1)
        return [float(x) for x in body.split(",")]

    def num(name):
        return float(re.search(name + r"\s*=\s*([\d.]+)", src).group(1))

    return (arr("AUTO_SPEED_DIST_IN"), arr("AUTO_SPEED_TPS"),
            num("LAUNCHER_NEAR_VELOCITY"), num("LAUNCHER_FAR_VELOCITY"))


def read_labels(since):
    seen, rows = set(), []
    for f in sorted(glob.glob(os.path.join(REPO, "robot_logs", "**", "events_teleop_*.csv"), recursive=True)):
        b = os.path.basename(f)
        if b in seen or b[14:22] < since:
            continue
        seen.add(b)
        with open(f, newline="", encoding="utf-8", errors="replace") as fh:
            for r in csv.reader(fh):
                if len(r) > 2 and r[1] == "SHOT_RESULT":
                    d = dict(kv.split("=", 1) for kv in r[2].split() if "=" in kv)
                    try:
                        fl, t = float(d["floor_in"]), float(d["target"])
                    except (KeyError, ValueError):
                        continue
                    if not math.isnan(fl):
                        rows.append((fl, t, d.get("result", "?")))
    return rows


def window_at(d_in):
    return d_in, m.window(d_in * m.IN)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref-dist", type=float, default=33.0, help="distance (in) where --ref-tps is proven")
    ap.add_argument("--ref-tps", type=float, default=1400.0)
    ap.add_argument("--since", default="20261107", help="labelled shots from this date (one-wheel launcher)")
    ap.add_argument("--out", default=os.path.join(REPO, "speed_chart.png"))
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    dist_tbl, tps_tbl, near, far = read_constants()
    per_tps = math.pi * m.WHEEL_D / 2 / m.TICKS_PER_REV       # ball m/s per (eta * t/s)
    ref = m.window(args.ref_dist * m.IN)
    eta = ref[2] / (args.ref_tps * per_tps)
    to_tps = lambda v: v / (eta * per_tps)

    dists = list(range(26, 67, 2))
    with ProcessPoolExecutor() as ex:
        res = dict(ex.map(window_at, dists))
    ds = [d for d in dists if res[d]]
    lo = [to_tps(res[d][0]) for d in ds]
    hi = [to_tps(res[d][1]) for d in ds]
    best = [to_tps(res[d][2]) for d in ds]
    pct = [res[d][3] for d in ds]
    ball = [res[d][2] for d in ds]

    labels = read_labels(args.since)

    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(12, 10), height_ratios=[3, 1], sharex=True)
    ax.fill_between(ds, lo, hi, color="#b7e4c7", alpha=0.7, label="Physics: scores (lip-to-lip, ball clears both lips)")
    inner_lo = [b - (b - l) / 2 for b, l in zip(best, lo)]
    inner_hi = [b + (h - b) / 2 for b, h in zip(best, hi)]
    ax.fill_between(ds, inner_lo, inner_hi, color="#52b788", alpha=0.45,
                    label="Physics: safe half (room for the ~+/-4 % ball-to-ball dip)")
    ax.plot(ds, best, color="#1b4332", lw=2.2, label="Physics: best speed (window centre)")
    ax.plot(dist_tbl, tps_tbl, "o-", color="#d62828", lw=2, ms=7, label="TeleOp table AUTO_SPEED_TPS (now)")
    for d, t in zip(dist_tbl, tps_tbl):
        ax.annotate(f"{t:.0f}", (d, t), textcoords="offset points", xytext=(0, -16), ha="center",
                    fontsize=8, color="#d62828")
    ax.axhline(near, color="#f77f00", ls="--", lw=1.2, label=f"NEAR preset {near:.0f}")
    ax.axhline(far, color="#7b2cbf", ls="--", lw=1.2, label=f"FAR preset {far:.0f}")
    ax.plot([args.ref_dist], [args.ref_tps], "*", color="gold", mec="black", ms=18, zorder=6,
            label=f"Calibration: {args.ref_tps:.0f} t/s proven at {args.ref_dist:.0f} in (eta {eta:.3f})")

    style = {"IN": ("o", "#2a9d8f"), "SHORT": ("v", "#0077b6"), "LONG": ("^", "#e63946"), "SIDE": ("x", "#6c757d")}
    for res_name, (mk, col) in style.items():
        pts = [(fl, t) for fl, t, r in labels if r == res_name]
        if pts:
            ax.scatter([p[0] for p in pts], [p[1] for p in pts], marker=mk, color=col, s=28, alpha=0.55,
                       zorder=5, label=f"Labelled shot {res_name} ({len(pts)})")

    ax.set_ylabel("Flywheel speed (ticks/s)")
    ax.set_ylim(1250, 1760)
    ax.grid(alpha=0.3)
    sec = ax.secondary_yaxis("right", functions=(lambda t: t * 60 / m.TICKS_PER_REV,
                                                 lambda r: r * m.TICKS_PER_REV / 60))
    sec.set_ylabel("Motor RPM (28 ticks/rev, 1:1)")
    ax.set_title("BioBuzz flywheel speed vs. distance to the CELL: physics model, speed table and labelled shots",
                 fontsize=12)
    ax.legend(loc="upper left", fontsize=8, ncol=2)

    ax2.plot(ds, pct, color="#1b4332", lw=2)
    ax2.set_ylabel("Scoring window +/- %")
    ax2.set_xlabel("Camera floor distance to the CELL opening (in)  ~  launcher exit -> opening")
    ax2.grid(alpha=0.3)
    ax2.axhline(4.0, color="#e63946", ls=":", lw=1.2, label="~ball-to-ball dip spread (+/-4 %)")
    ax3 = ax2.twinx()
    ax3.plot(ds, ball, color="#577590", lw=1.5, ls="--", label="ball exit speed (m/s)")
    ax3.set_ylabel("Ball exit speed (m/s)")
    h1, l1 = ax2.get_legend_handles_labels()
    h2, l2 = ax3.get_legend_handles_labels()
    ax2.legend(h1 + h2, l1 + l2, loc="upper right", fontsize=8)

    fig.text(0.01, 0.005, "Model: tools/shot_model.py (gravity + drag Cd 0.5 + Magnus topspin, CELL lips from field CAD, "
             "74 deg hood exit). Labels: LB + D-pad in BioBuzz TeleOp. Regenerate: python tools/speed_chart.py",
             fontsize=7, color="#555")
    fig.tight_layout(rect=(0, 0.015, 1, 1))
    fig.savefig(args.out, dpi=130)

    print(f"eta = {eta:.3f}  (calibrated: {args.ref_tps:.0f} t/s best at {args.ref_dist:.0f} in)")
    print("dist  physics low  best  high   +/-%   table")
    tbl = dict(zip(dist_tbl, tps_tbl))
    for d, l, b, h, p in zip(ds, lo, best, hi, pct):
        t = tbl.get(d)
        print(f"{d:4d}  {l:8.0f} {b:6.0f} {h:6.0f}  {p:4.1f}   {'' if t is None else f'{t:.0f} ({t - b:+.0f})'}")
    print(f"saved {args.out}  ({len(labels)} labelled shots)")


if __name__ == "__main__":
    main()

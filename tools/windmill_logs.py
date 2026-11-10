"""Windmill pushes in the robot's TeleOp logs: push length, stalls, and when the flywheel dips (a ball goes in).

  python tools/windmill_logs.py robot_logs [since]      since = file stamp, e.g. 20261106_0900 (default: all)

A push = diag rows with windmill power > 0 (ShotFeeder FEEDING). Its aimed pocket boundary is the logged one (note
"FEEDING <ticks>", since 6 Nov) or, in older logs, read back from the ramp (power = feed x ticks left / one pocket);
the measured stroke is the encoder travel from the push's start until the windmill is at rest (coasting included).
Things to check after the 6 Nov fixes: no push aimed more than 2 pockets ahead, strokes near 1 pocket, few late dips
(> 0.15 s after the windmill stopped), few pushes that may have fed two balls (two dips, or one extra-deep or extra-long
dip: two balls 0.1-0.3 s apart make one dip), STUCK mostly followed by a SHOT, few JAMs.
Since 9 Nov (TeleOp skips the pause after an empty pocket): FEED HOLD stops the windmill mid-pocket, so a held pocket
shows here as two short pushes. Compare NO PAUSE and HOLD counts with the shots seen to pick Tune's empty wait.
"""
import csv, glob, os, sys
import numpy as np

POCKET = 384.5 / 4
MINP = 0.15
DIP = 35                                                   # t/s below the speed before the push (AUTO_SHOT_DIP_TPS)
BIG_DIP, LONG_DIP = 160, 0.45                              # one ball (old logs): p90 120 t/s deep, 0.55 s below the line

root = sys.argv[1] if len(sys.argv) > 1 else 'robot_logs'
since = sys.argv[2] if len(sys.argv) > 2 else ''


def newest_copies(pattern, stamp_at):
    """the largest copy of each log file name (the same file is often pulled into several folders)"""
    out = {}
    for p in glob.glob(os.path.join(root, '**', pattern), recursive=True):
        n = os.path.basename(p)
        if n[stamp_at:stamp_at + 13] >= since and (n not in out or os.path.getsize(p) > os.path.getsize(out[n])):
            out[n] = p
    return out


def dips(vel, t, i0, i1, pre, t_stop):
    """separate flywheel dips (two rows below pre - DIP, over once back above pre - DIP / 2):
    (seconds after the stop, depth t/s, seconds below the line)"""
    out, k = [], i0
    while k < i1 - 1:
        if vel[k] < pre - DIP and vel[k + 1] < pre - DIP:
            s, lo = k, vel[k]
            while k < i1 and vel[k] <= pre - DIP / 2:
                lo = min(lo, vel[k])
                k += 1
            out.append((t[s] - t_stop, pre - lo, t[min(k, len(t) - 1)] - t[s]))
        k += 1
    return out


pushes = []
diag = newest_copies('diag_teleop_*.csv', 12)
for n in sorted(diag):
    with open(diag[n], newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        continue
    col = lambda k: np.array([float(r[k] or 0) for r in rows])
    t, pw, tk, vel, tgt = col('t'), col('indexer_pw'), col('indexer_ticks'), col('launcher_vel'), col('launcher_target')
    note = [r.get('note', '') for r in rows]
    starts = [i for i in range(len(rows)) if pw[i] > 0.01 and (i == 0 or pw[i - 1] <= 0.01)]
    for s_i, i in enumerate(starts):
        j = i
        while j + 1 < len(rows) and pw[j + 1] > 0.01:
            j += 1
        feed = pw[i:j + 1].max()
        logged = [float(note[k].split()[1]) for k in range(i, j + 1) if note[k].startswith('FEEDING ')]
        if logged:
            target, how = logged[0], 'logged'
        else:
            mid = [k for k in range(i, j + 1) if MINP + 0.005 < pw[k] < feed - 0.005]
            target, how = (float(np.median([tk[k] + pw[k] * POCKET / feed for k in mid])), 'ramp') if mid else (None, None)
        end_t = t[min(j + 1, len(t) - 1)]
        rest = j + 1
        while rest + 1 < len(rows) and abs(pw[rest + 1]) <= 0.01 and t[rest + 1] - end_t < 0.3:
            rest += 1                                       # coasting; a back-off (negative power) ends it
        nxt = starts[s_i + 1] if s_i + 1 < len(starts) else len(rows)
        until = min(nxt, int(np.searchsorted(t, end_t + 1.5)))
        pre = float(np.median(vel[max(0, i - 4):i + 1]))
        pushes.append(dict(file=n, t=t[i], dur=end_t - t[i], flywheel=tgt[i] > 500, how=how,
                           pockets=None if target is None else (target - tk[i]) / POCKET,
                           stroke=(tk[i:min(rest, len(tk) - 1) + 1].max() - tk[i]) / POCKET,
                           stalled=any(tk[k + 1] == tk[k] for k in range(i, j)), timeout=end_t - t[i] >= 0.98,
                           dips=dips(vel, t, i, until, pre, end_t) if tgt[i] > 500 else []))

fp = [p for p in pushes if p['flywheel']]
print('%d logs, %d windmill pushes with the flywheel on' % (len(diag), len(fp)))
L = np.array([p['pockets'] for p in fp if p['pockets'] is not None])
if len(L):
    print('aimed push length (pockets): median %.2f | > 1.5: %d | > 2: %d | longest %.1f | logged %d, from the ramp %d, unknown %d' % (
        np.median(L), (L > 1.5).sum(), (L > 2.0 + 1e-6).sum(), L.max(), sum(p['how'] == 'logged' for p in fp),
        sum(p['how'] == 'ramp' for p in fp), sum(p['how'] is None for p in fp)))
S = np.array([p['stroke'] for p in fp])
if len(S):
    print('measured forward stroke incl. coasting (pockets): median %.2f | > 1.5: %d | > 2: %d | longest %.1f' % (
        np.median(S), (S > 1.5).sum(), (S > 2.0).sum(), S.max()))
    d = np.array([p['dur'] for p in fp])
    print('push time: median %.2f s, p90 %.2f s' % (np.median(d), np.percentile(d, 90)))
print('stalled (no encoder change between two rows while powered): %d | ended by the 1 s pocket timeout: %d' % (
    sum(p['stalled'] for p in fp), sum(p['timeout'] for p in fp)))
first = [p['dips'][0][0] for p in fp if p['dips']]
print('first flywheel dip: while the windmill turns %d | up to 0.15 s after it stopped %d | LATE (> 0.15 s) %d | none %d' % (
    sum(x <= 0 for x in first), sum(0 < x <= 0.15 for x in first), sum(x > 0.15 for x in first), sum(not p['dips'] for p in fp)))
print('maybe two balls in one push: two or more separate dips %d | one dip deeper than %d t/s %d | longer than %.2f s %d' % (
    sum(len(p['dips']) >= 2 for p in fp), BIG_DIP, sum(any(d[1] > BIG_DIP for d in p['dips']) for p in fp),
    LONG_DIP, sum(any(d[2] > LONG_DIP for d in p['dips']) for p in fp)))

events = newest_copies('events_teleop_*.csv', 14)
kinds, after_stuck = {}, {}
for n in sorted(events):
    with open(events[n], newline='') as f:
        feed = [r['detail'] for r in csv.DictReader(f) if r.get('event') == 'FEED']
    for k, d in enumerate(feed):
        w = 'NO PAUSE' if d.startswith('NO PAUSE') else d.split(' ')[0]
        if w in ('STUCK', 'JAM', 'JAMMED', 'NO PAUSE', 'HOLD'):
            kinds[w] = kinds.get(w, 0) + 1
        if w == 'STUCK':
            nxt = next((e.split(' ')[0] for e in feed[k + 1:] if e.split(' ')[0] in ('SHOT', 'JAM', 'JAMMED')), 'none')
            after_stuck[nxt] = after_stuck.get(nxt, 0) + 1
print('windmill events (%d event logs): %s' % (len(events), kinds or 'none'))
if after_stuck:
    print('after STUCK came:', after_stuck)

"""Shot results -> AUTO speed table (TeleOp diagnostics, Nov 2026).

  python tools/shot_results.py robot_logs [since]            report (since = file stamp, e.g. 20261110_0900)
  python tools/shot_results.py robot_logs [since] --write    also write the proposed AUTO_SPEED_TPS into BioBuzzConstants

In TeleOp every ball the flywheel launches is logged (events_teleop_*.csv):
  BALL  when it is counted: camera floor distance to the CELL, the speed table's value there, the driver's trim, the
        target, ready / fallback, how hard the robot was driving / turning, ...
  GRAB  when its flywheel dip is over (GrabTracker): the real speed just before the wheel took it ("pre"), launch error
        (pre - target), depth, the share of the flywheel's energy it took, recovery, drops (2+ = two balls in one dip),
        where the windmill was.
The driver or a scout adds what it did with LB + D-pad (event SHOT_RESULT): IN, SHORT (too slow), LONG (too fast) or SIDE
(left / right / bounced out: aim, not speed). IN shots from BioBuzz Shot Log (shots_*.csv) count too.

The fit uses the REAL speed at the grab, not the target: on a good battery the flywheel runs ~+15 t/s above its target,
on a weak one 20-50 below (7 Nov logs), so targets alone carry that error. Each ball's speed = pre - bias, where bias =
the median launch error of these logs (what the table must ask for to get that real speed). Balls whose result says
nothing about the speed are left out of the fit and listed: two balls in one dip, the target changed mid-dip, the robot
driving or turning hard, the flywheel not at speed (|launch error| > 60, fallback), and weak grabs (energy taken below
WEAK x the IN balls' median: the windmill pushed the ball in badly; a SHORT there is the hand-over, not the speed).

For every point of the AUTO speed table (AUTO_SPEED_DIST_IN / AUTO_SPEED_TPS, read from BioBuzzConstants.java):
  offset = the ball's speed - the table's speed at the ball's distance
  - IN balls: the right offset is around their median;
  - SHORT balls: it is above their offsets; LONG balls: below.
A point changes only with at least MIN_SHOTS labelled IN / SHORT / LONG balls within NEAR_IN of it, and the new table
keeps rising with distance. Install the new code afterwards (the robot reads the table from the APK).
"""
import csv, datetime, glob, os, re, statistics, sys

MIN_SHOTS = 3            # labelled IN / SHORT / LONG balls near a table point before it moves
NEAR_IN = 3.0            # a ball counts for the table point(s) within this many inches
STEP = 5                 # proposals are rounded to the D-pad trim step (t/s)
MOVING = 0.15            # BALL move / turn_cmd above this: the robot was driving or turning hard
NOT_AT_SPEED = 60        # |pre - target| above this: the flywheel was not at speed
WEAK = 0.75              # energy taken below WEAK x the IN balls' median: a weak grab
HERE = os.path.dirname(os.path.abspath(__file__))
CONSTANTS = os.path.join(HERE, '..', 'TeamCode', 'src', 'main', 'java', 'org', 'firstinspires', 'ftc', 'teamcode',
                         'biobuzz', 'BioBuzzConstants.java')


def read_table(path):
    src = open(path, encoding='utf-8').read()
    arr = lambda name: [float(v) for v in re.search(name + r'\s*=\s*\{([^}]*)\}', src).group(1).split(',')]
    return arr('AUTO_SPEED_DIST_IN'), arr('AUTO_SPEED_TPS')


def table_speed(dist, speed, d):
    if d <= dist[0]:
        return speed[0]
    for i in range(1, len(dist)):
        if d <= dist[i]:
            return speed[i - 1] + (speed[i] - speed[i - 1]) * (d - dist[i - 1]) / (dist[i] - dist[i - 1])
    return speed[-1]


def fields(detail):
    """'ball=3 result=IN floor_in=47.3 ...' -> dict"""
    return dict(kv.split('=', 1) for kv in detail.split() if '=' in kv)


def num(v):
    try:
        x = float(v)
        return x if x == x else None                   # NaN -> None
    except (TypeError, ValueError):
        return None


def newest_copies(root, pattern, stamp_at, since):
    """the largest copy of each log file (the same file is often pulled into several folders)"""
    out = {}
    for p in glob.glob(os.path.join(root, '**', pattern), recursive=True):
        n = os.path.basename(p)
        if n[stamp_at:stamp_at + 13] >= since and (n not in out or os.path.getsize(p) > os.path.getsize(out[n])):
            out[n] = p
    return out


def load_shots(root, since):
    shots, launched, ignored, grabs_all = [], 0, 0, []
    for name, path in sorted(newest_copies(root, 'events_teleop_*.csv', 14, since).items()):
        balls, results, grabs = {}, {}, {}
        for r in csv.DictReader(open(path, newline='')):
            if r.get('event') == 'BALL':
                b = fields(r['detail'])
                balls[b.get('ball')] = b
            elif r.get('event') == 'GRAB':
                g = fields(r['detail'])
                grabs[g.get('ball')] = g
            elif r.get('event') == 'SHOT_RESULT':
                s = fields(r['detail'])
                results[s.get('ball')] = s.get('result')
            elif r.get('event') == 'SHOT_RESULT_IGNORED':
                ignored += 1
        launched += len(balls)
        grabs_all += list(grabs.values())
        for k, res in results.items():
            b, g = balls.get(k, {}), grabs.get(k)
            shots.append(dict(src=name[14:29], ball=k, result=res, floor=num(b.get('floor_in')), target=num(b.get('target')),
                              trim=num(b.get('trim')), bearing=num(b.get('bearing')), source=b.get('source'),
                              battery=num(b.get('battery')), ready=b.get('ready'), fallback=b.get('fallback'),
                              move=num(b.get('move')), turn=num(b.get('turn_cmd')),
                              pre=num(g.get('pre')) if g else None,
                              err=num(g.get('launch_err')) if g else None,
                              energy=num(g.get('energy')) if g else None,
                              drops=int(num(g.get('drops')) or 1) if g else 1,
                              overlapped=g.get('overlapped') == 'true' if g else False,
                              target_changed=g.get('target_changed') == 'true' if g else False))
    for name, path in sorted(newest_copies(root, 'shots_*.csv', 6, since).items()):
        for r in csv.DictReader(open(path, newline='')):
            if r.get('result') in ('IN', 'MISS'):
                pre, tgt = num(r.get('vel_at_feed')), num(r.get('target_tps'))
                shots.append(dict(src='shotlog ' + name[6:21], ball=r.get('shot'), result=r['result'],
                                  floor=num(r.get('floor_dist_in')), target=tgt, trim=None,
                                  bearing=num(r.get('bearing_deg')), source='SHOT_LOG', battery=num(r.get('battery_v')),
                                  ready='true', fallback='false', move=None, turn=None, pre=pre,
                                  err=pre - tgt if pre is not None and tgt is not None else None, energy=None, drops=1,
                                  overlapped=False, target_changed=False))
    return shots, launched, ignored, grabs_all


def prepare(shots):
    """Each shot's speed for the fit (real speed at the grab minus the typical launch error) and why it is left out."""
    errs = [s['err'] for s in shots if s['err'] is not None and abs(s['err']) <= NOT_AT_SPEED and s['drops'] == 1]
    bias = statistics.median(errs) if errs else 0.0
    in_energy = [s['energy'] for s in shots if s['result'] == 'IN' and s['energy'] is not None]
    weak = WEAK * statistics.median(in_energy) if len(in_energy) >= 5 else None
    for s in shots:
        s['speed'] = s['pre'] - bias if s['pre'] is not None else s['target']
        why = None
        if s['drops'] >= 2 or s['overlapped']:
            why = 'two balls in one dip'
        elif s['target_changed']:
            why = 'speed changed mid-shot'
        elif (s['move'] or 0) > MOVING or abs(s['turn'] or 0) > MOVING:
            why = 'robot driving / turning'
        elif s['fallback'] == 'true' or (s['err'] is not None and abs(s['err']) > NOT_AT_SPEED):
            why = 'flywheel not at speed'
        elif weak is not None and s['energy'] is not None and s['energy'] < weak:
            why = 'weak grab (hand-over)'
        s['excluded'] = why
    return bias, weak


def propose(dist, speed, shots):
    rows = []
    for k, (d, v) in enumerate(zip(dist, speed)):
        near = [s for s in shots if s['floor'] is not None and s['speed'] is not None and not s['excluded']
                and abs(s['floor'] - d) <= NEAR_IN]
        off = {lab: [s['speed'] - table_speed(dist, speed, s['floor']) for s in near if s['result'] == lab]
               for lab in ('IN', 'SHORT', 'LONG')}
        side = sum(1 for s in near if s['result'] == 'SIDE')
        miss = sum(1 for s in near if s['result'] == 'MISS')
        lo = max(off['SHORT']) if off['SHORT'] else None   # the right offset is above every short ball's ...
        hi = min(off['LONG']) if off['LONG'] else None     # ... and below every long ball's
        est, why = None, ''
        if off['IN']:
            est = statistics.median(off['IN'])
            if lo is not None and est <= lo and (hi is None or lo < hi):
                est, why = lo + STEP, 'above the short balls'
            elif hi is not None and est >= hi and (lo is None or lo < hi):
                est, why = hi - STEP, 'below the long balls'
        elif lo is not None and hi is not None and lo < hi:
            est, why = (lo + hi) / 2, 'between short and long'
        elif lo is not None:
            why = 'only short balls: at least %+.0f' % (lo + STEP)
        elif hi is not None:
            why = 'only long balls: at most %+.0f' % (hi - STEP)
        n = len(off['IN']) + len(off['SHORT']) + len(off['LONG'])
        new = v
        if est is not None and n >= MIN_SHOTS:
            new = STEP * round((v + est) / STEP)
        elif est is not None:
            why = (why + '; ' if why else '') + 'needs %d more labelled balls' % (MIN_SHOTS - n)
        rows.append(dict(d=d, v=v, new=new, n_in=len(off['IN']), n_short=len(off['SHORT']), n_long=len(off['LONG']),
                         side=side, miss=miss, med_in=statistics.median(off['IN']) if off['IN'] else None, lo=lo, hi=hi,
                         est=est, why=why))
    for k in range(1, len(rows)):                      # keep the table rising with distance
        if rows[k]['new'] < rows[k - 1]['new']:
            rows[k]['why'] += ('; ' if rows[k]['why'] else '') + 'raised to keep the table rising'
            rows[k]['new'] = rows[k - 1]['new']
    return rows


def write_table(path, new, note):
    src = open(path, encoding='utf-8').read()
    line = re.search(r'( *public static final double\[\] AUTO_SPEED_TPS\s*=\s*\{)[^}]*(\};)[^\n]*', src)
    repl = '%s%s%s  // %s' % (line.group(1), ', '.join('%d' % v for v in new), line.group(2), note)
    open(path, 'w', encoding='utf-8', newline='').write(src[:line.start()] + repl + src[line.end():])


def grab_summary(grabs):
    """How the flywheel took the balls (all GRAB records, labelled or not)."""
    if not grabs:
        return
    val = lambda k: [x for x in (num(g.get(k)) for g in grabs) if x is not None]
    q = lambda v, p: sorted(v)[min(len(v) - 1, int(p * len(v)))]
    err, en, rec, late = val('launch_err'), val('energy'), val('recovery_s'), val('since_pocket_end_s')
    two = sum(1 for g in grabs if (num(g.get('drops')) or 1) >= 2 or g.get('overlapped') == 'true')
    during = sum(1 for g in grabs if g.get('feeding') == 'true')
    if err:
        print('grabs: launch error (real - target) median %+.0f t/s (p10 %+.0f, p90 %+.0f) | energy taken median %.2f '
              '(p10 %.2f) | recovery median %.2f s (p90 %.2f)' % (statistics.median(err), q(err, .1), q(err, .9),
                                                             statistics.median(en) if en else float('nan'),
                                                             q(en, .1) if en else float('nan'),
                                                             statistics.median(rec) if rec else float('nan'),
                                                             q(rec, .9) if rec else float('nan')))
    print('       taken while the windmill turned %d, after it stopped %d (median %.2f s later) | two balls in one dip %d' % (
        during, len(grabs) - during, statistics.median(late) if late else float('nan'), two))


def main(argv):
    args = [a for a in argv if not a.startswith('--')]
    root = args[0] if args else 'robot_logs'
    since = args[1] if len(args) > 1 else ''
    constants = next((a.split('=', 1)[1] for a in argv if a.startswith('--constants=')), CONSTANTS)
    dist, speed = read_table(constants)
    shots, launched, ignored, grabs = load_shots(root, since)
    print('%d balls launched in TeleOp logs (%d with a GRAB record), %d with a result (+ Shot Log), %d results without a ball' % (
        launched, len(grabs), len(shots), ignored))
    grab_summary(grabs)
    if not shots:
        print('No labelled balls yet: in TeleOp press LB + D-pad after a ball (UP in, DOWN short, RIGHT long, LEFT side).')
        return 0
    count = lambda lab: sum(1 for s in shots if s['result'] == lab)
    print('results: IN %d, SHORT %d, LONG %d, SIDE %d, MISS (Shot Log) %d; without a camera distance: %d' % (
        count('IN'), count('SHORT'), count('LONG'), count('SIDE'), count('MISS'), sum(1 for s in shots if s['floor'] is None)))
    bias, weak = prepare(shots)
    with_pre = sum(1 for s in shots if s['pre'] is not None)
    print('fit speed = real speed at the grab - %+.0f t/s (the median launch error) for %d balls; the target for %d without '
          'a GRAB record' % (bias, with_pre, len(shots) - with_pre))
    left_out = {}
    for s in shots:
        if s['excluded']:
            left_out.setdefault(s['excluded'], []).append(s['result'])
    for why, res in sorted(left_out.items()):
        print('  left out, %s: %d (%s)' % (why, len(res), ', '.join('%s %d' % (k, res.count(k)) for k in sorted(set(res)))))
    if weak is not None:
        print('  weak grab = energy taken below %.2f (%.0f %% of the IN balls\' median)' % (weak, 100 * WEAK))
    sides = [abs(s['bearing']) for s in shots if s['result'] == 'SIDE' and s['bearing'] is not None]
    ins = [abs(s['bearing']) for s in shots if s['result'] == 'IN' and s['bearing'] is not None]
    if sides:
        print('aim at launch, |bearing|: SIDE balls median %.1f deg, IN balls %s (many SIDE = aim trim, not speed)' % (
            statistics.median(sides), '%.1f deg' % statistics.median(ins) if ins else '-'))
    rows = propose(dist, speed, shots)
    f = lambda x: '%+5.0f' % x if x is not None else '    -'
    print('\n dist   table |  IN SHORT LONG SIDE MISS | IN offset  above  below | proposal')
    for r in rows:
        print('%4.0f in %6.0f | %3d %5d %4d %4d %4d |    %s  %s  %s | %6.0f %s %s' % (
            r['d'], r['v'], r['n_in'], r['n_short'], r['n_long'], r['side'], r['miss'], f(r['med_in']), f(r['lo']),
            f(r['hi']), r['new'], ('(%+d)' % (r['new'] - r['v'])) if r['new'] != r['v'] else '     ', r['why']))
    new = [int(r['new']) for r in rows]
    changed = [r for r in rows if r['new'] != r['v']]
    print('\nAUTO_SPEED_TPS now:      {%s}' % ', '.join('%d' % v for v in speed))
    print('AUTO_SPEED_TPS proposed: {%s}%s' % (', '.join('%d' % v for v in new), '' if changed else '  (no change)'))
    if changed and '--write' in argv:
        note = 'tools/shot_results.py %s: %d labelled balls' % (datetime.date.today().isoformat(), len(shots))
        write_table(constants, new, note)
        print('written to %s: rebuild and install' % os.path.normpath(constants))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))

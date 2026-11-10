"""Shot results -> AUTO speed table (TeleOp diagnostics, Nov 2026).

  python tools/shot_results.py robot_logs [since]            report (since = file stamp, e.g. 20261110_0900)
  python tools/shot_results.py robot_logs [since] --write    also write the proposed AUTO_SPEED_TPS into BioBuzzConstants

In TeleOp every ball the flywheel launches is logged (events_teleop_*.csv, event BALL: camera floor distance to the
CELL, the speed table's value there, the driver's trim, the target, ...). The driver or a scout adds what it did with
LB + D-pad (event SHOT_RESULT): IN, SHORT (too slow), LONG (too fast) or SIDE (left / right / bounced out: aim, not
speed). IN shots from BioBuzz Shot Log (shots_*.csv) count too (its misses say nothing about short or long).

For every point of the AUTO speed table (AUTO_SPEED_DIST_IN / AUTO_SPEED_TPS, read from BioBuzzConstants.java):
  offset = the ball's target speed - the table's speed at the ball's distance (the trim it needed, in effect)
  - IN balls: the right offset is around their median;
  - SHORT balls: it is above their offsets; LONG balls: below.
A point changes only with at least MIN_SHOTS labelled IN / SHORT / LONG balls within NEAR_IN of it, and the new table
keeps rising with distance. Install the new code afterwards (the robot reads the table from the APK).
"""
import csv, datetime, glob, os, re, statistics, sys

MIN_SHOTS = 3            # labelled IN / SHORT / LONG balls near a table point before it moves
NEAR_IN = 3.0            # a ball counts for the table point(s) within this many inches
STEP = 5                 # proposals are rounded to the D-pad trim step (t/s)
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
    shots, launched, ignored = [], 0, 0
    for name, path in sorted(newest_copies(root, 'events_teleop_*.csv', 14, since).items()):
        balls, results = {}, {}
        for r in csv.DictReader(open(path, newline='')):
            if r.get('event') == 'BALL':
                b = fields(r['detail'])
                balls[b.get('ball')] = b
            elif r.get('event') == 'SHOT_RESULT':
                s = fields(r['detail'])
                results[s.get('ball')] = s.get('result')
            elif r.get('event') == 'SHOT_RESULT_IGNORED':
                ignored += 1
        launched += len(balls)
        for k, res in results.items():
            b = balls.get(k, {})
            shots.append(dict(src=name[14:29], ball=k, result=res, floor=num(b.get('floor_in')), target=num(b.get('target')),
                              trim=num(b.get('trim')), bearing=num(b.get('bearing')), source=b.get('source'),
                              battery=num(b.get('battery')), ready=b.get('ready')))
    for name, path in sorted(newest_copies(root, 'shots_*.csv', 6, since).items()):
        for r in csv.DictReader(open(path, newline='')):
            if r.get('result') in ('IN', 'MISS'):
                shots.append(dict(src='shotlog ' + name[6:21], ball=r.get('shot'), result=r['result'],
                                  floor=num(r.get('floor_dist_in')), target=num(r.get('target_tps')), trim=None,
                                  bearing=num(r.get('bearing_deg')), source='SHOT_LOG', battery=num(r.get('battery_v')),
                                  ready='true'))
    return shots, launched, ignored


def propose(dist, speed, shots):
    rows = []
    for k, (d, v) in enumerate(zip(dist, speed)):
        near = [s for s in shots if s['floor'] is not None and s['target'] is not None and abs(s['floor'] - d) <= NEAR_IN]
        off = {lab: [s['target'] - table_speed(dist, speed, s['floor']) for s in near if s['result'] == lab]
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


def main(argv):
    args = [a for a in argv if not a.startswith('--')]
    root = args[0] if args else 'robot_logs'
    since = args[1] if len(args) > 1 else ''
    constants = next((a.split('=', 1)[1] for a in argv if a.startswith('--constants=')), CONSTANTS)
    dist, speed = read_table(constants)
    shots, launched, ignored = load_shots(root, since)
    print('%d balls launched in TeleOp logs, %d with a result (+ Shot Log), %d results without a ball' % (
        launched, len(shots), ignored))
    if not shots:
        print('No labelled balls yet: in TeleOp press LB + D-pad after a ball (UP in, DOWN short, RIGHT long, LEFT side).')
        return 0
    count = lambda lab: sum(1 for s in shots if s['result'] == lab)
    print('results: IN %d, SHORT %d, LONG %d, SIDE %d, MISS (Shot Log) %d; without a camera distance: %d' % (
        count('IN'), count('SHORT'), count('LONG'), count('SIDE'), count('MISS'), sum(1 for s in shots if s['floor'] is None)))
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

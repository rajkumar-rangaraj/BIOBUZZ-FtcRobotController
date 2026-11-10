// Printable driver guide: tune ONE shooting spot + speed, the driver rule, camera/AIM check, TeleOp button card.
// Same controls as BioBuzzShotLog / BioBuzzVisionTest / BioBuzzTeleOp - keep them in step.
//   node tools/make_shooting_guide.js [out.docx]     (needs the npm "docx" package in the working folder)
const fs = require('fs');
const path = require('path');
const {
  Document, Packer, Paragraph, TextRun, Table, TableRow, TableCell, AlignmentType, HeadingLevel, BorderStyle,
  WidthType, ShadingType, LevelFormat, Footer, PageNumber, PageBreak,
} = require('docx');

const OUT = process.argv[2] || path.join(__dirname, '..', 'BioBuzz_Shooting_Guide.docx');
const YELLOW = 'FFD000', GREEN = 'EAF5E4', RED = 'FDECEC', GREY = 'EDEDED';
const thin = { style: BorderStyle.SINGLE, size: 6, color: '808080' };
const borders = { top: thin, bottom: thin, left: thin, right: thin };
const W = 10080;                                   // portrait letter, 0.75 in margins

const runs = parts => (Array.isArray(parts) ? parts : [parts]).map(x => typeof x === 'string'
  ? new TextRun({ text: x, size: 22 }) : new TextRun({ size: 22, ...x }));
const p = (parts, o = {}) => new Paragraph({ spacing: { after: 80 }, ...o, children: runs(parts) });
const bullet = parts => new Paragraph({ numbering: { reference: 'b', level: 0 }, spacing: { after: 50 }, children: runs(parts) });
const num = (parts, ref) => new Paragraph({ numbering: { reference: ref, level: 0 }, spacing: { after: 60 }, children: runs(parts) });
const h1 = t => new Paragraph({ spacing: { before: 200, after: 80 }, shading: { fill: YELLOW, type: ShadingType.CLEAR },
  border: { left: { style: BorderStyle.SINGLE, size: 36, color: '000000', space: 8 } },
  children: [new TextRun({ text: t, bold: true, size: 30 })] });
const box = (label, text, fill, color) => new Paragraph({ spacing: { before: 80, after: 120 },
  shading: { fill, type: ShadingType.CLEAR }, border: { left: { style: BorderStyle.SINGLE, size: 24, color, space: 6 } },
  children: [new TextRun({ text: label + ':  ', bold: true, color, size: 22 }), ...runs(text)] });
const warn = t => box('CHECK', t, RED, 'C00000');
const ok = t => box('DONE WHEN', t, GREEN, '2E7D32');

function cell(text, w, o = {}) {
  return new TableCell({ borders, width: { size: w, type: WidthType.DXA }, verticalAlign: 'center',
    shading: o.fill ? { fill: o.fill, type: ShadingType.CLEAR } : undefined,
    margins: { top: 60, bottom: 60, left: 100, right: 100 },
    children: [new Paragraph({ alignment: o.center ? AlignmentType.CENTER : AlignmentType.LEFT,
      children: [new TextRun({ text, bold: !!o.bold, size: o.size || 21 })] })] });
}
function table(head, rows, widths, o = {}) {
  return new Table({ width: { size: widths.reduce((a, b) => a + b), type: WidthType.DXA }, columnWidths: widths, rows: [
    new TableRow({ tableHeader: true, children: head.map((t, i) => cell(t, widths[i], { bold: true, fill: o.headFill || GREY })) }),
    ...rows.map(r => new TableRow({ cantSplit: true, height: o.rowH ? { value: o.rowH, rule: 'atLeast' } : undefined,
      children: r.map((t, i) => cell(t, widths[i], { center: o.center && i > 0, size: o.size })) }))] });
}

const c = [
  new Paragraph({ spacing: { after: 40 }, children: [new TextRun({ text: 'BIOBUZZ 2026-27 \u2013 Team 37102', size: 20, color: '595959' })] }),
  new Paragraph({ heading: HeadingLevel.TITLE, spacing: { after: 100 }, children: [new TextRun({ text: 'Shooting Guide \u2013 one spot, one speed', bold: true, size: 40 })] }),
  p(['The robot\u2019s flywheel and windmill are now consistent (robot logs 30 Oct: every ball left at 1200\u20131260 t/s). Misses come from ',
    { text: 'distance and aim', bold: true }, ': one speed only lands from one distance. So we tune ', { text: 'one shooting spot', bold: true },
    ' and always shoot from it.']),

  h1('0.  Before you start'),
  bullet(['Practice HIVE with the ', { text: 'AprilTags on the bottom of the CELLs', bold: true }, ' (IDs 30\u201345, printed for practice from the FIRST field resources page). Without them the camera has nothing to see.']),
  bullet(['Pick the ', { text: 'alliance that matches the HIVE you shoot at', bold: true }, '. AUTO RED / AUTO BLUE sets it on its own; in TeleOp INIT press B / Circle = RED, A / Cross = BLUE. The robot remembers it (even after a restart). ',
    { text: 'Always read the \u201cAlliance\u201d line in INIT before PLAY', bold: true }, ' \u2013 the camera only looks for that alliance\u2019s CELLs.']),
  bullet(['Fresh battery (12.5 V or more), 4 balls, tape measure, coloured tape.']),

  h1('1.  Pick ONE shooting spot and tape it'),
  num(['Choose a spot in front of your alliance\u2019s HIVE that drivers can reach quickly and that is not in a lane the other alliance drives through.'], 's1'),
  num(['With the printed hood (Nov 2026): shoot ', { text: 'straight in front of the CELL, robot front about 2 in from the field wall', bold: true }, ' (the ball leaves about 3 ft from the CELL opening; with one flywheel wheel the camera picks about 1440\u20131490 t/s at 28\u201344 in). The camera sees the tags better when not too far.'], 's1'),
  num(['Put tape on the floor where the ', { text: 'front edge and both sides of the robot', bold: true }, ' go (an \u201cL\u201d at two corners works well), robot square to the HIVE.'], 's1'),
  num(['Write down where the spot is using things that are the same on every field (tile seams count from the walls, distance from the HIVE frame):'], 's1'),
  table(['', 'Our spot (fill in)'], [
    ['Alliance / HIVE side', ''], ['Tiles from our alliance wall (count seams)', ''], ['Tiles from the audience / rear wall', ''],
    ['Floor distance, camera lens \u2192 point under the CELL tags', '            in'], ['Robot heading (which wall is behind the launcher)', ''],
  ], [5200, 4880], { rowH: 420 }),

  new Paragraph({ children: [new PageBreak()] }),
  h1('2.  Find the speed: BioBuzz Shot Log'),
  p('Robot on the tape, square to the HIVE. Select BioBuzz Shot Log, press INIT, then PLAY. D-pad = the + shaped arrows on the left of the controller; in Shot Log it changes the speed being tested.'),
  table(['Button', 'What it does'], [
    ['Y / Triangle', 'Flywheel ON / OFF (starts at the NEAR preset, 1450 t/s)'],
    ['D-pad UP / DOWN', 'Speed +50 / \u221250 t/s (big steps)'],
    ['D-pad RIGHT / LEFT', 'Speed +10 / \u221210 t/s (fine steps)'],
    ['RB (right bumper)', 'Fire ONE ball (only when the flywheel is at speed)'],
    ['Then A / Cross', 'Ball went IN the CELL'],
    ['Then B / Circle', 'MISS'],
    ['Then Back / Share', 'Discard (no ball came out)'],
    ['Triggers', 'Intake in / out to reload'],
  ], [3000, 7080]),
  p(''),
  num(['Flywheel ON (Y). Fire 10 balls at ', { text: '1450', bold: true }, ', marking each one IN or MISS.'], 's2'),
  num(['Mostly ', { text: 'SHORT', bold: true }, ' (hits the front / falls before the CELL) \u2192 D-pad UP (+50). Mostly ', { text: 'LONG', bold: true }, ' (over the back) \u2192 D-pad DOWN (\u221250).'], 's2'),
  num(['When it is close, use RIGHT / LEFT (+/\u221210) and repeat 10 balls.'], 's2'),
  num(['The Driver Hub shows the hit rate for each speed. Keep the speed with ', { text: '8 or more of 10 IN', bold: true }, '.'], 's2'),
  warn('Do not move the robot between shots \u2013 re-check it is on the tape after reloading. Left/right misses are aim, not speed.'),
  table(['Speed (t/s)', 'IN', 'MISS short', 'MISS long', 'MISS left/right'], [
    ['1450', '', '', '', ''], ['', '', '', '', ''], ['', '', '', '', ''], ['', '', '', '', ''], ['', '', '', '', ''],
  ], [2400, 1920, 1920, 1920, 1920], { rowH: 400, center: true }),
  ok('One speed gives 8/10 or better from the tape. Final speed: ________ t/s'),

  h1('3.  Put the speed in the robot'),
  bullet(['Tell the programmer (or Copilot) the speed. In ', { text: 'BioBuzzConstants.java', bold: true }, ':']),
  bullet([{ text: 'AUTO_SPEED_DIST_IN / AUTO_SPEED_TPS', bold: true, font: 'Consolas' }, ' = the TeleOp auto speed table: put your speed at your spot\u2019s camera distance (the \u201cHIVE aim\u201d line shows it).']),
  bullet([{ text: 'LAUNCHER_NEAR_VELOCITY', bold: true, font: 'Consolas' }, ' = your speed (used when the camera can\u2019t see the CELL, and by D-pad DOWN = NEAR).']),
  bullet([{ text: 'AUTO_LAUNCH_VELOCITY', bold: true, font: 'Consolas' }, ' = the same, only if AUTO shoots from about the same distance.']),
  bullet('Rebuild, deploy, then fire 5 balls in TeleOp from the tape to confirm.'),

  new Paragraph({ children: [new PageBreak()] }),
  h1('3b.  Save the SHOT SPOT on the robot (Share button, no code change)'),
  num('Start BioBuzz TeleOp (check the alliance in INIT). Put the robot on the tape, launcher facing the CELL.', 's3'),
  num(['Check the \u201cShot speed\u201d line: it says AUTO and the camera picked the speed. Use RIGHT / LEFT until it shows your Shot Log speed.'], 's3'),
  num(['Hold X until the bearing settles, release, then press ', { text: 'Back / Share', bold: true }, '. The controller rumbles and the screen shows \u201cSHOT SPOT saved: 40 in @ 1465 t/s\u201d.'], 's3'),
  num('Fire 4 balls to confirm. Not good? Move, re-tune the speed and press Share again \u2013 the new save replaces the old one.', 's3'),
  box('RESULT', ['From now on, ', { text: 'holding X', bold: true }, ' turns the robot to the CELL and drives it forward/back to the saved distance; the camera sets the speed for that distance. At the spot the controller rumbles and the screen shows ',
    { text: 'ON SPOT - FIRE', bold: true }, '. The spot is a distance from the HIVE, so it works on both alliances and survives restarts. AUTO shoots at the saved speed.'], GREEN, '2E7D32'),
  warn('Share only saves when the camera sees your alliance\u2019s CELL tags (otherwise: \u201cCan\u2019t save the spot\u201d). Pushing the left stick forward/back while holding X takes over the driving.'),

  h1('3c.  Teach the AUTO speeds in practice (LB + D-pad)'),
  p('In BioBuzz TeleOp the robot logs every ball it launches: the camera distance, the speed, where the robot was. Tell it what each ball did, and the laptop works out the right speed for every distance (no more trimming by hand).'),
  num(['In INIT, check the ', { text: 'Diagnostics log says ON', bold: true }, ' (it is on by default; Back / Share toggles it).'], 's5'),
  num(['Shoot from different distances; the camera sets the speed (AUTO). After each ball press ', { text: 'LB + D-pad', bold: true },
    ': UP = IN, DOWN = SHORT (too slow), RIGHT = LONG (too fast), LEFT = SIDE (left / right / bounced out: aim, not speed).'], 's5'),
  num(['A buzz = saved. Two blips = no ball waiting (all labelled, or more than 10 s ago). A scout can call the results out; for a volley, label the balls in the order they flew. The \u201cBalls\u201d line shows the counts.'], 's5'),
  num(['After practice, on the laptop: pull the logs (', { text: 'tools\\pull_robot_logs.ps1', font: 'Consolas' }, '), then run ',
    { text: 'python tools/shot_results.py robot_logs <date>', font: 'Consolas' },
    '. It shows every table distance with its IN / SHORT / LONG balls and proposes new speeds (a distance moves only with 3 or more labelled balls near it). ',
    { text: '--write', font: 'Consolas' }, ' puts them into BioBuzzConstants.java; then rebuild and install.'], 's5'),
  warn('Many SIDE balls = aim, not speed: check the aim trim in BioBuzz Tune. Use a good battery (12.5 V or more): a weak battery makes balls short and teaches the wrong speeds.'),
  ok('At every distance you shoot from, most labels are IN and the tool proposes no more changes.'),

  new Paragraph({ children: [new PageBreak()] }),
  h1('4.  Camera and AIM check: BioBuzz Vision Test'),
  num('Robot on the tape. Select BioBuzz Vision Test, INIT, PLAY. B / Circle switches alliance \u2013 pick the HIVE you face.', 's4'),
  num('The screen should show your CELL with 2\u20134 tags and a bearing near 0. Hold X / Square: the robot turns until the bearing settles.', 's4'),
  num('In TeleOp the line \u201cHIVE aim\u201d now always shows what the camera sees (cell, tags, bearing, range) \u2013 even when you are not pressing X.', 's4'),
  warn('\u201cno RED CELL tags in view (any tags: 3)\u201d means tags are seen but they belong to the other alliance\u2019s CELL \u2013 fix the alliance in INIT. \u201cany tags: 0\u201d means the camera sees no tags at all: check the practice tags, the camera tilt (30\u00b0 up) and that nothing blocks the view.'),
  ok('Your CELL shows with 2+ tags from the tape, and holding X turns the robot onto it.'),

  h1('5.  DRIVER RULE (match play)'),
  new Table({ width: { size: W, type: WidthType.DXA }, columnWidths: [W], rows: [new TableRow({ children: [new TableCell({ borders,
    shading: { fill: 'FFF4CC', type: ShadingType.CLEAR }, margins: { top: 160, bottom: 160, left: 240, right: 240 }, children: [
      ...[['1', 'DRIVE NEAR THE SPOT', ' \u2013 launcher toward our CELL, press Y to pre-spin.'], ['2', 'HOLD X', ' \u2013 the robot turns and drives to the saved distance.'],
        ['3', 'WAIT FOR THE RUMBLE', ' \u2013 screen shows ON SPOT - FIRE.'], ['4', 'KEEP HOLDING X + HOLD RB', ' \u2013 one ball every ~0.5 s.'],
        ['5', 'RELEASE', ' RB when empty, then drive away.']].map(([n, b, t]) =>
        new Paragraph({ spacing: { after: 100 }, children: [new TextRun({ text: n + '.  ', bold: true, size: 30 }),
          new TextRun({ text: b, bold: true, size: 30 }), new TextRun({ text: t, size: 26 })] })),
      new Paragraph({ spacing: { before: 120 }, children: [new TextRun({ text: 'No tags seen (no rumble)? Drive onto the taped spot by eye, square up, then RB. Do NOT shoot while moving.', bold: true, size: 26, color: 'C00000' })] }),
    ] })] })] }),
  p(''),
  p([{ text: 'Tip: ', bold: true }, 'press Y (pre-spin) while driving to the spot, so the flywheel is already at speed when you stop.']),

  new Paragraph({ children: [new PageBreak()] }),
  h1('TeleOp button card'),
  p([{ text: 'D-pad = the + shaped arrow buttons on the LEFT side of the controller.', bold: true },
    ' In TeleOp it changes the SHOT SPEED (how hard the ball is thrown); with LB held it saves what a ball did, for the log. It does not drive or aim. Normally you don\u2019t need UP / DOWN: the camera sets the speed from the distance (AUTO).']),
  table(['D-pad (TeleOp)', 'What it does', 'Use it when'], [
    ['UP', 'MANUAL FAR: 1570 t/s, trim reset to 0. The camera speed is off until you press X', 'camera can\u2019t see our CELL, far away'],
    ['DOWN', 'MANUAL NEAR: 1450 t/s (or the speed you tuned), trim reset to 0. The camera speed is off until you press X', 'camera can\u2019t see our CELL, near'],
    ['RIGHT', '+5 t/s (each press), stays on top of AUTO', 'balls land SHORT (in front of / below the CELL)'],
    ['LEFT', '\u22125 t/s (each press), stays on top of AUTO', 'balls go LONG (over / behind the CELL)'],
    ['LB + UP / DOWN / RIGHT / LEFT', 'What the ball did, saved in the diagnostics log: UP = IN, DOWN = SHORT, RIGHT = LONG, LEFT = SIDE (left / right / bounced out). Goes to the oldest ball of the last 10 s without a result: label a volley in order', 'practice: after every ball (a scout can do it). tools/shot_results.py turns the labels into new AUTO speeds'],
  ], [1700, 4580, 3800]),
  p([{ text: 'The Driver Hub line \u201cShot speed\u201d always shows it, e.g. ', bold: false },
    { text: 'AUTO 41 in 1465 +5 = 1470 t/s', font: 'Consolas', bold: true },
    '. AUTO keeps the last speed for 3 s when the CELL drops out of view, then uses the last preset (NEAR at the start), and it never changes while RB is held. Pressing UP or DOWN clears the +/\u2212 trim and switches to MANUAL; X switches back to AUTO.']),
  warn('D-pad in other programs is different: BioBuzz Shot Log = speed \u00b150 (UP/DOWN) and \u00b110 (RIGHT/LEFT); BioBuzz Calibration and Arm Setup = arm position; AUTO INIT = start delay.'),
  p(''),
  table(['Button', 'Action'], [
    ['Left stick / right stick X', 'Drive + strafe / turn. FIELD-CENTRIC by default: stick up = away from you, whichever way the robot faces (L1 in INIT = robot-centric: up = intake side). While X aims at a CELL the stick is robot-centric'],
    ['Options', 'Field-centric reset: point the intake away from you and press (after a big hit; the start comes from AUTO, or the robot is assumed to have its intake pointing away at INIT)'],
    ['LB (hold)', 'Precision (slow) drive'],
    ['RT / LT', 'Intake IN / OUT (half LT = flower back-out)'],
    ['RB (hold)', 'FIRE: spins up and feeds one ball at a time, when the flywheel is steady and (CELL in view) the aim is good. FIRE HELD on the screen says why; the ball goes anyway after 1.5 s'],
    ['X / Square (hold)', 'HIVE aim + drive to the saved SHOT SPOT distance; rumbles ON SPOT. Also switches MANUAL speed back to AUTO (camera)'],
    ['Back / Share', 'Save the SHOT SPOT here (camera distance + current shot speed)'],
    ['Y / Triangle', 'Flywheel pre-spin ON / OFF'],
    ['D-pad UP / DOWN', 'MANUAL shot speed FAR (1570) / NEAR (1450) \u2013 see table above'],
    ['D-pad RIGHT / LEFT', 'Shot speed +5 / \u22125 per press'],
    ['LB + D-pad', 'Shot result for the log: UP IN, DOWN SHORT, RIGHT LONG, LEFT SIDE (see the D-pad table)'],
    ['A / Cross', 'Flower arm DOWN / UP (only when stopped)'],
    ['B / Circle (hold)', 'Unjam: windmill slowly backwards'],
    ['INIT: B / A, Back, X, Y, L1, Options', 'Alliance RED / BLUE (remembered; AUTO sets it); diagnostics log on/off; aim gate on/off (off = FIRE never waits for the camera); auto shot speed on/off (off = NEAR / FAR only); field-centric on/off (ON by default); Options = the intake points away from you now'],
  ], [3600, 6480]),
];

const doc = new Document({
  creator: 'BIOBUZZ', title: 'BIOBUZZ Shooting Guide',
  styles: { default: { document: { run: { font: 'Arial', size: 22 } } } },
  numbering: { config: [{ reference: 'b', levels: [{ level: 0, format: LevelFormat.BULLET, text: '\u2022', alignment: AlignmentType.LEFT,
      style: { paragraph: { indent: { left: 400, hanging: 260 } } } }] },
    ...['s1', 's2', 's3', 's4', 's5'].map(ref => ({ reference: ref, levels: [{ level: 0, format: LevelFormat.DECIMAL, text: '%1.', alignment: AlignmentType.LEFT,
      style: { paragraph: { indent: { left: 460, hanging: 300 } } } }] }))] },
  sections: [{
    properties: { page: { size: { width: 12240, height: 15840 }, margin: { top: 1000, right: 1080, bottom: 1000, left: 1080 } } },
    footers: { default: new Footer({ children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [
      new TextRun({ text: 'BIOBUZZ shooting guide \u2013 page ', size: 16, color: '808080' }),
      new TextRun({ children: [PageNumber.CURRENT], size: 16, color: '808080' })] })] }) },
    children: c,
  }],
});
Packer.toBuffer(doc).then(b => { fs.writeFileSync(OUT, b); console.log('wrote', OUT, b.length); });

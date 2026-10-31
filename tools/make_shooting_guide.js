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
  bullet(['In TeleOp INIT choose the ', { text: 'alliance that matches the HIVE you shoot at', bold: true }, ': B / Circle = RED, A / Cross = BLUE. The camera only looks for that alliance\u2019s CELLs.']),
  bullet(['Fresh battery (12.5 V or more), 4 balls, tape measure, coloured tape.']),

  h1('1.  Pick ONE shooting spot and tape it'),
  num(['Choose a spot in front of your alliance\u2019s HIVE that drivers can reach quickly and that is not in a lane the other alliance drives through.'], 's1'),
  num(['Pick a comfortable distance \u2013 start around ', { text: '4\u20136 ft', bold: true }, ' from the floor under the CELL. The camera sees the tags better when not too far.'], 's1'),
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
    ['Y / Triangle', 'Flywheel ON / OFF (starts at 1250 t/s)'],
    ['D-pad UP / DOWN', 'Speed +50 / \u221250 t/s (big steps)'],
    ['D-pad RIGHT / LEFT', 'Speed +10 / \u221210 t/s (fine steps)'],
    ['RB (right bumper)', 'Fire ONE ball (only when the flywheel is at speed)'],
    ['Then A / Cross', 'Ball went IN the CELL'],
    ['Then B / Circle', 'MISS'],
    ['Then Back / Share', 'Discard (no ball came out)'],
    ['Triggers', 'Intake in / out to reload'],
  ], [3000, 7080]),
  p(''),
  num(['Flywheel ON (Y). Fire 10 balls at ', { text: '1250', bold: true }, ', marking each one IN or MISS.'], 's2'),
  num(['Mostly ', { text: 'SHORT', bold: true }, ' (hits the front / falls before the CELL) \u2192 D-pad UP (+50). Mostly ', { text: 'LONG', bold: true }, ' (over the back) \u2192 D-pad DOWN (\u221250).'], 's2'),
  num(['When it is close, use RIGHT / LEFT (+/\u221210) and repeat 10 balls.'], 's2'),
  num(['The Driver Hub shows the hit rate for each speed. Keep the speed with ', { text: '8 or more of 10 IN', bold: true }, '.'], 's2'),
  warn('Do not move the robot between shots \u2013 re-check it is on the tape after reloading. Left/right misses are aim, not speed.'),
  table(['Speed (t/s)', 'IN', 'MISS short', 'MISS long', 'MISS left/right'], [
    ['1250', '', '', '', ''], ['', '', '', '', ''], ['', '', '', '', ''], ['', '', '', '', ''], ['', '', '', '', ''],
  ], [2400, 1920, 1920, 1920, 1920], { rowH: 400, center: true }),
  ok('One speed gives 8/10 or better from the tape. Final speed: ________ t/s'),

  h1('3.  Put the speed in the robot'),
  bullet(['Tell the programmer (or Copilot) the speed. In ', { text: 'BioBuzzConstants.java', bold: true }, ':']),
  bullet([{ text: 'LAUNCHER_NEAR_VELOCITY', bold: true, font: 'Consolas' }, ' = your speed (TeleOp default, D-pad DOWN = NEAR).']),
  bullet([{ text: 'AUTO_LAUNCH_VELOCITY', bold: true, font: 'Consolas' }, ' = the same, only if AUTO shoots from about the same distance.']),
  bullet('Rebuild, deploy, then fire 5 balls in TeleOp from the tape to confirm.'),

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
      ...[['1', 'DRIVE TO THE SPOT', ' \u2013 same place every time.'], ['2', 'SQUARE UP', ' to the HIVE, launcher facing the CELL.'],
        ['3', 'STOP', ' \u2013 sticks released, robot still.'], ['4', 'HOLD X', ' (aim) until the bearing settles \u2013 if tags are seen.'],
        ['5', 'HOLD RB', ' \u2013 one ball every ~0.5 s while held; release to stop.']].map(([n, b, t]) =>
        new Paragraph({ spacing: { after: 100 }, children: [new TextRun({ text: n + '.  ', bold: true, size: 30 }),
          new TextRun({ text: b, bold: true, size: 30 }), new TextRun({ text: t, size: 26 })] })),
      new Paragraph({ spacing: { before: 120 }, children: [new TextRun({ text: 'Do NOT shoot from anywhere else. Do NOT shoot while moving.', bold: true, size: 26, color: 'C00000' })] }),
    ] })] })] }),
  p(''),
  p([{ text: 'Tip: ', bold: true }, 'press Y (pre-spin) while driving to the spot, so the flywheel is already at speed when you stop.']),

  new Paragraph({ children: [new PageBreak()] }),
  h1('TeleOp button card'),
  p([{ text: 'D-pad = the + shaped arrow buttons on the LEFT side of the controller.', bold: true },
    ' In TeleOp it only changes the SHOT SPEED (how hard the ball is thrown). It does not drive or aim.']),
  table(['D-pad (TeleOp)', 'What it does', 'Use it when'], [
    ['UP', 'FAR preset: 1450 t/s, trim reset to 0', 'shooting from far away'],
    ['DOWN', 'NEAR preset: 1250 t/s (or the speed you tuned), trim reset to 0', 'shooting from our taped spot \u2013 normal choice'],
    ['RIGHT', '+25 t/s (each press)', 'balls land SHORT (in front of / below the CELL)'],
    ['LEFT', '\u221225 t/s (each press)', 'balls go LONG (over / behind the CELL)'],
  ], [1700, 4580, 3800]),
  p([{ text: 'The Driver Hub line \u201cShot speed\u201d always shows it, e.g. ', bold: false },
    { text: 'NEAR 1250 +25 = 1275 t/s', font: 'Consolas', bold: true },
    '. Pressing UP or DOWN clears the +/\u2212 trim. The speed stays until you change it; it is used the next time you press RB or Y.']),
  warn('D-pad in other programs is different: BioBuzz Shot Log = speed \u00b150 (UP/DOWN) and \u00b110 (RIGHT/LEFT); BioBuzz Calibration and Arm Setup = arm position; AUTO INIT = start delay.'),
  p(''),
  table(['Button', 'Action'], [
    ['Left stick / right stick X', 'Drive + strafe / turn  (intake side = FRONT)'],
    ['LB (hold)', 'Precision (slow) drive'],
    ['RT / LT', 'Intake IN / OUT (half LT = flower back-out)'],
    ['RB (hold)', 'FIRE: spins up and feeds one ball at a time'],
    ['X / Square (hold)', 'HIVE auto-aim (camera turns the robot)'],
    ['Y / Triangle', 'Flywheel pre-spin ON / OFF'],
    ['D-pad UP / DOWN', 'Shot speed FAR (1450) / NEAR (1250) \u2013 see table above'],
    ['D-pad RIGHT / LEFT', 'Shot speed +25 / \u221225 per press'],
    ['A / Cross', 'Flower arm DOWN / UP (only when stopped)'],
    ['B / Circle (hold)', 'Unjam: windmill slowly backwards'],
    ['INIT: B / A, Back', 'Alliance RED / BLUE; diagnostics log on/off'],
  ], [3600, 6480]),
];

const doc = new Document({
  creator: 'BIOBUZZ', title: 'BIOBUZZ Shooting Guide',
  styles: { default: { document: { run: { font: 'Arial', size: 22 } } } },
  numbering: { config: [{ reference: 'b', levels: [{ level: 0, format: LevelFormat.BULLET, text: '\u2022', alignment: AlignmentType.LEFT,
      style: { paragraph: { indent: { left: 400, hanging: 260 } } } }] },
    ...['s1', 's2', 's4'].map(ref => ({ reference: ref, levels: [{ level: 0, format: LevelFormat.DECIMAL, text: '%1.', alignment: AlignmentType.LEFT,
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

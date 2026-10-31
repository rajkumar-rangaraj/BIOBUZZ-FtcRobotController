// Printable wiring sheet for the BIOBUZZ robot: which cable goes to which hub port + cut-out cable tags.
// Same content as the "Robot wiring and configuration" section of README_2.md - keep the two in step.
//   node tools/make_wiring_sheet.js [out.docx]     (needs the npm "docx" package in the working folder)
const fs = require('fs');
const path = require('path');
const {
  Document, Packer, Paragraph, TextRun, Table, TableRow, TableCell, AlignmentType, HeadingLevel, BorderStyle,
  WidthType, ShadingType, PageOrientation, LevelFormat, Footer, PageNumber, PageBreak,
} = require('docx');

const OUT = process.argv[2] || path.join(__dirname, '..', 'BioBuzz_Wiring_Sheet.docx');
const YELLOW = 'FFD000', CH = 'DCEBFA', EH = 'FCE8D4';
const thin = { style: BorderStyle.SINGLE, size: 6, color: '808080' };
const borders = { top: thin, bottom: thin, left: thin, right: thin };
const dashed = { style: BorderStyle.DASHED, size: 6, color: '808080' };
const dash4 = { top: dashed, bottom: dashed, left: dashed, right: dashed };

const run = (t, o = {}) => new TextRun({ text: t, size: 20, ...o });
const p = (t, o = {}) => new Paragraph({ spacing: { after: 40 }, children: [run(t, o)] });
const rich = parts => new Paragraph({ spacing: { after: 40 }, children: parts.map(x => typeof x === 'string' ? run(x) : run(x.t, x)) });
const bullet = parts => new Paragraph({ numbering: { reference: 'b', level: 0 }, spacing: { after: 30 },
  children: (Array.isArray(parts) ? parts : [parts]).map(x => typeof x === 'string' ? run(x) : run(x.t, x)) });
const h = (t, color) => new Paragraph({ heading: HeadingLevel.HEADING_1, spacing: { before: 100, after: 60 },
  shading: color ? { fill: color, type: ShadingType.CLEAR } : undefined, children: [new TextRun({ text: t, bold: true, size: 26 })] });

function cell(text, w, o = {}) {
  return new TableCell({ borders, width: { size: w, type: WidthType.DXA }, verticalAlign: 'center',
    shading: o.fill ? { fill: o.fill, type: ShadingType.CLEAR } : undefined,
    margins: { top: 35, bottom: 35, left: 90, right: 90 },
    children: [new Paragraph({ alignment: o.center ? AlignmentType.CENTER : AlignmentType.LEFT,
      children: [new TextRun({ text, bold: !!o.bold, size: o.size || 21, font: o.mono ? 'Consolas' : undefined })] })] });
}
const W = [1500, 4300, 1600, 2300, 2400, 900];       // landscape letter: 13 680 twips between margins
function portTable(rows, fill) {
  const head = ['Port', 'Device', 'Encoder cable', 'Config type', 'Name (exact)', 'Done'];
  return new Table({ width: { size: W.reduce((a, b) => a + b), type: WidthType.DXA }, columnWidths: W, rows: [
    new TableRow({ tableHeader: true, children: head.map((t, i) => cell(t, W[i], { bold: true, fill })) }),
    ...rows.map(r => new TableRow({ cantSplit: true, children: [
      cell(r[0], W[0], { bold: true }), cell(r[1], W[1]), cell(r[2], W[2], { center: true }), cell(r[3], W[3], { size: 20 }),
      cell(r[4], W[4], { mono: true, bold: true }), cell('\u2610', W[5], { center: true, size: 30 })] }))] });
}

const CH_ROWS = [
  ['Motor 0', 'Front-left drive wheel', 'Encoder 0', 'goBILDA 5202/3/4', 'left_front_drive'],
  ['Motor 1', 'Back-left drive wheel', 'Encoder 1', 'goBILDA 5202/3/4', 'left_back_drive'],
  ['Motor 2', 'Launcher / flywheel motor (middle, 1:1)', 'Encoder 2', 'goBILDA 5202/3/4', 'launcher'],
  ['Motor 3', 'SPARE \u2013 leave empty', '\u2013', '\u2013', '\u2013'],
  ['Servo 0', 'Left corner intake wheel servo', '\u2013', 'Continuous Rotation Servo', 'left_intake_servo'],
  ['USB 2.0', 'Logitech C270 webcam', '\u2013', 'Webcam', 'Webcam 1'],
  ['(built in)', 'IMU inside the Control Hub \u2013 nothing to plug', '\u2013', 'REV internal IMU', 'imu'],
  ['I2C 1', 'goBILDA Pinpoint (OPTIONAL, AUTO only) \u2013 pods into its X / Y ports', '\u2013', 'goBILDA Pinpoint Odometry Computer', 'pinpoint'],
];
const EH_ROWS = [
  ['Motor 0', 'Front-right drive wheel', 'Encoder 0', 'goBILDA 5202/3/4', 'right_front_drive'],
  ['Motor 1', 'Back-right drive wheel', 'Encoder 1', 'goBILDA 5202/3/4', 'right_back_drive'],
  ['Motor 2', 'Intake roller motor', 'NOT needed', 'goBILDA 5202/3/4', 'intake'],
  ['Motor 3', 'Windmill / indexer motor (435 RPM)', 'Encoder 3', 'goBILDA 5202/3/4', 'indexer'],
  ['Servo 0', 'Right corner intake wheel servo', '\u2013', 'Continuous Rotation Servo', 'right_intake_servo'],
  ['Servo 1', 'Flower arm servo (SERVO mode, not continuous)', '\u2013', 'Servo', 'flower_arm'],
];
// cut-out tags: [hub, port, name]
const TAGS = [...CH_ROWS.filter(r => r[4] !== '\u2013' && r[0] !== '(built in)').map(r => ['CONTROL HUB', r[0], r[4], CH]),
              ...EH_ROWS.map(r => ['EXPANSION HUB', r[0], r[4], EH])];
for (const r of [...CH_ROWS, ...EH_ROWS])
  if (r[2].startsWith('Encoder')) TAGS.push([(CH_ROWS.includes(r) ? 'CONTROL HUB' : 'EXPANSION HUB'), r[2], r[4] + ' (encoder)', CH_ROWS.includes(r) ? CH : EH]);
TAGS.push(['BOTH HUBS', 'RS485', 'Control Hub \u2194 Expansion Hub', 'EEEEEE'], ['BOTH HUBS', 'XT30', 'Expansion Hub power (from Control Hub)', 'EEEEEE']);

function tagGrid() {
  const cols = 5, w = 2736, rows = [];
  for (let i = 0; i < TAGS.length; i += cols) {
    const cells = TAGS.slice(i, i + cols).map(([hub, port, name, fill]) => new TableCell({ borders: dash4, width: { size: w, type: WidthType.DXA },
      shading: { fill, type: ShadingType.CLEAR }, margins: { top: 30, bottom: 30, left: 60, right: 60 }, children: [
        new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: hub, size: 16, bold: true, color: '404040' })] }),
        new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: port, size: 24, bold: true })] }),
        new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: name, size: 16, font: 'Consolas' })] })] }));
    while (cells.length < cols) cells.push(new TableCell({ borders: dash4, width: { size: w, type: WidthType.DXA }, children: [new Paragraph('')] }));
    rows.push(new TableRow({ cantSplit: true, children: cells }));
  }
  return new Table({ width: { size: w * cols, type: WidthType.DXA }, columnWidths: Array(cols).fill(w), rows });
}

const c = [
  new Paragraph({ spacing: { after: 40 }, children: [new TextRun({ text: 'BIOBUZZ 2026-27 \u2013 goBILDA StarterBot', size: 20, color: '595959' })] }),
  new Paragraph({ heading: HeadingLevel.TITLE, spacing: { after: 80 }, children: [new TextRun({ text: 'Wiring Sheet \u2013 which cable goes where', bold: true, size: 40 })] }),
  rich([{ t: 'Left / right / front / back', bold: true }, ' are seen from BEHIND the robot, looking toward the intake. FRONT = intake + flower arm. BACK = launcher + camera.']),
  rich([{ t: 'Simple rule: ', bold: true }, 'everything on the robot\u2019s LEFT goes to the ', { t: 'CONTROL HUB', bold: true }, ', everything on the RIGHT goes to the ', { t: 'EXPANSION HUB', bold: true }, '. The launcher (middle) goes to the Control Hub.']),
  h('CONTROL HUB (left side)', CH),
  portTable(CH_ROWS, CH),
  h('EXPANSION HUB (right side)', EH),
  portTable(EH_ROWS, EH),
  new Paragraph({ children: [new PageBreak()] }),
  h('Power and hub link', YELLOW),
  bullet(['Battery \u2192 main power switch \u2192 Control Hub ', { t: 'XT30 power', bold: true }, ' input.']),
  bullet(['Control Hub\u2019s second XT30 \u2192 Expansion Hub XT30 (the Expansion Hub has no battery of its own).']),
  bullet(['Control Hub ', { t: 'RS485', bold: true }, ' \u2192 Expansion Hub RS485 (either RS485 port on each hub).']),
  h('Rules (check every cable)', YELLOW),
  bullet([{ t: 'Encoder number = motor number. ', bold: true }, 'Motor in port 2 \u2192 its encoder cable in encoder port 2 on the SAME hub. Swapped encoder cables make the robot drive crazy.']),
  bullet(['Encoders needed: all 4 drive motors, the launcher and the windmill/indexer. The intake motor needs NO encoder cable.']),
  bullet([{ t: 'Servo plugs: ', bold: true }, 'black (ground) wire on the pin marked \u2013 / ground on the hub. A plug turned around does nothing.']),
  bullet(['The flower arm servo must be in SERVO (position) mode (goBILDA servo programmer) before it is installed.']),
  bullet(['Webcam goes in the Control Hub\u2019s USB 2.0 port (the black one), not the blue USB 3.0 port.']),
  bullet(['Keep cables away from the wheels, the intake rollers, the flywheel, the windmill and the flower arm\u2019s swing. Zip-tie them; leave a little slack at every moving part.']),
  bullet(['Power the robot OFF before plugging or unplugging anything.']),
  h('When you are done', YELLOW),
  bullet(['Driver Hub: robot configuration \u2192 check every name above is typed EXACTLY (lower case, underscores). A typo stops the program at INIT.']),
  bullet(['Robot on a box, wheels free: run ', { t: 'BioBuzz Wiring Check', bold: true }, '. It must say WIRING OK; every FAIL line says which cable to fix.']),
  bullet(['Then run ', { t: 'BioBuzz System Check', bold: true }, '.']),
  h('Cable tags \u2013 cut out and tape/zip-tie one to each end of the cable', YELLOW),
  tagGrid(),
];

const doc = new Document({
  creator: 'BIOBUZZ', title: 'BIOBUZZ Wiring Sheet',
  styles: { default: { document: { run: { font: 'Arial', size: 22 } } } },
  numbering: { config: [{ reference: 'b', levels: [{ level: 0, format: LevelFormat.BULLET, text: '\u2022', alignment: AlignmentType.LEFT,
    style: { paragraph: { indent: { left: 400, hanging: 260 } } } }] }] },
  sections: [{
    properties: { page: { size: { width: 12240, height: 15840, orientation: PageOrientation.LANDSCAPE },
      margin: { top: 720, right: 720, bottom: 720, left: 720 } } },
    footers: { default: new Footer({ children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [
      new TextRun({ text: 'BIOBUZZ wiring sheet \u2013 same as README_2.md \u2013 page ', size: 16, color: '808080' }),
      new TextRun({ children: [PageNumber.CURRENT], size: 16, color: '808080' })] })] }) },
    children: c,
  }],
});
Packer.toBuffer(doc).then(b => { fs.writeFileSync(OUT, b); console.log('wrote', OUT, b.length); });

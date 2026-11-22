package org.firstinspires.ftc.teamcode.v2;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.io.FileOutputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

/** v2 calibration file: defaults, round trip, bad files, validation, safe save; v2 config names. */
public class V2CalibrationTest {
    @Rule
    public TemporaryFolder tmp = new TemporaryFolder();

    private static V2Calibration full() {
        V2Calibration c = new V2Calibration();
        c.setMotorReversed(V2Config.LEFT_FRONT, true);
        c.setMotorReversed(V2Config.LAUNCHER2, true);
        c.setCrServoReversed(V2Config.RIGHT_INTAKE_SERVO, true);
        c.setDirectionsChecked(true);
        c.setGate(0.215, 0.735);
        c.setArm(0.10, 0.62);
        c.setBeams(true, false);
        c.setQueueTicksPerBall(338);
        c.setIntakeJamAmps(3.6);
        return c;
    }

    @Test
    public void defaultsNeedEverythingMeasured() {
        V2Calibration c = new V2Calibration();
        assertFalse(c.gateCalibrated());
        assertFalse(c.armCalibrated());
        assertEquals(V2Calibration.DEFAULT_TICKS_PER_BALL, c.queueTicksPerBall());
        assertEquals(V2Calibration.DEFAULT_JAM_AMPS, c.intakeJamAmps(), 1e-9);
        assertTrue(c.beamStagedBrokenWhenLow());
        for (String m : V2Config.MOTORS) {
            assertFalse(c.motorReversed(m));
        }
        assertEquals(6, c.missing().size());
        assertFalse(c.fromFile());
    }

    @Test
    public void formatParseRoundTrip() {
        V2Calibration c = V2Calibration.parse(full().format("21 Nov 2026 23:10"));
        assertTrue(c.warnings().toString(), c.warnings().isEmpty());
        assertTrue(c.motorReversed(V2Config.LEFT_FRONT));
        assertTrue(c.motorReversed(V2Config.LAUNCHER2));
        assertFalse(c.motorReversed(V2Config.LAUNCHER));
        assertTrue(c.crServoReversed(V2Config.RIGHT_INTAKE_SERVO));
        assertFalse(c.crServoReversed(V2Config.LEFT_INTAKE_SERVO));
        assertEquals(0.215, c.gateClosed(), 1e-4);
        assertEquals(0.735, c.gateOpen(), 1e-4);
        assertEquals(0.10, c.armStowed(), 1e-4);
        assertEquals(0.62, c.armDeployed(), 1e-4);
        assertTrue(c.beamStagedBrokenWhenLow());
        assertFalse(c.beamFullBrokenWhenLow());
        assertEquals(338, c.queueTicksPerBall());
        assertEquals(3.6, c.intakeJamAmps(), 1e-6);
        assertEquals("21 Nov 2026 23:10", c.savedAt());
        assertTrue(c.missing().toString(), c.missing().isEmpty());
    }

    @Test
    public void emptyOrGarbageGivesDefaultsNeverThrows() {
        for (String text : new String[]{null, "", "\n\n", "garbage", "=nokey", "#just a comment", "\u0000\u0001"}) {
            V2Calibration c = V2Calibration.parse(text);
            assertNotNull(c);
            assertFalse(c.gateCalibrated());
            assertEquals(V2Calibration.DEFAULT_TICKS_PER_BALL, c.queueTicksPerBall());
        }
    }

    @Test
    public void badValuesKeepDefaultsAndWarn() {
        V2Calibration c = V2Calibration.parse(String.join("\n",
                "dir.launcher2=SIDEWAYS",
                "gate.closed=abc",
                "gate.open=0.7",
                "arm.stowed=0.40",
                "arm.deployed=0.42",          // too little travel
                "queue.ticksPerBall=12",      // out of range
                "intake.jamAmps=NaN",
                "beams.checked=maybe"));
        assertFalse(c.motorReversed(V2Config.LAUNCHER2));
        assertFalse(c.gateCalibrated());
        assertFalse(c.armCalibrated());
        assertEquals(V2Calibration.DEFAULT_TICKS_PER_BALL, c.queueTicksPerBall());
        assertFalse(c.queueTicksMeasured());
        assertEquals(V2Calibration.DEFAULT_JAM_AMPS, c.intakeJamAmps(), 1e-9);
        assertFalse(c.beamsChecked());
        assertTrue(c.warnings().toString(), c.warnings().size() >= 7);
    }

    @Test
    public void halfAGateIsNotAGate() {
        V2Calibration c = V2Calibration.parse("gate.closed=0.3\nend=ok\n");
        assertFalse(c.gateCalibrated());
        assertEquals(c.warnings().toString(), 1, c.warnings().size());
    }

    @Test
    public void unknownKeysAndOtherVersionsStillLoad() {
        V2Calibration c = V2Calibration.parse("version=2\nfuture.thing=7\nqueue.ticksPerBall=400\nend=ok\n");
        assertEquals(400, c.queueTicksPerBall());
        assertEquals(c.warnings().toString(), 1, c.warnings().size());   // only the version note
    }

    @Test
    public void partialFileKeepsTheRest() {
        V2Calibration c = V2Calibration.parse("queue.ticksPerBall=355\n");
        assertEquals(355, c.queueTicksPerBall());
        assertFalse(c.gateCalibrated());
        assertTrue(c.missing().contains("gate CLOSED / OPEN"));
        assertFalse(c.missing().contains("queue ticks per ball"));
    }

    @Test
    public void settersRejectBadValues() {
        V2Calibration c = new V2Calibration();
        expectBad(() -> c.setGate(0.5, 0.52));
        expectBad(() -> c.setGate(-0.1, 0.5));
        expectBad(() -> c.setGate(0.5, Double.NaN));
        expectBad(() -> c.setArm(0.2, 1.2));
        expectBad(() -> c.setQueueTicksPerBall(10));
        expectBad(() -> c.setQueueTicksPerBall(9000));
        expectBad(() -> c.setIntakeJamAmps(0.1));
        expectBad(() -> c.setIntakeJamAmps(Double.NaN));
        expectBad(() -> c.setMotorReversed("nope", true));
        expectBad(() -> c.setCrServoReversed("gate", true));
        assertFalse(c.gateCalibrated());               // nothing half-applied
        assertFalse(c.queueTicksMeasured());
    }

    private static void expectBad(Runnable r) {
        try {
            r.run();
            fail("expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            // ok
        }
    }

    @Test
    public void saveAndLoadFile() throws Exception {
        File f = new File(tmp.getRoot(), "sub/" + V2Calibration.FILE_NAME);
        full().save(f, "stamp");
        assertTrue(f.exists());
        assertFalse(new File(f.getParentFile(), f.getName() + ".tmp").exists());
        V2Calibration c = V2Calibration.load(f);
        assertTrue(c.fromFile());
        assertTrue(c.missing().isEmpty());
        assertEquals(338, c.queueTicksPerBall());
        V2Calibration c2 = V2Calibration.load(f);       // saving again replaces the old file
        c2.setQueueTicksPerBall(350);
        c2.save(f, "stamp2");
        assertEquals(350, V2Calibration.load(f).queueTicksPerBall());
    }

    @Test
    public void missingFileGivesDefaults() {
        V2Calibration c = V2Calibration.load(new File(tmp.getRoot(), "none.txt"));
        assertFalse(c.fromFile());
        assertTrue(c.warnings().isEmpty());
        assertEquals(6, c.missing().size());
        assertFalse(V2Calibration.load(null).fromFile());
    }

    @Test
    public void interruptedSaveReadsTheTempFile() throws Exception {
        File f = new File(tmp.getRoot(), V2Calibration.FILE_NAME);
        File t = new File(tmp.getRoot(), V2Calibration.FILE_NAME + ".tmp");
        try (FileOutputStream o = new FileOutputStream(t)) {
            o.write(full().format("x").getBytes(StandardCharsets.UTF_8));
        }
        V2Calibration c = V2Calibration.load(f);
        assertEquals(338, c.queueTicksPerBall());
        assertTrue(c.warnings().get(0).contains("main file missing"));
    }

    @Test
    public void unreadableFileGivesDefaultsWithWarning() throws Exception {
        File dirNotFile = tmp.newFolder(V2Calibration.FILE_NAME);   // a folder where the file should be
        V2Calibration c = V2Calibration.load(dirNotFile);
        assertFalse(c.gateCalibrated());
        assertEquals(1, c.warnings().size());
    }

    @Test
    public void missingIsEmptyOnlyWhenAllMeasured() {
        assertTrue(full().missing().isEmpty());
        V2Calibration d = new V2Calibration();
        d.setGate(0.2, 0.8);
        assertNull(V2Calibration.checkPair("gate", 0.2, 0.8, V2Calibration.MIN_GATE_TRAVEL));
        assertFalse(d.missing().contains("gate CLOSED / OPEN"));
        assertEquals(5, d.missing().size());
    }

    @Test
    public void formatEndsWithTheCompletenessMarker() {
        String text = full().format("x");
        assertTrue(text.endsWith("end=ok\n"));
        assertTrue(new V2Calibration().format("x").endsWith("end=ok\n"));
    }

    @Test
    public void cutShortFileWarnsButKeepsWhatItRead() {
        String text = full().format("x");
        String cut = text.substring(0, text.indexOf("beam.staged"));      // lost the tail (and end=ok)
        V2Calibration c = V2Calibration.parse(cut);
        assertTrue(c.gateCalibrated());                                   // read before the cut
        assertFalse(c.queueTicksMeasured());                              // lost
        assertTrue(c.warnings().toString(), c.warnings().get(0).contains("incomplete"));
    }

    @Test
    public void typoInAKnownSectionWarns() {
        V2Calibration c = V2Calibration.parse("gate.cloesd=0.2\ndir.lancher2=REVERSE\nqueue.ticksPerBal=300\n"
                + "beam.staged.brokenWhenLo=true\nintake.jamAmp=3\narm.stowd=0.1\nother.key=1\nend=ok\n");
        assertEquals(c.warnings().toString(), 6, c.warnings().size());   // other.key: no known prefix, silent
        for (String w : c.warnings()) {
            assertTrue(w, w.startsWith("unknown key"));
        }
    }

    @Test
    public void everyWrittenKeyIsKnown() {
        V2Calibration c = V2Calibration.parse(full().format("x"));
        assertTrue(c.warnings().toString(), c.warnings().isEmpty());
        for (String line : full().format("x").split("\n")) {
            if (line.isEmpty() || line.startsWith("#")) continue;
            String key = line.substring(0, line.indexOf('='));
            assertTrue(key, V2Calibration.knownKeys().contains(key));
        }
    }

    // ---------------- V2Config
    @Test
    public void configNamesAreUnique() {
        List<String> all = new ArrayList<>(V2Config.MOTORS);
        all.addAll(V2Config.CR_SERVOS);
        all.addAll(V2Config.POSITION_SERVOS);
        all.addAll(V2Config.BEAMS);
        all.addAll(Arrays.asList(V2Config.IMU, V2Config.PINPOINT, V2Config.WEBCAM));
        assertEquals(all.size(), new HashSet<>(all).size());
    }
}

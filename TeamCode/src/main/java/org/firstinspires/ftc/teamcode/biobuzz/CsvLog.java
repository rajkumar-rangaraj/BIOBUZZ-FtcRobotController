package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Writes test data to /sdcard/BioBuzzLogs/&lt;name&gt;_&lt;date&gt;.csv on the Control Hub.
 * Copy logs to a laptop with:  adb pull /sdcard/BioBuzzLogs
 * A logging failure never stops the OpMode; it only disables the log.
 */
public class CsvLog {
    private BufferedWriter out;
    private final String fileName;
    private int rows = 0;
    private final com.qualcomm.robotcore.util.ElapsedTime flushTimer = new com.qualcomm.robotcore.util.ElapsedTime();

    public CsvLog(String name, String... header) {
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        fileName = name + "_" + stamp + ".csv";
        try {
            File dir = new File(AppUtil.ROOT_FOLDER, "BioBuzzLogs");
            if (!dir.exists() && !dir.mkdirs()) {
                throw new IOException("cannot create " + dir);
            }
            out = new BufferedWriter(new FileWriter(new File(dir, fileName)));
            row((Object[]) header);
            rows = 0;
        } catch (IOException e) {
            out = null;
        }
    }

    public void row(Object... values) {
        if (out == null) {
            return;
        }
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < values.length; i++) {
            if (i > 0) {
                sb.append(',');
            }
            Object v = values[i];
            String s = v instanceof Double ? String.format(Locale.US, "%.3f", (Double) v) : String.valueOf(v);
            if (s.contains(",") || s.contains("\"")) {
                s = "\"" + s.replace("\"", "\"\"") + "\"";
            }
            sb.append(s);
        }
        try {
            out.write(sb.toString());
            out.newLine();
            rows++;
            if (flushTimer.milliseconds() > 1000) {                 // survive a power cut: at most 1 s of data lost
                out.flush();
                flushTimer.reset();
            }
        } catch (IOException e) {
            out = null;
        }
    }

    public String status() {
        return out == null ? "NOT LOGGING (storage error)" : fileName + " (" + rows + " rows)";
    }

    public void close() {
        if (out == null) {
            return;
        }
        try {
            out.close();
        } catch (IOException ignored) {
        }
        out = null;
    }
}

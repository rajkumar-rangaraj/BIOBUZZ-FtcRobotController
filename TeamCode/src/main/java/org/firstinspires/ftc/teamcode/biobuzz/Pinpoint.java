package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Optional goBILDA Pinpoint. {@link #tryCreate} returns null when "pinpoint" is not in the robot configuration,
 * so every caller keeps working on wheel encoders + Control Hub IMU without it.
 */
public class Pinpoint {
    private final GoBildaPinpointDriver dev;
    private double x, y, heading;
    private int rawX, rawY;
    private GoBildaPinpointDriver.DeviceStatus status = GoBildaPinpointDriver.DeviceStatus.NOT_READY;
    private PinpointCalibration cal;

    private Pinpoint(GoBildaPinpointDriver dev, boolean raw) {
        this.dev = dev;
        dev.setEncoderResolution(PINPOINT_SWINGARM_PODS ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
                : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        apply(raw ? new PinpointCalibration(false, false, 0, 0, false) : PinpointCalibration.load());
    }

    /** Use these pod directions / offsets from now on. */
    public void apply(PinpointCalibration c) {
        cal = c;
        dev.setOffsets(c.xPodLeftMm, c.yPodForwardMm, DistanceUnit.MM);
        dev.setEncoderDirections(
                c.xReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                c.yReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    public PinpointCalibration calibration() {
        return cal;
    }

    public static Pinpoint tryCreate(HardwareMap hw) {
        return tryCreate(hw, false);
    }

    /** @param raw both pods FORWARD and offsets 0 (for BioBuzz Pinpoint Setup, which measures them). */
    public static Pinpoint tryCreate(HardwareMap hw, boolean raw) {
        try {
            GoBildaPinpointDriver d = hw.tryGet(GoBildaPinpointDriver.class, PINPOINT);
            return d == null ? null : new Pinpoint(d, raw);
        } catch (Exception e) {
            return null;
        }
    }

    /** Zero position and heading (robot must be still for ~0.25 s afterwards while the IMU calibrates). */
    public void reset() {
        dev.resetPosAndIMU();
    }

    public void update() {
        try {
            dev.update();
            status = dev.getDeviceStatus();
            x = dev.getPosX(DistanceUnit.INCH);
            y = dev.getPosY(DistanceUnit.INCH);
            heading = dev.getHeading(AngleUnit.DEGREES);
            rawX = dev.getEncoderX();
            rawY = dev.getEncoderY();
        } catch (Exception e) {
            status = GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ;
        }
    }

    /** Raw pod encoder ticks (forward pod / strafe pod). */
    public int rawX() {
        return rawX;
    }

    public int rawY() {
        return rawY;
    }

    public boolean ok() {
        return status == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    public String status() {
        return String.valueOf(status);
    }

    /** Inches forward of the reset point (field frame at reset). */
    public double xIn() {
        return x;
    }

    /** Inches LEFT of the reset point. */
    public double yIn() {
        return y;
    }

    /** CCW positive, degrees. */
    public double headingDeg() {
        return heading;
    }
}

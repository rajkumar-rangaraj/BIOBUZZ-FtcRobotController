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
    private GoBildaPinpointDriver.DeviceStatus status = GoBildaPinpointDriver.DeviceStatus.NOT_READY;

    private Pinpoint(GoBildaPinpointDriver dev) {
        this.dev = dev;
        dev.setOffsets(PINPOINT_X_POD_LEFT_MM, PINPOINT_Y_POD_FORWARD_MM, DistanceUnit.MM);
        dev.setEncoderResolution(PINPOINT_SWINGARM_PODS ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
                : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        dev.setEncoderDirections(
                PINPOINT_X_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                PINPOINT_Y_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    public static Pinpoint tryCreate(HardwareMap hw) {
        try {
            GoBildaPinpointDriver d = hw.tryGet(GoBildaPinpointDriver.class, PINPOINT);
            return d == null ? null : new Pinpoint(d);
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
        } catch (Exception e) {
            status = GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ;
        }
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

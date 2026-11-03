package org.firstinspires.ftc.teamcode.pedro;

import static org.firstinspires.ftc.teamcode.biobuzz.BioBuzzConstants.*;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing 3 setup for the BIOBUZZ robot.
 * Tune with AutoTune (http://192.168.43.1:10158 while on the robot Wi-Fi): Mecanum -> Pinpoint -> Foresight -> Tests,
 * then paste the generated values here and set {@link #FORESIGHT_TUNED} to true.
 */
public class Constants {

    /** Field Tour refuses to drive until the Foresight values below come from the AutoTune Foresight procedure. */
    public static final boolean FORESIGHT_TUNED = false;

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(LEFT_FRONT_DRIVE);
        c.backLeftName.set(LEFT_BACK_DRIVE);
        c.frontRightName.set(RIGHT_FRONT_DRIVE);
        c.backRightName.set(RIGHT_BACK_DRIVE);
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);    // same as BioBuzzRobot
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    /** From the AutoTune Pinpoint procedure, 2026-11-02 (tape measure was -40 mm / -175 mm). */
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(PINPOINT);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-1.8471144128033496);
        c.yPodOffset.set(-5.917046013779528);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    /** PLACEHOLDER (Pedro docs example) until the AutoTune Foresight procedure has been run on this robot. */
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.forwardTranslational.set(Controller.piecewise(Controller.proportional(0.1)).put(2.5, Controller.proportional(0.3)));
        c.strafeTranslational.set(Controller.piecewise(Controller.proportional(0.1)).put(2.5, Controller.proportional(0.3)));
        c.coast.set(Controller.proportionalFeedforward(0.010978350889324107));
        c.brake.set(Controller.proportionalFeedforward(0.008731598255925491));
        c.headingFeedback.set(Controller.proportional(5.258721785960744));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695));
        c.linearBrakeCoefficients.set(Matrix.diag(0.10605894992901523, 0.08719146175596092));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014663966976606565, 0.0013837064502458813));
        c.maxAchievableForwardVelocity.set(72.72923108818539);
        c.maxAchievableStrafeVelocity.set(52.34323936525474);
        c.naturalForwardDeceleration.set(85.01144677379789);
        c.naturalStrafeDeceleration.set(104.49787535782846);
    });

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}

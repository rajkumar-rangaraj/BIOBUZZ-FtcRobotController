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
    public static final boolean FORESIGHT_TUNED = true;

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

    /** From the AutoTune Pinpoint procedure, 2026-11-04 (2 Nov run: -1.85 / -5.92 in; tape measure: -40 mm / -175 mm). */
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(PINPOINT);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-1.2882707250399854);
        c.yPodOffset.set(-7.2705378494863435);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    /** From the AutoTune Foresight procedure, 2026-11-04. */
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.21631146018329875);
        Controller secondaryTranslationalForward = Controller.proportional(0.07992130458906194);
        Controller primaryTranslationalLateral = Controller.proportional(0.27236376951218844);
        Controller secondaryTranslationalLateral = Controller.proportional(0.10063113514079707);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.016453592398691385));
        c.brake.set(Controller.proportionalFeedforward(0.013985553538887677));

        c.headingFeedback.set(Controller.proportional(3.9329043862132984));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05495723593955732, 0.0033123865056223713));

        c.linearBrakeCoefficients.set(Matrix.diag(0.0749135997896932, 0.07688701227589287));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0013044997782294713, 9.807392296463033E-4));

        c.maxAchievableForwardVelocity.set(61.38983901518153);
        c.maxAchievableStrafeVelocity.set(50.279659611815866);
        c.naturalForwardDeceleration.set(30.33899445527);
        c.naturalStrafeDeceleration.set(61.79525763531984);
    });

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}

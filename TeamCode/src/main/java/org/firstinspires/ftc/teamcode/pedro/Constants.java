package org.firstinspires.ftc.teamcode.pedro;


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


public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("mfl");
        c.frontRightName.set("mfr");
        c.backLeftName.set("mbl");
        c.backRightName.set("mbr");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pip");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-4.138629342627338);
        c.yPodOffset.set(-2.5808229221133736);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });


    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3751934999206755);
                Controller secondaryTranslationalForward = Controller.proportional(0.13862397286573214);
                Controller primaryTranslationalLateral = Controller.proportional(0.6296370671557231);
                Controller secondaryTranslationalLateral = Controller.proportional(0.23263407210174936);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.019931540189627804));
                c.brake.set(Controller.proportionalFeedforward(0.016941809161183635));

                c.headingFeedback.set(Controller.proportional(6.203994897293285));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04804018662806453, 0.0100965568865441));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07819414635065416, 0.09336959312099646));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002120464352729402, 8.364008127927858E-4));

                c.maxAchievableForwardVelocity.set(48.780929302799684);
                c.maxAchievableStrafeVelocity.set(35.62332885184409);
                c.naturalForwardDeceleration.set(30.85917977836366);
                c.naturalStrafeDeceleration.set(57.50904632278812);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
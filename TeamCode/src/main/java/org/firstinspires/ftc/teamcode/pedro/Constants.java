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
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pip");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(2.114244446041077);
        c.yPodOffset.set(4.519837146668922);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3716952838641803);
                Controller secondaryTranslationalForward = Controller.proportional(0.13733147550691172);
                Controller primaryTranslationalLateral = Controller.proportional(1.1124192312765373);
                Controller secondaryTranslationalLateral = Controller.proportional(0.4110091815673789);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017293842199150335));
                c.brake.set(Controller.proportionalFeedforward(0.014699765869277785));

                c.headingFeedback.set(Controller.proportional(7.33470559980554));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06554739108208665, 0.003250885072778692));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11882447514947582, 0.032263241535290275));
                c.quadraticBrakeCoefficients.set(Matrix.diag(9.626726861636657E-4, 0.0023806693384184857));

                c.maxAchievableForwardVelocity.set(60.31093566276404);
                c.maxAchievableStrafeVelocity.set(44.85876579937071);
                c.naturalForwardDeceleration.set(29.321892199643546);
                c.naturalStrafeDeceleration.set(68.90806987312881);
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
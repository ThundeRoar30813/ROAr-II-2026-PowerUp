package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class DriveSubsystem {

    private final Follower follower;

    public DriveSubsystem(HardwareMap hardwareMap) {

        follower = Constants.create(hardwareMap);
    }


    // =====================================================
    // UPDATE
    // =====================================================

    /*
     * IMPORTANTE:
     *
     * Pedro necesita update() constantemente.
     */
    public void update() {

        follower.update();
    }


    // =====================================================
    // DRIVE
    // =====================================================

    public void drive(
            double forward,
            double strafe,
            double turn
    ) {

        follower.manual(
                forward,
                strafe,
                turn
        );
    }


    // =====================================================
    // STOP
    // =====================================================

    public void stop() {

        follower.manual(
                0,
                0,
                0
        );
    }


    // =====================================================
    // POSE COMPLETA
    // =====================================================

    public Pose getPose() {

        return follower.pose();
    }


    // =====================================================
    // X
    // =====================================================

    public double getX() {

        return follower.pose().x();
    }


    // =====================================================
    // Y
    // =====================================================

    public double getY() {

        return follower.pose().y();
    }


    // =====================================================
    // HEADING RADIANS
    // =====================================================

    public double getHeadingRadians() {

        return follower.pose().heading();
    }


    // =====================================================
    // HEADING GRADOS
    // =====================================================

    public double getHeadingDegrees() {

        return Math.toDegrees(
                follower.pose().heading()
        );
    }


    // =====================================================
    // CAMBIAR POSE
    // =====================================================

    public void setPose(Pose pose) {

        follower.setPose(pose);
    }


    // =====================================================
    // FOLLOWER
    // =====================================================

    /*
     * Lo dejamos disponible porque después
     * lo necesitaremos para Auto / Paths.
     */
    public Follower getFollower() {

        return follower;
    }
}
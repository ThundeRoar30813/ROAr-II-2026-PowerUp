
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class DriveSubsystem {

    // =========================================
    // FOLLOWER
    // =========================================

    private final Follower follower;

    // =========================================
    // CONSTRUCTOR NORMAL - TELEOP
    // =========================================

    public DriveSubsystem(HardwareMap hardwareMap) {

        follower = Constants.create(hardwareMap);
    }

    // =========================================
    // CONSTRUCTOR COMPARTIDO - AUTONOMO IVY
    // =========================================

    public DriveSubsystem(Follower follower) {

        if (follower == null) {
            throw new IllegalArgumentException(
                    "El Follower no puede ser null"
            );
        }

        this.follower = follower;
    }

    // =========================================
    // UPDATE
    // =========================================

    public void update() {

        follower.update();
    }

    // =========================================
    // MANUAL DRIVE
    // =========================================

    public void drive(
            double forward,
            double strafe,
            double turn) {

        follower.manual(
                forward,
                strafe,
                turn
        );
    }

    // =========================================
    // STOP
    // =========================================

    public void stop() {

        follower.manual(0, 0, 0);
    }

    // =========================================
    // SET INITIAL POSE
    // =========================================

    public void setInitialPose(
            double x,
            double y,
            double headingDegrees) {

        follower.setPose(
                new Pose(
                        x,
                        y,
                        Math.toRadians(headingDegrees)
                )
        );
    }

    // =========================================
    // SET POSE
    // =========================================

    public void setPose(Pose pose) {

        follower.setPose(pose);
    }

    // =========================================
    // GET POSE
    // =========================================

    public Pose getPose() {

        return follower.pose();
    }

    // =========================================
    // POSITION
    // =========================================

    public double getX() {

        return follower.pose().x();
    }

    public double getY() {

        return follower.pose().y();
    }

    // =========================================
    // HEADING
    // =========================================

    public double getHeadingDegrees() {

        return Math.toDegrees(
                follower.pose().heading()
        );
    }

    public double getHeadingRadians() {

        return follower.pose().heading();
    }

    // =========================================
    // GET FOLLOWER
    // =========================================

    public Follower getFollower() {

        return follower;
    }
}

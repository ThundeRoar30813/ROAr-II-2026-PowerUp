
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class DriveSubsystem {

    private final Follower follower;

    public DriveSubsystem(HardwareMap hardwareMap) {
        this(Constants.create(hardwareMap));
    }

    public DriveSubsystem(Follower follower) {
        if (follower == null) {
            throw new IllegalArgumentException("Follower null");
        }
        this.follower = follower;
    }

    public void update() {
        follower.update();
    }

    public void drive(double forward,
                      double strafe,
                      double turn) {
        follower.manual(forward, strafe, turn);
    }

    public void stop() {
        follower.manual(0, 0, 0);
    }

    public void setInitialPose(
            double x, double y, double headingDegrees) {
        follower.setPose(new Pose(
                x, y, Math.toRadians(headingDegrees)));
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public Pose getPose() {
        return follower.pose();
    }

    public double getX() {
        return follower.pose().x();
    }

    public double getY() {
        return follower.pose().y();
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(follower.pose().heading());
    }

    public double getHeadingRadians() {
        return follower.pose().heading();
    }

    public Follower getFollower() {
        return follower;
    }
}

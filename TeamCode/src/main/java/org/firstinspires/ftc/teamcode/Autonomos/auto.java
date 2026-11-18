package org.firstinspires.ftc.teamcode.Autonomos;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class auto {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(7.9784, 23.7634, 90);
    private final Pose pathe1 = poseFactory.of(16.5562, 23.7742, 270);
    private final Pose point2 = poseFactory.of(8.515, 23.7698, 270);
    private final Pose point3 = poseFactory.of(8.1142, 7.2295, 270);
    private final Pose point4 = poseFactory.of(22.4629, 9.9337, 270);
    private final Pose point5 = poseFactory.of(11.7032, 45.7363, 180);
    private final Pose point6 = poseFactory.of(15.2011, 45.8076, 180);
    private final Pose point7 = poseFactory.of(22.7815, 11.8888, 180);
    private final Pose point8 = poseFactory.of(7.0547, 98.981, 180);

    public Path pathe1() {
        return Paths.line(start, pathe1).constant(pathe1);
    }

    public Path path2() {
        return Paths.line(pathe1, point2).constant(point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).constant(point3);
    }

    public Path path4() {
        return Paths.line(point3, point4).constant(point4);
    }

    public Path path5() {
        return Paths.line(point4, point5).constant(point5);
    }

    public Path path6() {
        return Paths.line(point5, point6).constant(point6);
    }

    public Path path7() {
        return Paths.line(point6, point7).constant(point7);
    }

    public Path path8() {
        return Paths.line(point7, point8).constant(point8);
    }
}

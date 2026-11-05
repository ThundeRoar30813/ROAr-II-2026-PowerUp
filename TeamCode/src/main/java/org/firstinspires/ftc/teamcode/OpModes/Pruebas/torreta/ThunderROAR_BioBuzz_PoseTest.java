
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

@TeleOp(name = "ThunderROAR - BIO BUZZ Pose Test")
public class ThunderROAR_BioBuzz_PoseTest extends OpMode {

    private DriveSubsystem drive;

    // ==========================================
    // POSICION INICIAL PROVISIONAL
    // ==========================================

    private static final double START_X = 24.0;
    private static final double START_Y = 24.0;
    private static final double START_HEADING_DEG = 90.0;

    // ==========================================
    // INIT
    // ==========================================

    @Override
    public void init() {

        drive = new DriveSubsystem(hardwareMap);

        // Establecer la posicion inicial
        drive.setPose(
                new Pose(
                        START_X,
                        START_Y,
                        Math.toRadians(START_HEADING_DEG)
                )
        );

        telemetry.addLine("THUNDERROAR - BIO BUZZ");
        telemetry.addLine("Pose inicial configurada");

        telemetry.addData("Start X", START_X);
        telemetry.addData("Start Y", START_Y);
        telemetry.addData(
                "Start Heading",
                START_HEADING_DEG
        );

        telemetry.update();
    }

    // ==========================================
    // LOOP
    // ==========================================

    @Override
    public void loop() {

        // Control mecanum manual
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        drive.drive(forward, strafe, turn);

        // Actualizar Pedro Pathing y Pinpoint
        drive.update();

        // Posicion actual
        double x = drive.getX();
        double y = drive.getY();
        double heading = drive.getHeadingDegrees();

        telemetry.addLine("===== BIO BUZZ ODOMETRY =====");

        telemetry.addData("X (in)", "%.2f", x);
        telemetry.addData("Y (in)", "%.2f", y);
        telemetry.addData("Heading (deg)", "%.2f", heading);

        telemetry.addLine("-----------------------------");

        telemetry.addData("Initial X", START_X);
        telemetry.addData("Initial Y", START_Y);
        telemetry.addData(
                "Initial Heading",
                START_HEADING_DEG
        );

        telemetry.update();
    }

    // ==========================================
    // STOP
    // ==========================================

    @Override
    public void stop() {
        drive.stop();
        drive.update();
    }
}

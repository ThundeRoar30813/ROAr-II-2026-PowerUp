
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;

@TeleOp(name = "BIOBUZZ - DASHBOARD")
public class BioBuzzDashboard extends OpMode {

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCmd;

    private FtcDashboard dashboard;

    // ==========================================
    // CONFIGURACION DEL CAMPO
    // ==========================================

    // Cancha FTC de 144 x 144 pulgadas
    private static final double FIELD_SIZE = 144.0;
    private static final double FIELD_HALF = FIELD_SIZE / 2.0;

    // Dimensiones aproximadas del robot
    private static final double ROBOT_RADIUS = 9.0;

    // Pose inicial: CAMBIAR SEGUN POSICION REAL
    private static final double START_X = 48.0;
    private static final double START_Y = 6.0;
    private static final double START_HEADING = 90.0;

    // Objetivo de prueba, NO coordenadas oficiales
    private static final double GOAL_X = 62.0;
    private static final double GOAL_Y = 57.0;

    @Override
    public void init() {

        drive = new DriveSubsystem(hardwareMap);

        drive.setInitialPose(
                START_X,
                START_Y,
                START_HEADING
        );

        driveCmd = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> gamepad1.left_stick_x,
                () -> gamepad1.right_stick_x
        );

        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("BIOBUZZ DASHBOARD LISTO");
        telemetry.addLine("Verifica la pose inicial");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ======================================
        // ACTUALIZAR PEDRO
        // ======================================

        driveCmd.execute();
        drive.update();

        double x = drive.getX();
        double y = drive.getY();

        double headingDeg = drive.getHeadingDegrees();
        double headingRad = Math.toRadians(headingDeg);

        // ======================================
        // DISTANCIA AL PUNTO DE PRUEBA
        // ======================================

        double dx = GOAL_X - x;
        double dy = GOAL_Y - y;

        double distanceInches = Math.hypot(dx, dy);
        double distanceCm = distanceInches * 2.54;

        // ======================================
        // TELEMETRIA DRIVER STATION
        // ======================================

        telemetry.addLine("===== BIOBUZZ =====");

        telemetry.addData("X (in)", "%.2f", x);
        telemetry.addData("Y (in)", "%.2f", y);

        telemetry.addData(
                "Heading (deg)",
                "%.2f",
                headingDeg
        );

        telemetry.addData(
                "Distancia punto prueba (cm)",
                "%.1f",
                distanceCm
        );

        telemetry.update();

        // ======================================
        // FTC DASHBOARD
        // ======================================

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Robot X (in)", x);
        packet.put("Robot Y (in)", y);
        packet.put("Heading (deg)", headingDeg);
        packet.put("Distancia punto prueba (cm)", distanceCm);

        // Dashboard usa origen en centro del campo.
        double robotDashX = x - FIELD_HALF;
        double robotDashY = y - FIELD_HALF;

        double goalDashX = GOAL_X - FIELD_HALF;
        double goalDashY = GOAL_Y - FIELD_HALF;

        // DIBUJAR ROBOT
        packet.fieldOverlay()
                .setStroke("#2563EB")
                .setStrokeWidth(2)
                .strokeCircle(
                        robotDashX,
                        robotDashY,
                        ROBOT_RADIUS
                );

        // DIBUJAR ORIENTACION
        packet.fieldOverlay()
                .setStroke("#2563EB")
                .strokeLine(
                        robotDashX,
                        robotDashY,
                        robotDashX + 14.0 * Math.cos(headingRad),
                        robotDashY + 14.0 * Math.sin(headingRad)
                );

        // DIBUJAR PUNTO OBJETIVO
        packet.fieldOverlay()
                .setFill("#16A34A")
                .fillCircle(
                        goalDashX,
                        goalDashY,
                        4.0
                );

        // DIBUJAR DISTANCIA
        packet.fieldOverlay()
                .setStroke("#EAB308")
                .setStrokeWidth(1)
                .strokeLine(
                        robotDashX,
                        robotDashY,
                        goalDashX,
                        goalDashY
                );

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {

        if (driveCmd != null) {
            driveCmd.stop();
        }
    }
}

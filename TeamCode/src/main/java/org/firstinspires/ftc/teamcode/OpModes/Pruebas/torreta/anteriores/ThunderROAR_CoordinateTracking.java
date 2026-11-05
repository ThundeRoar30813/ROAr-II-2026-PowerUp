
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta.anteriores;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Commands.TurretCoordinateTrackingCommand;

import com.pedropathing.math.Pose;

@TeleOp(name = "ThunderROAR Coordinate Tracking")
public class ThunderROAR_CoordinateTracking extends OpMode {

    private DriveSubsystem drive;
    private TurretSubsystem turret;

    private TurretCoordinateTrackingCommand tracking;

    // Coordenadas de prueba.
    // Deben estar en el mismo sistema que Pedro Pathing.
    private static final double START_X = 27.5888;
    private static final double START_Y = 116.1869;
    private static final double START_HEADING_DEG = 180.0;

    // Objetivo provisional para la prueba.
    private static final double GOAL_X = 42.3925;
    private static final double GOAL_Y = 100.9346;

    private boolean previousX = false;

    @Override
    public void init() {

        drive = new DriveSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        drive.setPose(new Pose(
                START_X,
                START_Y,
                Math.toRadians(START_HEADING_DEG)
        ));

        tracking = new TurretCoordinateTrackingCommand(
                drive,
                turret
        );

        tracking.setTarget(GOAL_X, GOAL_Y);

        telemetry.addLine("ThunderROAR Coordinate Tracking");
        telemetry.addLine("Coloca la torreta en el centro antes de INIT");
        telemetry.addLine("Apunta manualmente al objetivo antes de activar X");
        telemetry.addLine("X = activar/desactivar tracking");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ========================================
        // CHASIS MECANUM
        // ========================================

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // Reducir velocidad para primeras pruebas
        double speed = 0.35;

        drive.drive(
                forward * speed,
                strafe * speed,
                turn * speed
        );

        drive.update();

        // ========================================
        // BOTÓN X: TRACKING
        // ========================================

        boolean x = gamepad1.x;

        if (x && !previousX) {

            if (tracking.isEnabled()) {
                tracking.disable();
            } else {
                tracking.enable();
            }
        }

        previousX = x;

        // ========================================
        // TORRETA
        // ========================================

        tracking.update();

        // ========================================
        // TELEMETRÍA
        // ========================================

        telemetry.addLine("=== PINPOINT ===");

        telemetry.addData("Robot X", "%.2f", drive.getX());
        telemetry.addData("Robot Y", "%.2f", drive.getY());
        telemetry.addData(
                "Heading",
                "%.2f",
                drive.getHeadingDegrees()
        );

        telemetry.addLine("=== OBJETIVO ===");

        telemetry.addData(
                "Goal X",
                "%.2f",
                tracking.getTargetX()
        );

        telemetry.addData(
                "Goal Y",
                "%.2f",
                tracking.getTargetY()
        );

        telemetry.addLine("=== TORRETA ===");

        telemetry.addData(
                "Tracking",
                tracking.isEnabled()
        );

        telemetry.addData(
                "Reachable",
                tracking.isReachable()
        );

        telemetry.addData(
                "Requested",
                "%.2f",
                tracking.getRequestedAngleDeg()
        );

        telemetry.addData(
                "Target",
                "%.2f",
                tracking.getCommandedAngleDeg()
        );

        telemetry.addData(
                "Actual",
                "%.2f",
                turret.getCurrentAngleDeg()
        );

        telemetry.addData(
                "Error",
                "%.2f",
                turret.getErrorDeg()
        );

        telemetry.addData(
                "Motor Power",
                "%.3f",
                turret.getMotorPower()
        );

        telemetry.update();
    }

    @Override
    public void stop() {

        tracking.disable();
        drive.stop();
    }
}

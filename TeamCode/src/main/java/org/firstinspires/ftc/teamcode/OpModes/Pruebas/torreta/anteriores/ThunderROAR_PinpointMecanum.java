package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta.anteriores;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;
@Disabled
@TeleOp(name = "ThunderROAR - PINPOINT + MECANUM")
public class ThunderROAR_PinpointMecanum extends OpMode {

    // =========================================
    // SUBSYSTEMS
    // =========================================

    private DriveSubsystem drive;
    private TurretSubsystem turret;

    private MecanumDriveCmd driveCmd;

    // =========================================
    // TRACKING
    // =========================================

    private boolean trackingEnabled = false;
    private boolean previousX = false;

    private double startingHeading = 0.0;
    private double startingTurretAngle = 0.0;

    private double targetAngle = 0.0;
    private double targetVelocity = 0.0;

    private double previousTarget = 0.0;
    private long previousTime;

    // Signo que ya funciono en tu robot
    private static final double TRACKING_SIGN = -1.0;

    // Feedforward
    private static final double MAX_TARGET_VELOCITY = 500.0;

    // =========================================
    // INIT
    // =========================================

    @Override
    public void init() {

        drive = new DriveSubsystem(hardwareMap);

        turret = new TurretSubsystem(hardwareMap);

        // =====================================
        // CHASIS MECANUM
        // =====================================

        driveCmd = new MecanumDriveCmd(
                drive,

                // Adelante / atras
                () -> -gamepad1.left_stick_y,

                // Movimiento lateral
                () -> gamepad1.left_stick_x,

                // Giro
                () -> gamepad1.right_stick_x
        );

        targetAngle = turret.getCurrentAngleDeg();

        previousTime = System.nanoTime();

        telemetry.addLine("THUNDERROAR READY");
        telemetry.addLine("X = Tracking ON/OFF");
        telemetry.addLine("Stick izquierdo = Traslacion");
        telemetry.addLine("Stick derecho = Giro");
        telemetry.update();
    }

    // =========================================
    // LOOP
    // =========================================

    @Override
    public void loop() {

        // =====================================
        // 1. ACTUALIZAR PINPOINT
        // =====================================

        drive.update();

        // =====================================
        // 2. CONTROLAR CHASIS
        // =====================================

        driveCmd.execute();

        // =====================================
        // 3. TOGGLE TRACKING
        // =====================================

        boolean currentX = gamepad1.x;

        if (currentX && !previousX) {

            trackingEnabled = !trackingEnabled;

            if (trackingEnabled) {

                startingHeading =
                        drive.getHeadingDegrees();

                startingTurretAngle =
                        turret.getCurrentAngleDeg();

                targetAngle = startingTurretAngle;

                previousTarget = targetAngle;

                targetVelocity = 0.0;

                previousTime = System.nanoTime();

            } else {

                turret.stop();
                targetVelocity = 0.0;
            }
        }

        previousX = currentX;

        // =====================================
        // 4. LEER HEADING
        // =====================================

        double heading =
                drive.getHeadingDegrees();

        double robotRotation = angleDifference(
                heading,
                startingHeading
        );

        // =====================================
        // 5. TRACKING DE TORRETA
        // =====================================

        if (trackingEnabled) {

            // Mantener signo -1 que funciono
            targetAngle = startingTurretAngle
                    - TRACKING_SIGN * robotRotation;

            // Respetar limites mecanicos
            targetAngle = clamp(
                    targetAngle,
                    TurretSubsystem.MIN_ANGLE_DEG,
                    TurretSubsystem.MAX_ANGLE_DEG
            );

            // =================================
            // VELOCIDAD OBJETIVO
            // =================================

            long now = System.nanoTime();

            double dt =
                    (now - previousTime) / 1_000_000_000.0;

            previousTime = now;

            if (dt <= 0.0 || dt > 0.1) {
                dt = 0.02;
            }

            double angleChange =
                    targetAngle - previousTarget;

            targetVelocity = clamp(
                    angleChange / dt,
                    -MAX_TARGET_VELOCITY,
                    MAX_TARGET_VELOCITY
            );

            previousTarget = targetAngle;

            // =================================
            // ENVIAR OBJETIVO
            // =================================

            turret.setTargetAngleDeg(targetAngle);

            turret.setTargetVelocityDegPerSec(
                    targetVelocity
            );

            turret.update();
        }

        // =====================================
        // 6. TELEMETRIA
        // =====================================

        telemetry.addLine("===== ROBOT =====");

        telemetry.addData(
                "X",
                "%.2f",
                drive.getX()
        );

        telemetry.addData(
                "Y",
                "%.2f",
                drive.getY()
        );

        telemetry.addData(
                "Heading",
                "%.2f",
                heading
        );

        telemetry.addData(
                "Robot Rotation",
                "%.2f",
                robotRotation
        );

        telemetry.addLine("===== TURRET =====");

        telemetry.addData(
                "Tracking",
                trackingEnabled ? "ON" : "OFF"
        );

        telemetry.addData(
                "Target",
                "%.2f",
                turret.getTargetAngleDeg()
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
                "Target Velocity",
                "%.2f",
                targetVelocity
        );

//        telemetry.addData(
//                "PID",
//                "%.3f",
//                turret.getPIDOutput()
//        );
//
//        telemetry.addData(
//                "Feedforward",
//                "%.3f",
//                turret.getFeedforward()
//        );

        telemetry.addData(
                "Motor Power",
                "%.3f",
                turret.getMotorPower()
        );

        telemetry.update();
    }

    // =========================================
    // DIFERENCIA ANGULAR
    // =========================================

    private double angleDifference(
            double current,
            double reference
    ) {

        double difference = current - reference;

        while (difference > 180.0) {
            difference -= 360.0;
        }

        while (difference < -180.0) {
            difference += 360.0;
        }

        return difference;
    }

    // =========================================
    // CLAMP
    // =========================================

    private double clamp(
            double value,
            double min,
            double max
    ) {

        return Math.max(
                min,
                Math.min(max, value)
        );
    }

    // =========================================
    // STOP
    // =========================================

    @Override
    public void stop() {

        if (turret != null) {
            turret.stop();
        }

        if (driveCmd != null) {
            driveCmd.end();
        } else if (drive != null) {
            drive.stop();
        }
    }
}

package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

@TeleOp(name = "ThunderROAR - PINPOINT TEST")
public class ThunderROAR_PinpointTest extends OpMode {

    private DriveSubsystem drive;
    private TurretSubsystem turret;

    private boolean trackingEnabled = false;
    private boolean previousX = false;

    private double startingHeading = 0.0;
    private double startingTurretAngle = 0.0;

    private double targetAngle = 0.0;
    private double targetVelocity = 0.0;

    private double previousTarget = 0.0;
    private long previousTime;

    // Cambiar solo si el sentido es incorrecto.
    private static final double TRACKING_SIGN = -1.0;

    private static final double MAX_TARGET_VELOCITY = 500.0;

    @Override
    public void init() {

        drive = new DriveSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        targetAngle = turret.getCurrentAngleDeg();

        previousTime = System.nanoTime();

        telemetry.addLine("PINPOINT TRACKING TEST");
        telemetry.addLine("X = Activar / Desactivar");
        telemetry.addLine("Gira el robot manualmente");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Actualizar odometria
        drive.update();

        // Activar/desactivar
        boolean currentX = gamepad1.x;

        if (currentX && !previousX) {

            trackingEnabled = !trackingEnabled;

            if (trackingEnabled) {

                startingHeading = drive.getHeadingDegrees();

                startingTurretAngle =
                        turret.getCurrentAngleDeg();

                targetAngle = startingTurretAngle;
                previousTarget = targetAngle;

                targetVelocity = 0.0;

                previousTime = System.nanoTime();

            } else {

                turret.stop();
            }
        }

        previousX = currentX;

        double heading = drive.getHeadingDegrees();

        double robotRotation = angleDifference(
                heading,
                startingHeading
        );

        if (trackingEnabled) {

            // Compensar rotacion del robot
            targetAngle = startingTurretAngle
                    - TRACKING_SIGN * robotRotation;

            targetAngle = Math.max(
                    TurretSubsystem.MIN_ANGLE_DEG,
                    Math.min(
                            TurretSubsystem.MAX_ANGLE_DEG,
                            targetAngle
                    )
            );

            long now = System.nanoTime();

            double dt =
                    (now - previousTime) / 1_000_000_000.0;

            previousTime = now;

            if (dt <= 0.0 || dt > 0.1) {
                dt = 0.02;
            }

            double angleChange =
                    targetAngle - previousTarget;

            targetVelocity = angleChange / dt;

            targetVelocity = Math.max(
                    -MAX_TARGET_VELOCITY,
                    Math.min(
                            MAX_TARGET_VELOCITY,
                            targetVelocity
                    )
            );

            previousTarget = targetAngle;

            turret.setTargetAngleDeg(targetAngle);

            turret.setTargetVelocityDegPerSec(
                    targetVelocity
            );

            turret.update();
        }

        telemetry.addLine("====== PINPOINT ======");

        telemetry.addData(
                "Tracking",
                trackingEnabled ? "ON" : "OFF"
        );

        telemetry.addData(
                "Heading",
                "%.2f",
                heading
        );

        telemetry.addData(
                "Starting Heading",
                "%.2f",
                startingHeading
        );

        telemetry.addData(
                "Robot Rotation",
                "%.2f",
                robotRotation
        );

        telemetry.addLine("====== TURRET ======");

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

    @Override
    public void stop() {

        if (turret != null) {
            turret.stop();
        }

        if (drive != null) {
            drive.stop();
        }
    }
}

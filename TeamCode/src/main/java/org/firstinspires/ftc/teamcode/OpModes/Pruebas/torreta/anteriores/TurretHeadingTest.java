package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta.anteriores;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.TurretHeadingCommand;
import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

@TeleOp(name = "TEST TURRET HEADING")
public class TurretHeadingTest extends OpMode {

    private DriveSubsystem drive;
    private TurretSubsystem turret;

    private MecanumDriveCmd driveCmd;
    private TurretHeadingCommand headingTracking;

    private boolean trackingEnabled = false;

    // Para detectar una sola pulsación de X
    private boolean previousX = false;

    @Override
    public void init() {

        // ==========================================
        // SUBSYSTEMS
        // ==========================================

        drive = new DriveSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        // ==========================================
        // MECANUM
        // ==========================================

        driveCmd = new MecanumDriveCmd(
                drive,

                // Stick izquierdo vertical
                () -> -gamepad1.left_stick_y,

                // Stick izquierdo horizontal
                () -> gamepad1.left_stick_x,

                // Stick derecho horizontal = giro
                () -> gamepad1.right_stick_x
        );

        // ==========================================
        // HEADING TRACKING
        // ==========================================

        headingTracking =
                new TurretHeadingCommand(
                        drive,
                        turret
                );

        telemetry.addLine("TEST TURRET HEADING");
        telemetry.addLine("");
        telemetry.addLine("X = activar/desactivar tracking");
        telemetry.addLine("Right Stick X = girar robot");
        telemetry.addLine("");
        telemetry.addLine("Primero gira MUY lento.");
        telemetry.addLine("Luego medio.");
        telemetry.addLine("Luego rapido.");

        telemetry.update();
    }

    @Override
    public void loop() {

        // ==========================================
        // IMPORTANTE: ACTUALIZAR PEDRO
        // ==========================================

        drive.update();

        // ==========================================
        // CONDUCIR
        // ==========================================

        driveCmd.execute();

        // ==========================================
        // TOGGLE TRACKING CON X
        // ==========================================

        boolean currentX = gamepad1.x;

        if (currentX && !previousX) {

            trackingEnabled = !trackingEnabled;

            if (trackingEnabled) {

                /*
                 * En este instante:
                 *
                 * - Guarda el heading inicial.
                 * - La posición actual de la torreta
                 *   pasa a ser 0°.
                 */
                headingTracking.enable();

            } else {

                headingTracking.disable();
            }
        }

        previousX = currentX;

        // ==========================================
        // EJECUTAR TRACKING
        // ==========================================

        if (trackingEnabled) {

            headingTracking.execute();
        }

        // ==========================================
        // TELEMETRÍA
        // ==========================================

        double heading =
                drive.getHeadingDegrees();

        double startingHeading =
                headingTracking.getStartingRobotHeadingDeg();

        double robotRotation =
                angleDifferenceDegrees(
                        heading,
                        startingHeading
                );

        double expectedTurret =
                -robotRotation;

        telemetry.addLine(
                "========== TRACKING =========="
        );

        telemetry.addData(
                "Tracking",
                trackingEnabled ? "ON" : "OFF"
        );

        telemetry.addData(
                "Heading",
                "%.2f deg",
                heading
        );

        telemetry.addData(
                "Starting Heading",
                "%.2f deg",
                startingHeading
        );

        telemetry.addData(
                "Robot Rotation",
                "%.2f deg",
                robotRotation
        );

        telemetry.addData(
                "Expected Turret",
                "%.2f deg",
                expectedTurret
        );

        telemetry.addLine(
                "=========== TURRET ==========="
        );

        telemetry.addData(
                "Target",
                "%.2f deg",
                turret.getTargetAngleDeg()
        );

        telemetry.addData(
                "Actual",
                "%.2f deg",
                turret.getCurrentAngleDeg()
        );

        telemetry.addData(
                "Error",
                "%.2f deg",
                turret.getErrorDeg()
        );

//        telemetry.addData(
//                "PID",
//                "%.3f",
//                turret.getPIDOutput()
//        );
//
//        telemetry.addData(
//                "FF",
//                "%.3f",
//                turret.getFeedforward()
//        );

        telemetry.addData(
                "Power",
                "%.3f",
                turret.getMotorPower()
        );

//        telemetry.addData(
//                "Ticks",
//                turret.getRelativeTicks()
//        );

        telemetry.update();
    }

    @Override
    public void stop() {

        headingTracking.disable();
        drive.stop();
    }

    // ==============================================
    // DIFERENCIA DE ANGULOS
    // ==============================================

    private double angleDifferenceDegrees(
            double current,
            double reference
    ) {

        double difference =
                current - reference;

        while (difference > 180) {
            difference -= 360;
        }

        while (difference < -180) {
            difference += 360;
        }

        return difference;
    }
}
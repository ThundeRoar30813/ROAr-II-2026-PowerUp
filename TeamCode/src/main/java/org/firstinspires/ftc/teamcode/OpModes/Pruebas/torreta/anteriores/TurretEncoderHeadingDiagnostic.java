package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta.anteriores;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
@Disabled
@TeleOp(name = "DIAG Heading + Turret Encoder")
public class TurretEncoderHeadingDiagnostic extends OpMode {

    private DriveSubsystem drive;
    private DcMotorEx turretEncoder;

    // Los mismos valores de tu TurretSubsystem
    private static final double TICKS_PER_REV = 51940.0;
    private static final double TICKS_PER_DEGREE =
            TICKS_PER_REV / 360.0;

    private double startingHeading;
    private int startingEncoderTicks;

    @Override
    public void init() {

        drive = new DriveSubsystem(hardwareMap);

        /*
         * Tu Through Bore está siendo leído
         * desde el puerto llamado "mfl".
         */
        turretEncoder =
                hardwareMap.get(
                        DcMotorEx.class,
                        "mfl"
                );

        telemetry.addLine("DIAGNOSTICO LISTO");
        telemetry.addLine("");
        telemetry.addLine("NO hay PID.");
        telemetry.addLine("NO hay FF.");
        telemetry.addLine("El programa NO mueve la torreta.");
        telemetry.addLine("");
        telemetry.addLine("Presiona START.");
        telemetry.update();
    }

    @Override
    public void start() {

        // Guardamos ambos ceros al mismo tiempo
        startingHeading =
                drive.getHeadingDegrees();

        startingEncoderTicks =
                turretEncoder.getCurrentPosition();
    }

    @Override
    public void loop() {

        // IMPORTANTE:
        // Pedro necesita actualizar constantemente.
        drive.update();

        // =========================================
        // ROBOT HEADING
        // =========================================

        double currentHeading =
                drive.getHeadingDegrees();

        double deltaHeading =
                angleDifferenceDegrees(
                        currentHeading,
                        startingHeading
                );

        // =========================================
        // TURRET ENCODER
        // =========================================

        int currentTicks =
                turretEncoder.getCurrentPosition();

        int relativeTicks =
                currentTicks - startingEncoderTicks;

        /*
         * Mismo signo que utilizas actualmente
         * en TurretSubsystem.
         */
        double turretAngle =
                -(relativeTicks / TICKS_PER_DEGREE);

        // =========================================
        // COMPROBACION
        // =========================================

        /*
         * Si la torreta permanece mirando
         * al MISMO punto del campo:
         *
         * deltaHeading + turretAngle
         *
         * debería permanecer cerca de 0°.
         */
        double combinedAngle =
                deltaHeading + turretAngle;

        // =========================================
        // TELEMETRIA
        // =========================================

        telemetry.addLine(
                "====== ROBOT ======"
        );

        telemetry.addData(
                "Heading actual",
                "%.2f deg",
                currentHeading
        );

        telemetry.addData(
                "Delta Heading",
                "%.2f deg",
                deltaHeading
        );

        telemetry.addLine(
                "====== TURRET ======"
        );

        telemetry.addData(
                "Raw Encoder",
                currentTicks
        );

        telemetry.addData(
                "Relative Ticks",
                relativeTicks
        );

        telemetry.addData(
                "Turret Angle",
                "%.2f deg",
                turretAngle
        );

        telemetry.addLine(
                "====== CHECK ======"
        );

        telemetry.addData(
                "Heading + Turret",
                "%.2f deg",
                combinedAngle
        );

        telemetry.update();
    }

    // =============================================
    // ANGLE DIFFERENCE
    // =============================================

    private double angleDifferenceDegrees(
            double current,
            double reference
    ) {

        double difference =
                current - reference;

        while (difference > 180.0) {
            difference -= 360.0;
        }

        while (difference < -180.0) {
            difference += 360.0;
        }

        return difference;
    }
}
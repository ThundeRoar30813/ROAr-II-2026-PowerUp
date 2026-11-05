package org.firstinspires.ftc.teamcode.OpModes.Pruebas.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TEST - Turret Encoder")
public class TurretEncoderTest extends LinearOpMode {

    private DcMotorEx turretEncoder;

    @Override
    public void runOpMode() {

        /*
         * Aunque físicamente sea un REV Through Bore Encoder,
         * el Control Hub lo lee desde un puerto de encoder
         * asociado a un puerto de motor.
         *
         * En la configuración del robot crea un motor llamado:
         *
         * turretEncoder
         */
        turretEncoder = hardwareMap.get(
                DcMotorEx.class,
                "mfl"
        );

        telemetry.addLine("=== TURRET ENCODER TEST ===");
        telemetry.addLine("NO muevas la torreta todavía.");
        telemetry.addLine("Presiona START.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Guardamos la posición inicial como nuestro cero.
        int zeroTicks = turretEncoder.getCurrentPosition();

        while (opModeIsActive()) {

            int rawTicks =
                    turretEncoder.getCurrentPosition();

            int relativeTicks =
                    rawTicks - zeroTicks;

            telemetry.addLine("=== REV THROUGH BORE V2 ===");

            telemetry.addData(
                    "Raw ticks",
                    rawTicks
            );

            telemetry.addData(
                    "Relative ticks",
                    relativeTicks
            );

            telemetry.addLine("");
            telemetry.addLine("Gira la torreta A MANO.");
            telemetry.addLine("No enciendas el motor.");
            telemetry.addLine("");
            telemetry.addLine("Anota:");
            telemetry.addLine("0°   = ?");
            telemetry.addLine("90°  = ?");
            telemetry.addLine("180° = ?");
            telemetry.addLine("360° = ?");

            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta.anteriores;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled
@TeleOp(name = "TEST - Turret Motor + Encoder")
public class TurretMotorEncoderTest extends LinearOpMode {

    private DcMotorEx turretMotor;
    private DcMotorEx turretEncoder;

    // Valor que acabamos de medir
    private static final double TICKS_PER_REV = 51940.0;
    private static final double TICKS_PER_DEGREE =
            TICKS_PER_REV / 360.0;

    @Override
    public void runOpMode() {

        turretMotor = hardwareMap.get(
                DcMotorEx.class,
                "turret"
        );

        turretEncoder = hardwareMap.get(
                DcMotorEx.class,
                "mfl"
        );

        turretMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        turretMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        /*
         * En la prueba anterior:
         * joystick derecha movía la torreta
         * hacia la izquierda.
         *
         * Por eso invertimos el motor.
         */
        turretMotor.setDirection(
                DcMotor.Direction.REVERSE
        );

        telemetry.addLine("=== MOTOR + ENCODER TEST ===");
        telemetry.addLine("");
        telemetry.addLine("IMPORTANTE:");
        telemetry.addLine("Pon la torreta mirando al frente.");
        telemetry.addLine("Despues presiona START.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // El lugar donde arrancamos será 0 grados
        int zeroTicks =
                turretEncoder.getCurrentPosition();

        while (opModeIsActive()) {

            // =============================
            // MOTOR
            // =============================

            double joystick =
                    gamepad2.right_stick_x;

            // Deadband
            if (Math.abs(joystick) < 0.05) {
                joystick = 0;
            }

            // Ya podemos probar hasta 80%
            double motorPower =
                    joystick * 0.80;

            turretMotor.setPower(
                    motorPower
            );

            // =============================
            // ENCODER
            // =============================

            int rawTicks =
                    turretEncoder.getCurrentPosition();

            int relativeTicks =
                    rawTicks - zeroTicks;

            /*
             * En tus pruebas el encoder contó
             * negativo en el sentido que giraste.
             *
             * Por ahora invertimos el signo
             * para obtener grados positivos.
             */
            double angle =
                    -(relativeTicks /
                            TICKS_PER_DEGREE);

            // =============================
            // TELEMETRY
            // =============================

            telemetry.addLine(
                    "=== TURRET TEST ==="
            );

            telemetry.addData(
                    "Motor Power",
                    "%.2f",
                    motorPower
            );

            telemetry.addData(
                    "Raw Encoder",
                    rawTicks
            );

            telemetry.addData(
                    "Relative Ticks",
                    relativeTicks
            );

            telemetry.addData(
                    "Turret Angle",
                    "%.2f deg",
                    angle
            );

            telemetry.update();
        }

        turretMotor.setPower(0);
    }
}
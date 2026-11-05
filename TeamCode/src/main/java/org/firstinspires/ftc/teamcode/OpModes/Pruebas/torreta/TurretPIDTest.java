package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TEST - Turret PID Calibracion")
public class TurretPIDTest extends LinearOpMode {

    private DcMotorEx turretMotor;
    private DcMotorEx turretEncoder;

    // =====================================================
    // ENCODER
    // =====================================================

    // Medido físicamente por nosotros:
    // 360 grados ≈ 51940 ticks
    private static final double TICKS_PER_REV = 51940.0;

    private static final double TICKS_PER_DEGREE =
            TICKS_PER_REV / 360.0;

    // =====================================================
    // PID
    // =====================================================

    private double kP = 0.025;
    private double kI = 0.0007;
    private double kD = 0.0007;

    private double MAX_POWER = 0.80;
    private double MIN_POWER = 0.12;
    private double targetAngle = 0.0;

    private double integral = 0.0;
    private double lastOutput = 0.0;
    private double previousError = 0.0;

    private long previousTime;

    private int zeroTicks;

    @Override
    public void runOpMode() {

        // =================================================
        // HARDWARE
        // =================================================

        turretMotor = hardwareMap.get(
                DcMotorEx.class,
                "turret"
        );

        /*
         * Tu Through Bore está conectado al puerto
         * de encoder correspondiente a "mfl".
         */
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

        // Dirección que ya comprobamos en las pruebas
        turretMotor.setDirection(
                DcMotor.Direction.REVERSE
        );

        // =================================================
        // ANTES DE START
        // =================================================

        telemetry.addLine("================================");
        telemetry.addLine("     TURRET PID CALIBRATION");
        telemetry.addLine("================================");
        telemetry.addLine("");
        telemetry.addLine("ANTES DE START:");
        telemetry.addLine("");
        telemetry.addLine("Pon la torreta EXACTAMENTE");
        telemetry.addLine("mirando hacia el frente.");
        telemetry.addLine("");
        telemetry.addLine("Luego presiona START.");
        telemetry.addLine("");
        telemetry.addLine("B = +45 grados");
        telemetry.addLine("A = 0 grados");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // =================================================
        // ZERO
        // =================================================

        zeroTicks =
                turretEncoder.getCurrentPosition();

        targetAngle = 0.0;

        previousError = 0.0;
        integral = 0.0;

        previousTime =
                System.nanoTime();

        // =================================================
        // LOOP
        // =================================================

        while (opModeIsActive()) {

            // =============================================
            // BOTONES
            // =============================================

            // SOLO vamos a utilizar estos dos por ahora.

            if (gamepad2.b) {

                targetAngle = 45.0;

                // Reiniciar PID
                integral = 0.0;
                previousError = 0.0;
            }

            if (gamepad2.a) {

                targetAngle = 0.0;

                // Reiniciar PID
                integral = 0.0;
                previousError = 0.0;
            }

            // =============================================
            // ENCODER
            // =============================================

            int rawTicks =
                    turretEncoder.getCurrentPosition();

            int relativeTicks =
                    rawTicks - zeroTicks;

            // =============================================
            // CONVERSIÓN TICKS -> GRADOS
            // =============================================

            /*
             * Nuestro encoder cuenta negativo cuando
             * la torreta gira hacia nuestro positivo.
             *
             * Por eso ponemos el signo -
             */

            double currentAngle =
                    -(relativeTicks / TICKS_PER_DEGREE);

            // =============================================
            // ERROR
            // =============================================

            double error =
                    targetAngle - currentAngle;

            // =============================================
            // TIEMPO
            // =============================================

            long now =
                    System.nanoTime();

            double dt =
                    (now - previousTime)
                            / 1_000_000_000.0;

            previousTime = now;

            if (dt <= 0) {
                continue;
            }

            // =============================================
            // PID
            // =============================================

            integral += error * dt;

            // Anti-windup
            integral =
                    clamp(
                            integral,
                            -50.0,
                            50.0
                    );

            double derivative =
                    (error - previousError) / dt;

            previousError = error;

            double pTerm =
                    kP * error;

            double iTerm =
                    kI * integral;

            double dTerm =
                    kD * derivative;

            double output =
                    pTerm
                            + iTerm
                            + dTerm;

            // =============================================
            // LIMITAR POTENCIA
            // =============================================

            output =
                    clamp(
                            output,
                            -MAX_POWER,
                            MAX_POWER
                    );


            // =============================================
// SUAVIZADO DE ACELERACION
// =============================================

            double MAX_POWER_CHANGE = 0.015;

            double change = output - lastOutput;

            change = clamp(
                    change,
                    -MAX_POWER_CHANGE,
                    MAX_POWER_CHANGE
            );

            output = lastOutput + change;

            lastOutput = output;
            // Potencia mínima para vencer la fricción
            double MIN_POWER = 0.15;

            if (Math.abs(error) >= 1.0 &&
                    Math.abs(output) < MIN_POWER) {

                output = Math.copySign(
                        MIN_POWER,
                        output
                );
            }

            // =============================================
            // TOLERANCIA
            // =============================================

            /*
             * Si estamos a menos de 1 grado,
             * dejamos de aplicar potencia.
             */

            if (Math.abs(error) < 1.0) {

                output = 0.0;

                integral = 0.0;
            }

            // =============================================
            // MOTOR
            // =============================================

            turretMotor.setPower(output);

            // =============================================
            // TELEMETRY
            // =============================================

            telemetry.addLine(
                    "========= PID ========="
            );

            telemetry.addData(
                    "Target",
                    "%.2f deg",
                    targetAngle
            );

            telemetry.addData(
                    "Actual",
                    "%.2f deg",
                    currentAngle
            );

            telemetry.addData(
                    "Error",
                    "%.2f deg",
                    error
            );

            telemetry.addData(
                    "Motor Power",
                    "%.3f",
                    output
            );

            telemetry.addLine("");

            telemetry.addLine(
                    "======= ENCODER ======="
            );

            telemetry.addData(
                    "Raw",
                    rawTicks
            );

            telemetry.addData(
                    "Zero",
                    zeroTicks
            );

            telemetry.addData(
                    "Relative",
                    relativeTicks
            );

            telemetry.addLine("");

            telemetry.addLine(
                    "===== CALIBRACION ====="
            );

            telemetry.addData(
                    "Ticks / vuelta",
                    "%.1f",
                    TICKS_PER_REV
            );

            telemetry.addData(
                    "Ticks / grado",
                    "%.4f",
                    TICKS_PER_DEGREE
            );

            telemetry.addData(
                    "Angulo calculado",
                    "%.2f deg",
                    -(relativeTicks / TICKS_PER_DEGREE)
            );

            telemetry.addLine("");

            telemetry.addLine(
                    "====== PID TERMS ======"
            );

            telemetry.addData(
                    "P",
                    "%.4f",
                    pTerm
            );

            telemetry.addData(
                    "I",
                    "%.4f",
                    iTerm
            );

            telemetry.addData(
                    "D",
                    "%.4f",
                    dTerm
            );

            telemetry.update();
        }

        turretMotor.setPower(0);
    }

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
}
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TEST - Gate Servo")
public class GateServoTest extends OpMode {

    private Servo gate;

    private double position = 0.5;

    @Override
    public void init() {

        // Nombre del servo en Driver Station
        gate = hardwareMap.get(Servo.class, "gate");

        // IMPORTANTE:
        // No usamos REVERSE ni FORWARD para esta prueba.
        // Tampoco mandamos una posición durante INIT.

        telemetry.addLine("===== GATE SERVO TEST =====");
        telemetry.addLine("");
        telemetry.addLine("A = 0.00");
        telemetry.addLine("B = 0.50");
        telemetry.addLine("X = 1.00");
        telemetry.addLine("");
        telemetry.addLine("DPAD LEFT  = -0.01");
        telemetry.addLine("DPAD RIGHT = +0.01");
        telemetry.addLine("");
        telemetry.addLine("Haz primero la prueba A/B/X");
        telemetry.update();
    }

    @Override
    public void loop() {

        // =============================================
        // POSICIONES DIRECTAS
        // =============================================

        if (gamepad1.a) {
            position = 0.00;
            gate.setPosition(position);
        }

        if (gamepad1.b) {
            position = 0.50;
            gate.setPosition(position);
        }

        if (gamepad1.x) {
            position = 1.00;
            gate.setPosition(position);
        }


        // =============================================
        // AJUSTE FINO
        // =============================================

        if (gamepad1.dpad_left) {

            position -= 0.01;

            if (position < 0.0) {
                position = 0.0;
            }

            gate.setPosition(position);

            sleepSmall();
        }

        if (gamepad1.dpad_right) {

            position += 0.01;

            if (position > 1.0) {
                position = 1.0;
            }

            gate.setPosition(position);

            sleepSmall();
        }


        // =============================================
        // TELEMETRÍA
        // =============================================

        telemetry.addLine("===== GATE SERVO TEST =====");

        telemetry.addData(
                "Position enviada",
                "%.2f",
                position
        );

        telemetry.addData(
                "Position reportada",
                "%.2f",
                gate.getPosition()
        );

        telemetry.addLine("");
        telemetry.addLine("A = 0.00");
        telemetry.addLine("B = 0.50");
        telemetry.addLine("X = 1.00");
        telemetry.addLine("");
        telemetry.addLine("DPAD LEFT/RIGHT = ajustar");

        telemetry.update();
    }


    // Solo evita que el DPAD cambie demasiado rápido
    private void sleepSmall() {

        try {
            Thread.sleep(100);
        } catch (InterruptedException ignored) {
        }
    }
}
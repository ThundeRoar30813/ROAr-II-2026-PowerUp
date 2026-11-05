package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ThunderROAR - Hood Angle Tuner")
public class HoodAngleTuner extends OpMode {

    // ==========================================
    // SERVO CAPUCHA
    // ==========================================

    private Servo hood;

    // CAMBIA ESTE NOMBRE SEGÚN TU CONFIGURACIÓN
    private static final String HOOD_NAME = "hood";

    // ==========================================
    // ÁNGULO
    // ==========================================

    private double hoodAngle = 90.0;

    private static final double MIN_ANGLE = 0.0;
    private static final double MAX_ANGLE = 180.0;

    // Cuántos grados cambia cada pulsación
    private double step = 1.0;

    // ==========================================
    // BOTONES ANTERIORES
    // ==========================================

    private boolean previousUp = false;
    private boolean previousDown = false;
    private boolean previousLeft = false;
    private boolean previousRight = false;

    @Override
    public void init() {

        hood = hardwareMap.get(
                Servo.class,
                HOOD_NAME
        );

        hood.setDirection(
                Servo.Direction.FORWARD
        );

        // No mover durante INIT

        telemetry.addLine("===== HOOD ANGLE TUNER =====");
        telemetry.addLine("DPAD UP    = + grados");
        telemetry.addLine("DPAD DOWN  = - grados");
        telemetry.addLine("DPAD RIGHT = paso +");
        telemetry.addLine("DPAD LEFT  = paso -");
        telemetry.addLine("");
        telemetry.addLine("A = mandar angulo actual");
        telemetry.addLine("");
        telemetry.addLine("Inicia con cuidado");
        telemetry.update();
    }

    @Override
    public void loop() {

        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;

        // ==========================================
        // AUMENTAR ÁNGULO
        // ==========================================

        if (up && !previousUp) {

            hoodAngle += step;

            setHoodAngle(hoodAngle);
        }

        // ==========================================
        // DISMINUIR ÁNGULO
        // ==========================================

        if (down && !previousDown) {

            hoodAngle -= step;

            setHoodAngle(hoodAngle);
        }

        // ==========================================
        // AUMENTAR PASO
        // ==========================================

        if (right && !previousRight) {

            step = Math.min(20.0, step + 1.0);
        }

        // ==========================================
        // DISMINUIR PASO
        // ==========================================

        if (left && !previousLeft) {

            step = Math.max(1.0, step - 1.0);
        }

        previousUp = up;
        previousDown = down;
        previousLeft = left;
        previousRight = right;

        // ==========================================
        // A = APLICAR ÁNGULO
        // ==========================================

        if (gamepad1.a) {

            setHoodAngle(hoodAngle);
        }

        // ==========================================
        // TELEMETRÍA
        // ==========================================

        telemetry.addLine("===== HOOD CALIBRATION =====");

        telemetry.addData(
                "Hood Angle",
                "%.1f grados",
                hoodAngle
        );

        telemetry.addData(
                "Servo Position",
                "%.4f",
                hoodAngle / 180.0
        );

        telemetry.addData(
                "Step",
                "%.1f grados",
                step
        );

        telemetry.addLine("");
        telemetry.addLine("UP/DOWN = mover capucha");
        telemetry.addLine("LEFT/RIGHT = cambiar paso");
        telemetry.addLine("A = aplicar posicion");

        telemetry.update();
    }

    // ==========================================
    // CONTROL DEL SERVO
    // ==========================================

    private void setHoodAngle(double angle) {

        angle = Math.max(
                MIN_ANGLE,
                Math.min(MAX_ANGLE, angle)
        );

        hoodAngle = angle;

        double position = angle / 180.0;

        hood.setPosition(position);
    }
}
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ThunderROAR - Hood Positions Test")
public class HoodPositionsTest extends OpMode {

    // ==========================================
    // SERVO CAPUCHA
    // ==========================================

    private Servo hood;

    // Cambiar si el nombre es diferente
    private static final String HOOD_NAME = "hood";

    // ==========================================
    // POSICIONES CALIBRADAS
    // ==========================================

    private static final double[] HOOD_POSITIONS = {
            0.4444,  // 1 - ABAJO
            0.5560,  // 2 - BAJA
            0.6667,  // 3 - MEDIA
            0.7778,  // 4 - ALTA
            0.8890   // 5 - ARRIBA
    };

    private static final String[] POSITION_NAMES = {
            "ABAJO",
            "BAJA",
            "MEDIA",
            "ALTA",
            "ARRIBA"
    };

    // Iniciar en posición media
    private int currentPosition = 2;

    // ==========================================
    // BOTONES
    // ==========================================

    private boolean previousUp = false;
    private boolean previousDown = false;

    // ==========================================
    // INIT
    // ==========================================

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

        telemetry.addLine("===== HOOD POSITIONS TEST =====");
        telemetry.addLine("DPAD UP = SUBIR POSICION");
        telemetry.addLine("DPAD DOWN = BAJAR POSICION");
        telemetry.addLine("");
        telemetry.addLine("A = APLICAR POSICION ACTUAL");
        telemetry.addLine("");
        telemetry.addLine("Presiona START para comenzar");
        telemetry.update();
    }

    // ==========================================
    // START
    // ==========================================

    @Override
    public void start() {

        // Iniciar en posición media
        setHoodPosition(currentPosition);
    }

    // ==========================================
    // LOOP
    // ==========================================

    @Override
    public void loop() {

        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        // ==========================================
        // SUBIR UNA POSICION
        // ==========================================

        if (up && !previousUp) {

            if (currentPosition < HOOD_POSITIONS.length - 1) {

                currentPosition++;

                setHoodPosition(currentPosition);
            }
        }

        // ==========================================
        // BAJAR UNA POSICION
        // ==========================================

        if (down && !previousDown) {

            if (currentPosition > 0) {

                currentPosition--;

                setHoodPosition(currentPosition);
            }
        }

        previousUp = up;
        previousDown = down;

        // ==========================================
        // A = APLICAR POSICION
        // ==========================================

        if (gamepad1.a) {

            setHoodPosition(currentPosition);
        }

        // ==========================================
        // TELEMETRY
        // ==========================================

        telemetry.addLine("===== HOOD POSITIONS =====");

        telemetry.addData(
                "Posicion",
                "%d / %d",
                currentPosition + 1,
                HOOD_POSITIONS.length
        );

        telemetry.addData(
                "Nombre",
                POSITION_NAMES[currentPosition]
        );

        telemetry.addData(
                "Servo Position",
                "%.4f",
                HOOD_POSITIONS[currentPosition]
        );

        telemetry.addLine("");

        telemetry.addLine("DPAD UP = SUBIR");
        telemetry.addLine("DPAD DOWN = BAJAR");
        telemetry.addLine("A = APLICAR");

        telemetry.update();
    }

    // ==========================================
    // MOVER CAPUCHA
    // ==========================================

    private void setHoodPosition(int index) {

        index = Math.max(
                0,
                Math.min(HOOD_POSITIONS.length - 1, index)
        );

        currentPosition = index;

        hood.setPosition(
                HOOD_POSITIONS[currentPosition]
        );
    }
}
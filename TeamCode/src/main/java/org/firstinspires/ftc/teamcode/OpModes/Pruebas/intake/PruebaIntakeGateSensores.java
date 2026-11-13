package org.firstinspires.ftc.teamcode.OpModes.Pruebas.intake;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Prueba Intake Gate Sensores")
public class PruebaIntakeGateSensores extends OpMode {

    // =========================================
    // MOTORES Y SERVO
    // =========================================
    private DcMotorEx shooter;
    private DcMotor intake;
    private DcMotor indexer;
    private Servo gate;

    private static final double GATE_CLOSED = 90.0 / 180.0;
    private static final double GATE_OPEN = 20.0 / 180.0;
    private boolean gateOpen = false;

    // =========================================
    // SENSORES DE COLOR / DISTANCIA REV
    // =========================================
    private ColorSensor colorBottom;       // REV V2 (Entrada Intake)
    private ColorSensor colorTop;          // REV V3 (Tope Shooter)
    private DistanceSensor distBottom;     // Sensor de proximidad integrado V2
    private DistanceSensor distTop;        // Sensor de proximidad integrado V3

    // Umbrales de calibracion (ajustar segun telemetria)
    private static final double BOTTOM_DIST_CM = 4.5; // Detectar pieza si esta a menos de 4.5 cm
    private static final double TOP_DIST_CM = 5.0;    // Detectar pieza si esta a menos de 5.0 cm
    private static final int BOTTOM_ALPHA_MIN = 150;  // Respaldo por luz/color
    private static final int TOP_ALPHA_MIN = 250;     // Respaldo por luz/color

    private int ballCount = 0;
    private boolean bottomDetected = false;
    private boolean topDetected = false;
    private boolean previousBottomDetected = false;
    private long lastBottomCountTime = 0;

    // =========================================
    // ESTADOS DE CONTROL
    // =========================================
    private boolean autoIntakeEnabled = false;
    private boolean lastAutoIntakeBtn = false;
    private boolean lastResetCountBtn = false;
    private boolean wasShooting = false;

    @Override
    public void init() {
        // SHOOTER
        shooter = hardwareMap.get(DcMotorEx.class, "shoot");
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Potencia directa para prueba al maximo
        shooter.setPower(0);

        // INTAKE E INDEXER
        intake = hardwareMap.get(DcMotor.class, "intake2");
        indexer = hardwareMap.get(DcMotor.class, "intake1");

        intake.setDirection(DcMotor.Direction.FORWARD);
        indexer.setDirection(DcMotor.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setPower(0);
        indexer.setPower(0);

        // GATE
        gate = hardwareMap.tryGet(Servo.class, "gate");
        if (gate != null) {
            gate.setDirection(Servo.Direction.REVERSE);
        }
        setGate(false);

        // SENSORES REV (V2 abajo, V3 arriba)
        colorBottom = hardwareMap.tryGet(ColorSensor.class, "colorBottom");
        colorTop = hardwareMap.tryGet(ColorSensor.class, "colorTop");

        if (colorBottom instanceof DistanceSensor) {
            distBottom = (DistanceSensor) colorBottom;
        }
        if (colorTop instanceof DistanceSensor) {
            distTop = (DistanceSensor) colorTop;
        }

        telemetry.addLine("PRUEBA INTAKE / GATE / SENSORES LISTA");
        telemetry.addLine("LB: Activar/Desactivar Carga Automatica");
        telemetry.addLine("X (Mantener): Disparar TODO al maximo");
        telemetry.addLine("B (Mantener): Reversa manual");
        telemetry.addLine("Y: Reiniciar contador a 0");
        telemetry.update();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();

        // 1. LEER SENSORES
        double bottomCm = (distBottom != null) ? distBottom.getDistance(DistanceUnit.CM) : 999.0;
        double topCm = (distTop != null) ? distTop.getDistance(DistanceUnit.CM) : 999.0;

        int bottomAlpha = (colorBottom != null) ? colorBottom.alpha() : 0;
        int topAlpha = (colorTop != null) ? colorTop.alpha() : 0;

        // Detecta si cumple distancia O si el alpha sube por encima del umbral
        bottomDetected = (colorBottom != null) && (bottomCm < BOTTOM_DIST_CM || bottomAlpha >= BOTTOM_ALPHA_MIN);
        topDetected = (colorTop != null) && (topCm < TOP_DIST_CM || topAlpha >= TOP_ALPHA_MIN);

        // 2. CONTAR PIEZAS EN EL SENSOR DE ABAJO (Flanco de subida + antirrebote de 180 ms)
        if (autoIntakeEnabled && !gamepad1.x) {
            if (bottomDetected && !previousBottomDetected && (now - lastBottomCountTime > 180_000_000L)) {
                ballCount++;
                lastBottomCountTime = now;
            }
        }
        previousBottomDetected = bottomDetected;

        // 3. CONTROLES DE BOTONES
        // Boton LB: Alternar modo de carga automatica
        if (gamepad1.left_bumper && !lastAutoIntakeBtn) {
            autoIntakeEnabled = !autoIntakeEnabled;
        }
        lastAutoIntakeBtn = gamepad1.left_bumper;

        // Boton Y: Reiniciar contador manualmente
        if (gamepad1.y && !lastResetCountBtn) {
            ballCount = 0;
        }
        lastResetCountBtn = gamepad1.y;

        // 4. MAQUINA DE ESTADOS DE MOTORES Y GATE
        double intakePower = 0;
        double indexerPower = 0;
        double shooterPower = 0;

        boolean shootBtn = gamepad1.x;    // Boton independiente para disparar al maximo
        boolean reverseBtn = gamepad1.b;  // Boton de reversa por si se atasca un pollen

        if (shootBtn) {
            // PRIORIDAD 1: DISPARO AL MAXIMO (Sin importar cuantas piezas haya)
            autoIntakeEnabled = false;
            wasShooting = true;

            setGate(true);          // Abrir gate
            shooterPower = 1.0;     // Shooter al maximo
            intakePower = 1.0;      // Intake al maximo
            indexerPower = 1.0;     // Indexer al maximo

        } else {
            // Si acabamos de soltar el boton de disparo, reiniciamos el contador a 0
            if (wasShooting) {
                ballCount = 0;
                wasShooting = false;
            }

            if (reverseBtn) {
                // PRIORIDAD 2: REVERSA MANUAL
                setGate(false);
                intakePower = -1.0;
                indexerPower = -1.0;
                shooterPower = 0.0;

            } else if (autoIntakeEnabled) {
                // PRIORIDAD 3: CARGA AUTOMATICA
                setGate(false); // Activar/mantener cerrada la gate

                if (ballCount >= 4) {
                    // Ya pasaron 4 piezas abajo: detener todo y apagar modo auto
                    intakePower = 0.0;
                    indexerPower = 0.0;
                    autoIntakeEnabled = false;
                } else {
                    // Menos de 4 piezas: el intake sigue girando
                    intakePower = 1.0;

                    // Si el sensor de arriba (V3) detecta pieza, frena el indexer; si no, avanza
                    if (topDetected) {
                        indexerPower = 0.0;
                    } else {
                        indexerPower = 1.0;
                    }
                }

            } else {
                // REPOSO
                setGate(false);
                intakePower = 0.0;
                indexerPower = 0.0;
                shooterPower = 0.0;
            }
        }

        // Aplicar potencias
        intake.setPower(intakePower);
        indexer.setPower(indexerPower);
        shooter.setPower(shooterPower);

        // 5. TELEMETRIA DE CALIBRACION Y ESTADO
        telemetry.addData("Modo Auto Intake (LB)", autoIntakeEnabled ? "ACTIVO" : "APAGADO");
        telemetry.addData("Piezas Contadas (Abajo)", "%d / 4", ballCount);
        telemetry.addData("Gate", gateOpen ? "ABIERTA" : "CERRADA");
        telemetry.addLine("------------------------------");
        telemetry.addData("Sensor Abajo (V2)", "%s | Dist: %.2f cm | Alpha: %d",
                bottomDetected ? "DETECTADO" : "---", bottomCm, bottomAlpha);
        telemetry.addData("Sensor Arriba (V3)", "%s | Dist: %.2f cm | Alpha: %d",
                topDetected ? "DETECTADO" : "---", topCm, topAlpha);
        telemetry.addLine("------------------------------");
        telemetry.addData("Potencias (In / Idx / Shoot)", "%.1f / %.1f / %.1f",
                intakePower, indexerPower, shooterPower);
        telemetry.update();
    }

    private void setGate(boolean open) {
        gateOpen = open;
        if (gate != null) {
            gate.setPosition(open ? GATE_OPEN : GATE_CLOSED);
        }
    }

    @Override
    public void stop() {
        if (shooter != null) shooter.setPower(0);
        if (intake != null) intake.setPower(0);
        if (indexer != null) indexer.setPower(0);
        setGate(false);
    }
}
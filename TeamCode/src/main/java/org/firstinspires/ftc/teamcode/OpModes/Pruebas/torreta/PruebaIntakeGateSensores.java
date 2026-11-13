package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Prueba Intake Gate Sensores primeeeeeee")
public class PruebaIntakeGateSensores extends OpMode {

    // =========================================
    // MOTORES Y SERVO
    // =========================================
    private DcMotorEx shooter;
    private DcMotor intake;
    private DcMotor indexer;
    private Servo gate;

    private static final double GATE_CLOSED = 40.0 / 180.0;
    private static final double GATE_OPEN = 90.0 / 180.0;
    private boolean gateOpen = false;

    // =========================================
    // SENSORES DE COLOR / DISTANCIA REV
    // =========================================
    private ColorSensor colorBottom;       // REV V2 (Entrada Intake)
    private ColorSensor colorTop;          // REV V3 (Tope Shooter)
    private DistanceSensor distBottom;     // Sensor de proximidad integrado V2
    private DistanceSensor distTop;        // Sensor de proximidad integrado V3

    // Umbrales calibrados
    private static final double BOTTOM_DIST_CM = 2.0;
    private static final double TOP_DIST_CM = 2.0;
    private static final int BOTTOM_ALPHA_MIN = 350;
    private static final int TOP_ALPHA_MIN = 350;

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
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setPower(0);

        // INTAKE E INDEXER (Corregidos: intake1 es el intake, intake2 es el indexer)
        intake = hardwareMap.get(DcMotor.class, "intake1");
        indexer = hardwareMap.get(DcMotor.class, "intake2");

        intake.setDirection(DcMotor.Direction.REVERSE);
        indexer.setDirection(DcMotor.Direction.FORWARD);

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
        colorBottom = hardwareMap.tryGet(ColorSensor.class, "colorV2");
        colorTop = hardwareMap.tryGet(ColorSensor.class, "colorV3");

        if (colorBottom instanceof DistanceSensor) {
            distBottom = (DistanceSensor) colorBottom;
        }
        if (colorTop instanceof DistanceSensor) {
            distTop = (DistanceSensor) colorTop;
        }

        telemetry.addLine("PRUEBA INTAKE / GATE / SENSORES LISTA");
        telemetry.addLine("LB: Activar/Desactivar Carga Automatica");
        telemetry.addLine("B (Mantener): Meter (Intake 100%, Indexer 50%, para si detecta arriba)");
        telemetry.addLine("X (Mantener): Abrir Gate + Disparar al 100%");
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

        bottomDetected = (colorBottom != null) && (bottomCm < BOTTOM_DIST_CM || bottomAlpha >= BOTTOM_ALPHA_MIN);
        topDetected = (colorTop != null) && (topCm < TOP_DIST_CM || topAlpha >= TOP_ALPHA_MIN);

        // 2. CONTAR PIEZAS EN EL SENSOR DE ABAJO
        if (!gamepad1.x) {
            if (bottomDetected && !previousBottomDetected && (now - lastBottomCountTime > 180_000_000L)) {
                ballCount++;
                lastBottomCountTime = now;
            }
        }
        previousBottomDetected = bottomDetected;

        // 3. CONTROLES DE BOTONES
        if (gamepad1.left_bumper && !lastAutoIntakeBtn) {
            if (!autoIntakeEnabled && ballCount >= 4) {
                ballCount = 0;
            }
            autoIntakeEnabled = !autoIntakeEnabled;
        }
        lastAutoIntakeBtn = gamepad1.left_bumper;

        if (gamepad1.y && !lastResetCountBtn) {
            ballCount = 0;
        }
        lastResetCountBtn = gamepad1.y;

        // 4. MAQUINA DE ESTADOS DE MOTORES Y GATE
        double intakePower = 0;
        double indexerPower = 0;
        double shooterPower = 0;

        boolean shootBtn = gamepad1.x;
        boolean manualIntakeBtn = gamepad1.b;

        if (shootBtn) {
            // PRIORIDAD 1: DISPARO AL 100% (Gate abierta + todo al maximo)
            autoIntakeEnabled = false;
            wasShooting = true;

            setGate(true);          // Abrir gate
            shooterPower = 1.0;     // Shooter al 100%
            intakePower = 1.0;      // Intake al 100%
            indexerPower = 1.0;     // Indexer al 100%

        } else {
            if (wasShooting) {
                ballCount = 0;
                wasShooting = false;
            }

            if (manualIntakeBtn) {
                // PRIORIDAD 2: METER CON B (Intake 100% hacia adentro, Indexer 50% hacia adentro)
                setGate(false);         // Gate cerrada
                shooterPower = 0.0;
                intakePower = 1.0;      // Intake al 100% hacia adentro

                if (topDetected) {
                    indexerPower = 0.0; // Si detecta pieza arriba, frena el indexer
                } else {
                    indexerPower = 0.5; // Indexer al 50% hacia adentro
                }

            } else if (autoIntakeEnabled) {
                // PRIORIDAD 3: CARGA AUTOMATICA CON LB
                setGate(false);

                if (ballCount >= 4) {
                    intakePower = 0.0;
                    indexerPower = 0.0;
                    autoIntakeEnabled = false;
                } else {
                    intakePower = 1.0;
                    indexerPower = topDetected ? 0.0 : 0.5;
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

        // 5. TELEMETRIA
        telemetry.addData("Modo Auto Intake (LB)", autoIntakeEnabled ? "ACTIVO" : "APAGADO");
        telemetry.addData("Piezas Contadas (Abajo)", "%d / 4", ballCount);
        telemetry.addData("Gate", gateOpen ? "ABIERTA" : "CERRADA");
        telemetry.addLine("------------------------------");
        telemetry.addData("Sensor Abajo (V2)", "%s | Dist: %.2f cm | Alpha: %d",
                bottomDetected ? "DETECTADO" : "---", bottomCm, bottomAlpha);
        telemetry.addData("Sensor Arriba (V3)", "%s | Dist: %.2f cm | Alpha: %d",
                topDetected ? "DETECTADO" : "---", topCm, topAlpha);
        telemetry.addLine("------------------------------");
        telemetry.addData("Potencias (In / Idx / Shoot)", "%.2f / %.2f / %.2f",
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
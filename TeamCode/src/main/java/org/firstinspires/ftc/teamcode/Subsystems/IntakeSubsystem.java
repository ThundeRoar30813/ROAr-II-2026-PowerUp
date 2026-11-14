
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem {

    // =========================================
    // HARDWARE
    // =========================================

    private final DcMotor intake;
    private final DcMotor indexer;
    private final Servo gate;

    private final ColorSensor colorBottom;
    private final ColorSensor colorTop;

    private final DistanceSensor distBottom;
    private final DistanceSensor distTop;

    // =========================================
    // CONFIGURACION
    // =========================================

    private static final double GATE_CLOSED = 40.0 / 180.0;
    private static final double GATE_OPEN = 90.0 / 180.0;

    private static final double BOTTOM_DIST_CM = 2.0;
    private static final double TOP_DIST_CM = 2.0;

    private static final int BOTTOM_ALPHA_MIN = 350;
    private static final int TOP_ALPHA_MIN = 350;

    private static final int MAX_BALLS = 4;

    private static final double INTAKE_POWER = 1.0;
    private static final double INDEXER_LOAD_POWER = 0.5;
    private static final double INDEXER_FEED_POWER = 1.0;

    // 0.5 segundos
    private static final long TOP_STOP_DELAY_NS = 500_000_000L;

    // =========================================
    // ESTADOS
    // =========================================

    private int ballCount = 0;

    private boolean autoIntakeEnabled = false;
    private boolean gateOpen = false;

    private boolean bottomDetected = false;
    private boolean topDetected = false;
    private boolean previousBottomDetected = false;

    private boolean wasFeeding = false;

    private long lastBottomCountTime = 0;

    // Temporizador del sensor superior
    private long topDetectionStartNs = 0;
    private boolean topStopDelayFinished = false;

    // Telemetria
    private double bottomCm = 999;
    private double topCm = 999;

    private int bottomAlpha = 0;
    private int topAlpha = 0;

    private double intakePower = 0;
    private double indexerPower = 0;

    // =========================================
    // CONSTRUCTOR
    // =========================================

    public IntakeSubsystem(HardwareMap hardwareMap) {

        // MOTORES
        intake = hardwareMap.get(
                DcMotor.class, "intake1"
        );

        indexer = hardwareMap.get(
                DcMotor.class, "intake2"
        );

        intake.setDirection(DcMotor.Direction.REVERSE);
        indexer.setDirection(DcMotor.Direction.FORWARD);

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        indexer.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        intake.setPower(0);
        indexer.setPower(0);

        // GATE
        gate = hardwareMap.tryGet(
                Servo.class, "gate"
        );

        if (gate != null) {
            gate.setDirection(Servo.Direction.REVERSE);
        }

        setGate(false);

        // SENSORES
        colorBottom = hardwareMap.tryGet(
                ColorSensor.class, "colorV2"
        );

        colorTop = hardwareMap.tryGet(
                ColorSensor.class, "colorV3"
        );

        distBottom =
                colorBottom instanceof DistanceSensor
                        ? (DistanceSensor) colorBottom
                        : null;

        distTop =
                colorTop instanceof DistanceSensor
                        ? (DistanceSensor) colorTop
                        : null;
    }

    // =========================================
    // ACTUALIZAR SENSORES
    // =========================================

    private void updateSensors(boolean feeding) {

        long now = System.nanoTime();

        bottomCm = distBottom != null
                ? distBottom.getDistance(DistanceUnit.CM)
                : 999.0;

        topCm = distTop != null
                ? distTop.getDistance(DistanceUnit.CM)
                : 999.0;

        bottomAlpha = colorBottom != null
                ? colorBottom.alpha()
                : 0;

        topAlpha = colorTop != null
                ? colorTop.alpha()
                : 0;

        bottomDetected =
                colorBottom != null &&
                        (bottomCm < BOTTOM_DIST_CM ||
                                bottomAlpha >= BOTTOM_ALPHA_MIN);

        topDetected =
                colorTop != null &&
                        (topCm < TOP_DIST_CM ||
                                topAlpha >= TOP_ALPHA_MIN);

        // CONTADOR POR FLANCO ASCENDENTE
        if (!feeding) {

            if (bottomDetected &&
                    !previousBottomDetected &&
                    now - lastBottomCountTime > 180_000_000L) {

                ballCount = Math.min(
                        MAX_BALLS,
                        ballCount + 1
                );

                lastBottomCountTime = now;
            }
        }

        previousBottomDetected = bottomDetected;
    }

    // =========================================
    // TEMPORIZADOR DEL INDEXER
    // =========================================

    private double getLoadingIndexerPower() {

        long now = System.nanoTime();

        // Si ya terminó la espera, mantener parado.
        if (topStopDelayFinished) {
            return 0.0;
        }

        // Primera deteccion de pelota arriba.
        if (topDetected && topDetectionStartNs == 0) {
            topDetectionStartNs = now;
        }

        // Una vez detectada, esperar 0.5 segundos
        // aunque el sensor deje de detectar.
        if (topDetectionStartNs != 0) {

            if (now - topDetectionStartNs >=
                    TOP_STOP_DELAY_NS) {

                topStopDelayFinished = true;
                return 0.0;
            }
        }

        return INDEXER_LOAD_POWER;
    }

    private void resetTopDelay() {
        topDetectionStartNs = 0;
        topStopDelayFinished = false;
    }

    // =========================================
    // CONTROL PRINCIPAL
    // =========================================

    public void update(
            boolean autoToggle,
            boolean manualIntake,
            boolean feed,
            boolean resetCount) {

        updateSensors(feed);

        intakePower = 0;
        indexerPower = 0;

        // =====================================
        // PRIORIDAD 1: ALIMENTAR PARA DISPARAR
        // =====================================

        if (feed) {

            autoIntakeEnabled = false;
            wasFeeding = true;

            resetTopDelay();

            setGate(true);

            intakePower = INTAKE_POWER;
            indexerPower = INDEXER_FEED_POWER;

        } else {

            // Termino de alimentar
            if (wasFeeding) {

                ballCount = 0;
                wasFeeding = false;

                resetTopDelay();
            }

            // =================================
            // PRIORIDAD 2: CARGA MANUAL
            // =================================

            if (manualIntake) {

                setGate(false);

                intakePower = INTAKE_POWER;
                indexerPower = getLoadingIndexerPower();

            }

            // =================================
            // PRIORIDAD 3: CARGA AUTOMATICA
            // =================================

            else if (autoIntakeEnabled) {

                setGate(false);

                if (ballCount >= MAX_BALLS) {

                    intakePower = 0;
                    indexerPower = 0;

                    autoIntakeEnabled = false;

                } else {

                    intakePower = INTAKE_POWER;
                    indexerPower = getLoadingIndexerPower();
                }
            }

            // =================================
            // REPOSO
            // =================================

            else {

                setGate(false);

                intakePower = 0;
                indexerPower = 0;
            }
        }

        // Aplicar potencias
        intake.setPower(intakePower);
        indexer.setPower(indexerPower);
    }

    // =========================================
    // AUTO INTAKE
    // =========================================

    public void toggleAutoIntake() {

        if (!autoIntakeEnabled) {

            if (ballCount >= MAX_BALLS) {
                ballCount = 0;
            }

            resetTopDelay();
        }

        autoIntakeEnabled = !autoIntakeEnabled;
    }

    // =========================================
    // REINICIAR CONTADOR
    // =========================================

    public void resetCount() {
        ballCount = 0;
        resetTopDelay();
    }

    // =========================================
    // GATE
    // =========================================

    public void setGate(boolean open) {

        gateOpen = open;

        if (gate != null) {

            gate.setPosition(
                    open ? GATE_OPEN : GATE_CLOSED
            );
        }
    }

    // =========================================
    // GETTERS
    // =========================================

    public boolean isAutoIntakeEnabled() {
        return autoIntakeEnabled;
    }

    public int getBallCount() {
        return ballCount;
    }

    public boolean isGateOpen() {
        return gateOpen;
    }

    public boolean isBottomDetected() {
        return bottomDetected;
    }

    public boolean isTopDetected() {
        return topDetected;
    }

    public double getBottomCm() {
        return bottomCm;
    }

    public double getTopCm() {
        return topCm;
    }

    public int getBottomAlpha() {
        return bottomAlpha;
    }

    public int getTopAlpha() {
        return topAlpha;
    }

    public double getIntakePower() {
        return intakePower;
    }

    public double getIndexerPower() {
        return indexerPower;
    }

    public boolean isTopStopDelayFinished() {
        return topStopDelayFinished;
    }

    public double getTopDelayRemainingSeconds() {

        if (topStopDelayFinished) {
            return 0.0;
        }

        if (topDetectionStartNs == 0) {
            return 0.5;
        }

        double elapsed =
                (System.nanoTime() - topDetectionStartNs)
                        / 1_000_000_000.0;

        return Math.max(0.0, 0.5 - elapsed);
    }

    // =========================================
    // STOP
    // =========================================

    public void stop() {

        autoIntakeEnabled = false;

        intakePower = 0;
        indexerPower = 0;

        intake.setPower(0);
        indexer.setPower(0);

        resetTopDelay();
        setGate(false);
    }
}

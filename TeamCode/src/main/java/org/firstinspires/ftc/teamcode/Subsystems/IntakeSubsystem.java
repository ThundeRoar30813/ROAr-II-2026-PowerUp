
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem {

    private final DcMotor intake;
    private final DcMotor indexer;
    private final Servo gate;

    private final ColorSensor colorBottom;
    private final ColorSensor colorTop;

    private final DistanceSensor distBottom;
    private final DistanceSensor distTop;

    private static final double GATE_CLOSED = 40.0 / 180.0;
    private static final double GATE_OPEN = 90.0 / 180.0;

    private static final double BOTTOM_DIST_CM = 2.0;
    private static final double TOP_DIST_CM = 2.0;

    private static final int BOTTOM_ALPHA_MIN = 350;
    private static final int TOP_ALPHA_MIN = 350;

    private static final int MAX_BALLS = 4;

    private int ballCount = 0;

    private boolean autoIntakeEnabled = false;
    private boolean gateOpen = false;

    private boolean bottomDetected = false;
    private boolean topDetected = false;
    private boolean previousBottomDetected = false;

    private boolean wasFeeding = false;

    private long lastBottomCountTime = 0;

    private double bottomCm = 999;
    private double topCm = 999;

    private int bottomAlpha = 0;
    private int topAlpha = 0;

    private double intakePower = 0;
    private double indexerPower = 0;

    public IntakeSubsystem(HardwareMap hardwareMap) {

        // MOTORES: EXACTAMENTE COMO TU PRUEBA
        intake = hardwareMap.get(DcMotor.class, "intake1");
        indexer = hardwareMap.get(DcMotor.class, "intake2");

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
        gate = hardwareMap.tryGet(Servo.class, "gate");

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

        distBottom = colorBottom instanceof DistanceSensor
                ? (DistanceSensor) colorBottom : null;

        distTop = colorTop instanceof DistanceSensor
                ? (DistanceSensor) colorTop : null;
    }

    // =========================================
    // SENSORES Y CONTADOR
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
                ? colorBottom.alpha() : 0;

        topAlpha = colorTop != null
                ? colorTop.alpha() : 0;

        bottomDetected = colorBottom != null &&
                (bottomCm < BOTTOM_DIST_CM ||
                        bottomAlpha >= BOTTOM_ALPHA_MIN);

        topDetected = colorTop != null &&
                (topCm < TOP_DIST_CM ||
                        topAlpha >= TOP_ALPHA_MIN);

        // CONTADOR POR FLANCO ASCENDENTE
        if (!feeding) {

            if (bottomDetected &&
                    !previousBottomDetected &&
                    now - lastBottomCountTime > 180_000_000L) {

                ballCount = Math.min(
                        MAX_BALLS, ballCount + 1
                );

                lastBottomCountTime = now;
            }
        }

        previousBottomDetected = bottomDetected;
    }

    // =========================================
    // CONTROL PRINCIPAL
    // =========================================

    public void update(
            boolean autoToggle,
            boolean manualIntake,
            boolean feed,
            boolean resetCount) {

        // Estos botones se procesan por IntakeCommand.
        // autoToggle y resetCount se mantienen en la
        // firma para compatibilidad con el Command.

        updateSensors(feed);

        intakePower = 0;
        indexerPower = 0;

        if (feed) {

            // DISPARO: GATE ABIERTA
            autoIntakeEnabled = false;
            wasFeeding = true;

            setGate(true);

            intakePower = 1.0;
            indexerPower = 1.0;

        } else {

            if (wasFeeding) {
                ballCount = 0;
                wasFeeding = false;
            }

            if (manualIntake) {

                // CARGA MANUAL
                setGate(false);

                intakePower = 1.0;
                indexerPower = topDetected ? 0.0 : 0.5;

            } else if (autoIntakeEnabled) {

                // CARGA AUTOMATICA
                setGate(false);

                if (ballCount >= MAX_BALLS) {

                    intakePower = 0;
                    indexerPower = 0;
                    autoIntakeEnabled = false;

                } else {

                    intakePower = 1.0;
                    indexerPower = topDetected ? 0.0 : 0.5;
                }

            } else {

                // REPOSO
                setGate(false);

                intakePower = 0;
                indexerPower = 0;
            }
        }

        intake.setPower(intakePower);
        indexer.setPower(indexerPower);
    }

    public void toggleAutoIntake() {

        if (!autoIntakeEnabled && ballCount >= MAX_BALLS) {
            ballCount = 0;
        }

        autoIntakeEnabled = !autoIntakeEnabled;
    }

    public void resetCount() {
        ballCount = 0;
    }

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

    public void stop() {

        autoIntakeEnabled = false;

        intakePower = 0;
        indexerPower = 0;

        intake.setPower(0);
        indexer.setPower(0);

        setGate(false);
    }
}

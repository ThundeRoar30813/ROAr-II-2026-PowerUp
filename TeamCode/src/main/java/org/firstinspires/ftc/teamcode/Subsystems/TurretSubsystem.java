
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class TurretSubsystem {

    private final DcMotorEx turretMotor;
    private final DcMotorEx turretEncoder;

    // ENCODER
    private static final double TICKS_PER_DEGREE = 51940.0 / 360.0;

    // Mantener el mismo signo que funciono en tu PID Tuner
    private static final double ENCODER_SIGN = 1.0;

    // PID CALIBRADO
    private static final double KP = 0.117;
    private static final double KD = 0.0014;
    private static final double KV = 0.0;
    private static final double TOLERANCE_DEG = 1.0;

    // LIMITES ANGULARES
    public static final double MIN_ANGLE_DEG = -110.0;
    public static final double MAX_ANGLE_DEG = 230.0;

    // POTENCIA INICIAL DE PRUEBA
    private double maxPower = 0.20;

    private int encoderZero;

    private double targetAngleDeg;
    private double targetVelocityDegPerSec;

    private double previousAngle;
    private double filteredVelocity;
    private long previousTime;

    private double lastOutput;
    private double lastPID;
    private double lastFeedforward;

    private boolean controlEnabled;
    private boolean stalled;

    private long stallStartNs;
    private double stallStartAngle;

    // PROTECCION SI NO SE DETECTA MOVIMIENTO
    private static final double STALL_POWER_THRESHOLD = 0.15;
    private static final double STALL_ERROR_THRESHOLD_DEG = 5.0;
    private static final double STALL_MOVEMENT_DEG = 1.0;
    private static final long STALL_TIMEOUT_NS = 500_000_000L;

    // =====================================================
    // CONSTRUCTOR
    // =====================================================

    public TurretSubsystem(HardwareMap hardwareMap) {

        turretMotor = hardwareMap.get(
                DcMotorEx.class, "turret"
        );

        // Encoder REV conectado al puerto de mfl
        turretEncoder = hardwareMap.get(
                DcMotorEx.class, "mfl"
        );

        turretMotor.setDirection(
                DcMotor.Direction.REVERSE
        );

        turretMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        turretMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        turretMotor.setPower(0);

        // Cero inicial
        encoderZero = turretEncoder.getCurrentPosition();

        targetAngleDeg = getCurrentAngleDeg();

        resetControllerState();
    }

    // =====================================================
    // REINICIAR ESTADO DEL CONTROLADOR
    // =====================================================

    private void resetControllerState() {

        previousAngle = getCurrentAngleDeg();
        filteredVelocity = 0;

        previousTime = System.nanoTime();

        lastOutput = 0;
        lastPID = 0;
        lastFeedforward = 0;

        stallStartNs = 0;
    }

    // =====================================================
    // ACTIVAR / DESACTIVAR
    // =====================================================

    public void enableControl() {

        targetAngleDeg = getCurrentAngleDeg();
        targetVelocityDegPerSec = 0;

        stalled = false;

        resetControllerState();

        controlEnabled = true;
    }

    public void disableControl() {

        controlEnabled = false;

        turretMotor.setPower(0);

        targetVelocityDegPerSec = 0;

        resetControllerState();
    }

    public boolean isControlEnabled() {
        return controlEnabled;
    }

    public boolean isStalled() {
        return stalled;
    }

    // =====================================================
    // POTENCIA
    // =====================================================

    public void setMaxPower(double power) {

        if (Double.isFinite(power)) {
            maxPower = Range.clip(power, 0, 0.90);
        }
    }

    public double getMaxPower() {
        return maxPower;
    }

    // =====================================================
    // PID UPDATE
    // =====================================================

    public void update() {

        if (!controlEnabled) {

            turretMotor.setPower(0);
            lastOutput = 0;
            return;
        }

        double currentAngle = getCurrentAngleDeg();

        if (!Double.isFinite(currentAngle)
                || !Double.isFinite(targetAngleDeg)) {

            disableControl();
            return;
        }

        long now = System.nanoTime();

        double dt =
                (now - previousTime) / 1_000_000_000.0;

        previousTime = now;

        if (dt <= 0 || dt > 0.1) {
            dt = 0.02;
            filteredVelocity = 0;
        }

        // VELOCIDAD REAL FILTRADA
        double rawVelocity =
                (currentAngle - previousAngle) / dt;

        previousAngle = currentAngle;

        filteredVelocity =
                0.8 * filteredVelocity
                        + 0.2 * rawVelocity;

        // ERROR
        double error =
                targetAngleDeg - currentAngle;

        // PD CALIBRADO
        double pid =
                KP * error
                        - KD * filteredVelocity;

        double ff =
                KV * targetVelocityDegPerSec;

        double output = pid + ff;

        // TOLERANCIA
        if (Math.abs(error) <= TOLERANCE_DEG) {
            output = 0;
        }

        // LIMITAR POTENCIA
        output = Range.clip(
                output,
                -maxPower,
                maxPower
        );

        // =================================================
        // LIMITES: PERMITIR REGRESO
        // =================================================

        if (currentAngle >= MAX_ANGLE_DEG) {

            // No permitir aumentar mas el angulo
            output = Math.min(output, 0);
        }

        if (currentAngle <= MIN_ANGLE_DEG) {

            // No permitir disminuir mas el angulo
            output = Math.max(output, 0);
        }

        // =================================================
        // PROTECCION POR FALTA DE MOVIMIENTO
        // =================================================

        if (Math.abs(output) >= STALL_POWER_THRESHOLD
                && Math.abs(error) >= STALL_ERROR_THRESHOLD_DEG) {

            if (stallStartNs == 0) {

                stallStartNs = now;
                stallStartAngle = currentAngle;

            } else if (
                    Math.abs(currentAngle - stallStartAngle)
                            >= STALL_MOVEMENT_DEG
            ) {

                stallStartNs = now;
                stallStartAngle = currentAngle;

            } else if (
                    now - stallStartNs >= STALL_TIMEOUT_NS
            ) {

                stalled = true;
                disableControl();
                return;
            }

        } else {

            stallStartNs = 0;
        }

        // ENVIAR POTENCIA
        turretMotor.setPower(output);

        lastPID = pid;
        lastFeedforward = ff;
        lastOutput = output;
    }

    // =====================================================
    // TARGET
    // =====================================================

    public void setTargetAngleDeg(double angle) {

        if (Double.isFinite(angle)) {

            targetAngleDeg = Range.clip(
                    angle,
                    MIN_ANGLE_DEG,
                    MAX_ANGLE_DEG
            );
        }
    }

    public void setTargetVelocityDegPerSec(double velocity) {

        targetVelocityDegPerSec =
                Double.isFinite(velocity) ? velocity : 0;
    }

    // =====================================================
    // ENCODER
    // =====================================================

    public double getCurrentAngleDeg() {

        return ENCODER_SIGN *
                (
                        (turretEncoder.getCurrentPosition()
                                - encoderZero)
                                / TICKS_PER_DEGREE
                );
    }

    // =====================================================
    // GETTERS
    // =====================================================

    public double getTargetAngleDeg() {
        return targetAngleDeg;
    }

    public double getTargetVelocityDegPerSec() {
        return targetVelocityDegPerSec;
    }

    public double getErrorDeg() {
        return targetAngleDeg - getCurrentAngleDeg();
    }

    public double getMotorPower() {
        return lastOutput;
    }

    public double getPIDOutput() {
        return lastPID;
    }

    public double getFeedforward() {
        return lastFeedforward;
    }

    public double getVelocityDegPerSec() {
        return filteredVelocity;
    }

    public int getRawTicks() {
        return turretEncoder.getCurrentPosition();
    }

    public int getRelativeTicks() {

        return turretEncoder.getCurrentPosition()
                - encoderZero;
    }

    // =====================================================
    // RESET ZERO
    // =====================================================

    public void resetZero() {

        // Solo usar con la torreta inmovil
        // y en una posicion cero conocida

        disableControl();

        encoderZero = turretEncoder.getCurrentPosition();

        targetAngleDeg = 0;

        resetControllerState();
    }

    // =====================================================
    // STOP
    // =====================================================

    public void stop() {
        disableControl();
    }
}

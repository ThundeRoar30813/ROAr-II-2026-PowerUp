
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class TurretSubsystemV4 {

    public static double MIN_ANGLE_DEG = -75.0;
    public static double MAX_ANGLE_DEG = 235.0;

    public static double TICKS_PER_REV = 8192.0;
    public static double GEAR_RATIO = 95.0 / 15.0;

    public static double ENCODER_SIGN = 1.0;
    public static double MOTOR_SIGN = 1.0;

    // PID: comenzar conservador para evitar oscilacion
    public double kP = 0.035;
    public double kD = 0.0015;
    public double kS = 0.0;

    // Se puede aumentar despues de verificar
    // que el feedforward tiene el signo correcto
    public double maxPower = 0.85;

    public double toleranceDeg = 0.8;

    // Feedforward:
    // potencia por grado/segundo solicitado
    // Valor inicial experimental
    public double kV = 0.0012;

    public double maxFeedforwardPower = 0.40;

    private static final double STALL_TIMEOUT_S = 1.5;
    private static final double STALL_MIN_POWER = 0.18;
    private static final double STALL_MIN_ERROR = 5.0;
    private static final double STALL_MIN_PROGRESS = 0.5;

    private final DcMotorEx motor;
    private final DcMotorEx encoder;
    private final int zeroTicks;

    private boolean enabled = false;
    private boolean fault = false;

    private String faultReason = "NONE";

    private double targetDeg = 0.0;
    private double velocityFeedforwardDegS = 0.0;

    private double lastAngle = 0.0;
    private double velocityDegS = 0.0;
    private double appliedPower = 0.0;

    private long lastUpdateNs;
    private long stallStartNs;
    private double stallReferenceAngle = 0.0;

    public TurretSubsystemV4(HardwareMap hardwareMap) {

        motor = hardwareMap.get(
                DcMotorEx.class, "turret"
        );

        encoder = hardwareMap.get(
                DcMotorEx.class, "mfl"
        );

        motor.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        motor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        motor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        motor.setPower(0);

        zeroTicks = encoder.getCurrentPosition();

        lastAngle = 0;
        targetDeg = 0;

        lastUpdateNs = System.nanoTime();
        stallStartNs = lastUpdateNs;
    }

    public double getAngle() {

        double ticks =
                encoder.getCurrentPosition() - zeroTicks;

        double ticksPerDegree =
                TICKS_PER_REV * GEAR_RATIO / 360.0;

        return ENCODER_SIGN * ticks / ticksPerDegree;
    }

    public double getTarget() {
        return targetDeg;
    }

    public double getError() {
        return targetDeg - getAngle();
    }

    public double getVelocity() {
        return velocityDegS;
    }

    public double getPower() {
        return appliedPower;
    }

    public double getVelocityFeedforward() {
        return velocityFeedforwardDegS;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isFault() {
        return fault;
    }

    public String getFaultReason() {
        return faultReason;
    }

    public void setTarget(double angleDeg) {

        if (!Double.isFinite(angleDeg)) {
            return;
        }

        targetDeg = Range.clip(
                angleDeg,
                MIN_ANGLE_DEG,
                MAX_ANGLE_DEG
        );
    }

    public void setVelocityFeedforward(
            double degreesPerSecond) {

        if (!Double.isFinite(degreesPerSecond)) {
            velocityFeedforwardDegS = 0;
            return;
        }

        velocityFeedforwardDegS = Range.clip(
                degreesPerSecond, -400, 400
        );
    }

    public void enableHold() {

        if (fault) {
            return;
        }

        targetDeg = Range.clip(
                getAngle(),
                MIN_ANGLE_DEG,
                MAX_ANGLE_DEG
        );

        velocityFeedforwardDegS = 0;
        enabled = true;

        stallStartNs = System.nanoTime();
        stallReferenceAngle = getAngle();
    }

    public void disable() {

        enabled = false;
        velocityFeedforwardDegS = 0;
        applyPower(0);
    }

    public void clearFault() {

        disable();

        fault = false;
        faultReason = "NONE";

        targetDeg = Range.clip(
                getAngle(),
                MIN_ANGLE_DEG,
                MAX_ANGLE_DEG
        );

        stallStartNs = System.nanoTime();
        stallReferenceAngle = getAngle();
    }

    private void setFault(String reason) {

        fault = true;
        faultReason = reason;
        disable();
    }

    private void applyPower(double power) {

        appliedPower = Range.clip(
                MOTOR_SIGN * power,
                -Math.abs(maxPower),
                Math.abs(maxPower)
        );

        motor.setPower(appliedPower);
    }

    public void update() {

        long now = System.nanoTime();

        double dt = (now - lastUpdateNs) / 1e9;
        lastUpdateNs = now;

        if (!Double.isFinite(dt) ||
                dt <= 0 || dt > 0.15) {
            dt = 0.02;
        }

        double angle = getAngle();

        if (!Double.isFinite(angle)) {
            setFault("INVALID_ENCODER");
            return;
        }

        double rawVelocity =
                (angle - lastAngle) / dt;

        lastAngle = angle;

        velocityDegS =
                0.65 * velocityDegS
                        + 0.35 * rawVelocity;

        if (!enabled || fault) {
            applyPower(0);
            return;
        }

        if (angle < MIN_ANGLE_DEG - 2.0 ||
                angle > MAX_ANGLE_DEG + 2.0) {

            setFault("ANGLE_OUT_OF_RANGE");
            return;
        }

        double error = targetDeg - angle;

        // =====================================
        // PID
        // =====================================

        double pidPower =
                kP * error
                        - kD * velocityDegS;

        // =====================================
        // VELOCITY FEEDFORWARD
        // =====================================

        double ffPower = Range.clip(
                kV * velocityFeedforwardDegS,
                -maxFeedforwardPower,
                maxFeedforwardPower
        );

        double command = pidPower + ffPower;

        if (Math.abs(error) > toleranceDeg) {
            command += Math.signum(error) * kS;
        }

        // No apagar el feedforward si
        // el objetivo se esta moviendo.
        if (Math.abs(error) <= toleranceDeg &&
                Math.abs(velocityFeedforwardDegS) < 3.0 &&
                Math.abs(velocityDegS) < 8.0) {

            command = 0;
        }

        command = Range.clip(
                command,
                -Math.abs(maxPower),
                Math.abs(maxPower)
        );

        // =====================================
        // LIMITES
        // =====================================

        if (angle <= MIN_ANGLE_DEG &&
                command < 0) {
            command = 0;
        }

        if (angle >= MAX_ANGLE_DEG &&
                command > 0) {
            command = 0;
        }

        // =====================================
        // STALL
        // =====================================

        if (Math.abs(command) >= STALL_MIN_POWER &&
                Math.abs(error) >= STALL_MIN_ERROR) {

            if (Math.abs(
                    angle - stallReferenceAngle
            ) >= STALL_MIN_PROGRESS) {

                stallReferenceAngle = angle;
                stallStartNs = now;

            } else if ((now - stallStartNs) / 1e9
                    >= STALL_TIMEOUT_S) {

                setFault("STALL");
                return;
            }

        } else {

            stallReferenceAngle = angle;
            stallStartNs = now;
        }

        applyPower(command);
    }

    public void stop() {
        disable();
    }
}

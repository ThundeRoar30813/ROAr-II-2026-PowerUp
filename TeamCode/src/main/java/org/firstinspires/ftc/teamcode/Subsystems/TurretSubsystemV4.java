
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class TurretSubsystemV4 {

    public static final double MIN_ANGLE = -75.0;
    public static final double MAX_ANGLE = 235.0;

    public static final double TICKS_PER_DEG =
            8192.0 * (95.0 / 15.0) / 360.0;

    // Verificar ambos signos antes de activar tracking.
    public static final double ENCODER_SIGN = 1.0;
    public static final double MOTOR_SIGN = 1.0;

    public double kP = 0.035;
    public double kD = 0.0020;
    public double maxPower = 0.35;
    public double toleranceDeg = 1.0;

    private final DcMotorEx motor;
    private final DcMotorEx encoder;
    private final int zeroTicks;

    private double target;
    private double power;
    private double velocity;
    private double previousAngle;

    private long lastNs;
    private long noProgressNs;
    private double progressAngle;

    private boolean enabled;
    private boolean fault;
    private String faultReason = "NONE";

    public TurretSubsystemV4(HardwareMap map) {

        motor = map.get(DcMotorEx.class, "turret");
        encoder = map.get(DcMotorEx.class, "mfl");

        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );
        motor.setPower(0);

        // Colocar torreta al frente ANTES de INIT.
        // No reiniciar el encoder compartido.
        zeroTicks = encoder.getCurrentPosition();

        target = previousAngle = getAngle();
        lastNs = System.nanoTime();
    }

    public double getAngle() {
        return ENCODER_SIGN *
                (encoder.getCurrentPosition() - zeroTicks)
                / TICKS_PER_DEG;
    }

    public void enableHold() {

        if (fault) return;

        target = getAngle();
        previousAngle = target;
        velocity = 0;

        lastNs = System.nanoTime();
        noProgressNs = 0;
        enabled = true;
    }

    public void setTarget(double degrees) {

        if (Double.isFinite(degrees)) {
            target = Range.clip(
                    degrees, MIN_ANGLE, MAX_ANGLE
            );
        }
    }

    public boolean setEquivalentTarget(double degrees) {

        if (!Double.isFinite(degrees)) return false;

        double best = Double.NaN;
        double distance = Double.POSITIVE_INFINITY;

        for (int k = -2; k <= 2; k++) {

            double candidate = degrees + 360.0 * k;

            if (candidate < MIN_ANGLE ||
                    candidate > MAX_ANGLE) continue;

            double delta = Math.abs(candidate - getAngle());

            if (delta < distance) {
                distance = delta;
                best = candidate;
            }
        }

        if (!Double.isFinite(best)) return false;

        setTarget(best);
        return true;
    }

    public void update() {

        if (!enabled || fault) {
            motor.setPower(0);
            power = 0;
            return;
        }

        long now = System.nanoTime();
        double dt = (now - lastNs) * 1e-9;
        lastNs = now;

        if (dt <= 0 || dt > 0.15) dt = 0.02;

        double actual = getAngle();

        if (!Double.isFinite(actual) ||
                actual < -85 || actual > 245) {
            trip("ANGLE_OUT_OF_RANGE");
            return;
        }

        double rawVelocity =
                (actual - previousAngle) / dt;

        previousAngle = actual;

        velocity = 0.75 * velocity
                + 0.25 * rawVelocity;

        double error = target - actual;

        double command =
                Math.abs(error) <= toleranceDeg
                        ? 0
                        : kP * error - kD * velocity;

        command = Range.clip(
                command, -maxPower, maxPower
        );

        if ((actual >= MAX_ANGLE && command > 0) ||
                (actual <= MIN_ANGLE && command < 0)) {
            command = 0;
        }

        // Detectar retroalimentacion invertida.
        if (Math.abs(error) > 8 &&
                Math.abs(command) > 0.12 &&
                velocity * Math.signum(error) < -15) {

            trip("WRONG_DIRECTION");
            return;
        }

        // Detectar mecanismo bloqueado.
        if (Math.abs(error) > 5 &&
                Math.abs(command) >= 0.15) {

            if (noProgressNs == 0 ||
                    Math.abs(actual - progressAngle) >= 0.6) {

                noProgressNs = now;
                progressAngle = actual;

            } else if (now - noProgressNs >
                    1_200_000_000L) {

                trip("STALL_NO_PROGRESS");
                return;
            }

        } else {
            noProgressNs = 0;
        }

        power = MOTOR_SIGN * command;
        motor.setPower(power);
    }

    private void trip(String reason) {
        fault = true;
        faultReason = reason;
        disable();
    }

    public void disable() {
        enabled = false;
        power = 0;
        motor.setPower(0);
        noProgressNs = 0;
    }

    public void clearFault() {
        disable();
        fault = false;
        faultReason = "NONE";
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

    public double getTarget() {
        return target;
    }

    public double getError() {
        return target - getAngle();
    }

    public double getPower() {
        return power;
    }

    public double getVelocity() {
        return velocity;
    }

    public void stop() {
        disable();
    }
}

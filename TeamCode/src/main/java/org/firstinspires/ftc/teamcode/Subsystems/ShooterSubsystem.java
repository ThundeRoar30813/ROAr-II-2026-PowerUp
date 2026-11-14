
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class ShooterSubsystem {

    public static double DEFAULT_TARGET_TPS = 1200.0;
    public static double MAX_TARGET_TPS = 6000.0;

    // Ganancias iniciales para control por software.
    // NO son equivalentes a las ganancias PIDF del hub.
    public double kP = 0.0004;
    public double kI = 0.0;
    public double kD = 0.0;
    public double kF = 0.000450;

    public double integralLimit = 5000.0;

    private final DcMotorEx motor;

    private boolean enabled = false;
    private double targetTPS = DEFAULT_TARGET_TPS;
    private double actualTPS = 0;
    private double errorTPS = 0;
    private double outputPower = 0;

    private double integral = 0;
    private double lastError = 0;
    private long lastNs = 0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "shoot");

        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);

        lastNs = System.nanoTime();
    }

    public void toggle() {
        setEnabled(!enabled);
    }

    public void setEnabled(boolean enabled) {
        if (this.enabled != enabled) {
            integral = 0;
            lastError = 0;
            lastNs = System.nanoTime();
        }

        this.enabled = enabled;

        if (!enabled) {
            outputPower = 0;
            motor.setPower(0);
        }
    }

    public void setTargetTPS(double target) {
        if (!Double.isFinite(target)) return;

        targetTPS = Range.clip(
                target, 0, MAX_TARGET_TPS
        );
    }

    public void update() {
        long now = System.nanoTime();
        double dt = (now - lastNs) * 1e-9;
        lastNs = now;

        if (!Double.isFinite(dt) || dt <= 0 || dt > 0.15) {
            dt = 0.02;
        }

        actualTPS = motor.getVelocity();

        if (!Double.isFinite(actualTPS)) {
            setEnabled(false);
            return;
        }

        errorTPS = targetTPS - actualTPS;

        if (!enabled) {
            outputPower = 0;
            motor.setPower(0);
            integral = 0;
            lastError = errorTPS;
            return;
        }

        double derivative = (errorTPS - lastError) / dt;

        // Calcular integral candidata.
        double nextIntegral = Range.clip(
                integral + errorTPS * dt,
                -integralLimit,
                integralLimit
        );

        double proposedPower =
                kP * errorTPS +
                        kI * nextIntegral +
                        kD * derivative +
                        kF * targetTPS;

        // Anti-windup: no acumular integral cuando
        // la salida ya está saturada en esa dirección.
        if ((proposedPower < 1.0 || errorTPS < 0) &&
                (proposedPower > 0.0 || errorTPS > 0)) {
            integral = nextIntegral;
        }

        outputPower = Range.clip(
                kP * errorTPS +
                        kI * integral +
                        kD * derivative +
                        kF * targetTPS,
                0.0,
                1.0
        );

        motor.setPower(outputPower);
        lastError = errorTPS;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public double getTargetTPS() {
        return targetTPS;
    }

    public double getActualTPS() {
        return actualTPS;
    }

    public double getErrorTPS() {
        return errorTPS;
    }

    public double getPower() {
        return outputPower;
    }

    public void stop() {
        setEnabled(false);
    }
}

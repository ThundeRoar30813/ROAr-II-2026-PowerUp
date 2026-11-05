
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class TurretSubsystem {

    // =========================================
    // CONFIGURACION ENCODER
    // =========================================

    private static final double TICKS_PER_REV = 51940.0;

    private static final double TICKS_PER_DEGREE =
            TICKS_PER_REV / 360.0;

    // =========================================
    // PID ORIGINAL
    // =========================================

    private static final double kP = 0.025;
    private static final double kI = 0.0007;
    private static final double kD = 0.0007;

    private static final double MAX_POWER = 0.80;
    private static final double MIN_POWER = 0.12;

    private static final double TOLERANCE_DEG = 1.0;

    // =========================================
    // LIMITES TORRETA
    // =========================================

    public static final double MIN_ANGLE_DEG = -120.0;
    public static final double MAX_ANGLE_DEG = 125.0;

    // =========================================
    // HARDWARE
    // =========================================

    private final DcMotorEx turretMotor;
    private final DcMotorEx turretEncoder;

    private int encoderZero;

    // =========================================
    // VARIABLES PID
    // =========================================

    private double targetAngleDeg = 0.0;

    private double targetVelocityDegPerSec = 0.0;

    private double integral = 0.0;
    private double previousError = 0.0;

    private long previousTime;

    private double motorPower = 0.0;

    // =========================================
    // CONSTRUCTOR
    // =========================================

    public TurretSubsystem(HardwareMap hardwareMap) {

        turretMotor = hardwareMap.get(
                DcMotorEx.class,
                "turret"
        );

        turretEncoder = hardwareMap.get(
                DcMotorEx.class,
                "mfl"
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

        encoderZero = turretEncoder.getCurrentPosition();

        previousTime = System.nanoTime();
    }

    // =========================================
    // UPDATE PID
    // =========================================

    public void update() {

        double currentAngle = getCurrentAngleDeg();

        double error = targetAngleDeg - currentAngle;

        long now = System.nanoTime();

        double dt = (now - previousTime) / 1e9;

        previousTime = now;

        if (dt <= 0 || dt > 0.1) {
            dt = 0.02;
        }

        // INTEGRAL
        integral += error * dt;

        integral = Range.clip(
                integral,
                -100.0,
                100.0
        );

        // DERIVADA
        double derivative =
                (error - previousError) / dt;

        // PID
        double output =
                kP * error
                        + kI * integral
                        + kD * derivative;

        // TOLERANCIA
        if (Math.abs(error) <= TOLERANCE_DEG) {

            output = 0.0;
            integral = 0.0;

        } else {

            // POTENCIA MINIMA
            if (Math.abs(output) < MIN_POWER) {

                output = Math.copySign(
                        MIN_POWER,
                        output == 0.0 ? error : output
                );
            }
        }

        // LIMITAR POTENCIA
        output = Range.clip(
                output,
                -MAX_POWER,
                MAX_POWER
        );

        // =========================================
        // PROTECCION DE LIMITES
        // =========================================

        if (currentAngle <= MIN_ANGLE_DEG
                && output < 0.0) {

            output = 0.0;
        }

        if (currentAngle >= MAX_ANGLE_DEG
                && output > 0.0) {

            output = 0.0;
        }

        // MOVER MOTOR
        turretMotor.setPower(output);

        motorPower = output;
        previousError = error;
    }

    // =========================================
    // OBJETIVO DE ANGULO
    // =========================================

    public void setTargetAngleDeg(double angle) {

        if (Double.isFinite(angle)) {

            targetAngleDeg = Range.clip(
                    angle,
                    MIN_ANGLE_DEG,
                    MAX_ANGLE_DEG
            );
        }
    }

    // =========================================
    // COMPATIBILIDAD CON TRACKING
    // =========================================

    public void setTargetVelocityDegPerSec(
            double velocity
    ) {

        if (Double.isFinite(velocity)) {
            targetVelocityDegPerSec = velocity;
        }

        // Se almacena para telemetria.
        // El PID original controla solo posicion.
    }

    // =========================================
    // LECTURA DE ANGULO
    // =========================================

    public double getCurrentAngleDeg() {

        int relativeTicks =
                turretEncoder.getCurrentPosition()
                        - encoderZero;

        // Convencion de la version antigua
        return -(relativeTicks / TICKS_PER_DEGREE);
    }

    // =========================================
    // GETTERS
    // =========================================

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
        return motorPower;
    }

    public int getRawTicks() {
        return turretEncoder.getCurrentPosition();
    }

    public int getRelativeTicks() {
        return turretEncoder.getCurrentPosition()
                - encoderZero;
    }

    // =========================================
    // REINICIAR CERO
    // =========================================

    public void resetZero() {

        turretMotor.setPower(0);

        encoderZero = turretEncoder.getCurrentPosition();

        targetAngleDeg = 0.0;
        targetVelocityDegPerSec = 0.0;

        integral = 0.0;
        previousError = 0.0;

        motorPower = 0.0;

        previousTime = System.nanoTime();
    }

    // =========================================
    // STOP
    // =========================================

    public void stop() {

        turretMotor.setPower(0);

        targetVelocityDegPerSec = 0.0;

        integral = 0.0;
        previousError = 0.0;

        motorPower = 0.0;

        previousTime = System.nanoTime();
    }
}

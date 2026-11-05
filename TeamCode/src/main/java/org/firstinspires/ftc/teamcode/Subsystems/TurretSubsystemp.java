package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class TurretSubsystemp {

    // =========================================================
    // HARDWARE
    // =========================================================

    private final DcMotorEx turretMotor;
    private final DcMotorEx turretEncoder;

    // =========================================================
    // ENCODER
    // =========================================================

    private static final double TICKS_PER_REV = 51940.0;
    private static final double TICKS_PER_DEGREE =
            TICKS_PER_REV / 360.0;

    private int encoderZero;

    // =========================================================
    // PID QUE YA CALIBRASTE
    // =========================================================

    private static final double kP = 0.025;
    private static final double kI = 0.0007;
    private static final double kD = 0.0007;

    private static final double MAX_POWER = 0.80;
    private static final double MIN_POWER = 0.12;

    private static final double TOLERANCE_DEG = 1.0;

    // =========================================================
    // FEEDFORWARD
    // =========================================================

    /*
     * Este valor lo vamos a calibrar.
     *
     * Unidad aproximada:
     * power / (grado/segundo)
     *
     * Empezamos BAJO.
     */
    private static final double kV = 0.0015;

    /*
     * Evitamos que un error de cálculo de velocidad
     * mande demasiada potencia.
     */
    private static final double MAX_FF_POWER = 0.40;

    // =========================================================
    // VARIABLES
    // =========================================================

    private double targetAngleDeg = 0.0;

    /*
     * Velocidad a la que queremos que se mueva
     * la torreta en grados/segundo.
     */
    private double targetVelocityDegPerSec = 0.0;

    private double integral = 0.0;
    private double previousError = 0.0;

    private long previousTime;

    // Debug
    private double lastOutput = 0.0;
    private double lastPID = 0.0;
    private double lastFeedforward = 0.0;

    // =========================================================
    // CONSTRUCTOR
    // =========================================================

    public TurretSubsystemp(HardwareMap hardwareMap) {

        turretMotor =
                hardwareMap.get(
                        DcMotorEx.class,
                        "turret"
                );

        turretEncoder =
                hardwareMap.get(
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

        encoderZero =
                turretEncoder.getCurrentPosition();

        previousTime =
                System.nanoTime();
    }

    // =========================================================
    // UPDATE
    // =========================================================

    public void update() {

        double currentAngle =
                getCurrentAngleDeg();

        double error =
                targetAngleDeg - currentAngle;

        long now =
                System.nanoTime();

        double dt =
                (now - previousTime)
                        / 1_000_000_000.0;

        previousTime = now;

        if (dt <= 0 || dt > 0.1) {
            dt = 0.02;
        }

        // =====================================================
        // PID
        // =====================================================

        integral += error * dt;

        integral =
                Range.clip(
                        integral,
                        -100.0,
                        100.0
                );

        double derivative =
                (error - previousError) / dt;

        double p =
                kP * error;

        double i =
                kI * integral;

        double d =
                kD * derivative;

        double pid =
                p + i + d;

        // =====================================================
        // VELOCITY FEEDFORWARD
        // =====================================================

        double velocityFF =
                kV * targetVelocityDegPerSec;

        velocityFF =
                Range.clip(
                        velocityFF,
                        -MAX_FF_POWER,
                        MAX_FF_POWER
                );

        // =====================================================
        // PID + FF
        // =====================================================

        double output =
                pid + velocityFF;

        // =====================================================
        // TOLERANCIA
        // =====================================================

        /*
         * Solo paramos completamente si:
         *
         * 1. Estamos cerca del target
         * 2. Y el target prácticamente NO se está moviendo.
         *
         * Esto es MUY importante.
         *
         * Antes el PID podía ponerse en 0 aunque el robot
         * siguiera girando.
         */
        if (
                Math.abs(error) <= TOLERANCE_DEG
                        &&
                        Math.abs(targetVelocityDegPerSec) < 2.0
        ) {

            output = 0.0;

            integral = 0.0;
        }

        // =====================================================
        // POTENCIA MÍNIMA
        // =====================================================

        /*
         * Solo usamos MIN_POWER cuando realmente
         * necesitamos movernos.
         */
        if (Math.abs(output) > 0.001) {

            if (Math.abs(output) < MIN_POWER) {

                output =
                        Math.copySign(
                                MIN_POWER,
                                output
                        );
            }
        }

        // =====================================================
        // CLAMP FINAL
        // =====================================================

        output =
                Range.clip(
                        output,
                        -MAX_POWER,
                        MAX_POWER
                );

        turretMotor.setPower(output);

        previousError = error;

        lastPID = pid;
        lastFeedforward = velocityFF;
        lastOutput = output;
    }

    // =========================================================
    // TARGET
    // =========================================================

    public void setTargetAngleDeg(double degrees) {

        targetAngleDeg = degrees;
    }

    public void setTargetVelocityDegPerSec(
            double velocity
    ) {

        targetVelocityDegPerSec = velocity;
    }

    // =========================================================
    // GETTERS
    // =========================================================

    public double getTargetAngleDeg() {

        return targetAngleDeg;
    }

    public double getTargetVelocityDegPerSec() {

        return targetVelocityDegPerSec;
    }

    public double getCurrentAngleDeg() {

        int relativeTicks =
                turretEncoder.getCurrentPosition()
                        - encoderZero;

        return -(relativeTicks / TICKS_PER_DEGREE);
    }

    public double getErrorDeg() {

        return targetAngleDeg
                - getCurrentAngleDeg();
    }

    public double getPIDOutput() {

        return lastPID;
    }

    public double getFeedforward() {

        return lastFeedforward;
    }

    public double getMotorPower() {

        return lastOutput;
    }

    public int getRawTicks() {

        return turretEncoder.getCurrentPosition();
    }

    public int getRelativeTicks() {

        return turretEncoder.getCurrentPosition()
                - encoderZero;
    }

    // =========================================================
    // RESET
    // =========================================================

    public void resetZero() {

        encoderZero =
                turretEncoder.getCurrentPosition();

        targetAngleDeg = 0.0;
        targetVelocityDegPerSec = 0.0;

        integral = 0.0;
        previousError = 0.0;

        lastPID = 0.0;
        lastFeedforward = 0.0;
        lastOutput = 0.0;

        previousTime =
                System.nanoTime();
    }

    // =========================================================
    // STOP
    // =========================================================

    public void stop() {

        turretMotor.setPower(0);

        targetVelocityDegPerSec = 0.0;

        integral = 0.0;

        lastPID = 0.0;
        lastFeedforward = 0.0;
        lastOutput = 0.0;
    }
}
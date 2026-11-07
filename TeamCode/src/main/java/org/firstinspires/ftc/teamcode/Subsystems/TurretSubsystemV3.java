
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class TurretSubsystemV3 {

    private final DcMotorEx motor;
    private final DcMotorEx encoder;

    // =============================================
    // ENCODER Y ENGRANES
    // =============================================

    private static final double ENCODER_CPR = 8192.0;
    private static final double GEAR_RATIO = 94.0 / 15.0;
    private static final double TICKS_PER_DEG =
            ENCODER_CPR * GEAR_RATIO / 360.0;

    // Signo validado anteriormente
    private static final double ENCODER_SIGN = 1.0;

    // =============================================
    // LIMITES
    // =============================================

    public static final double MIN_ANGLE = -110.0;
    public static final double MAX_ANGLE = 230.0;

    // =============================================
    // PID
    // =============================================

    private static final double KP = 0.020;
    private static final double KD = 0.0020;
    private static final double TOLERANCE = 1.0;

    // POTENCIA MAXIMA: 50%
    private double maxPower = 0.80;

    // =============================================
    // ESTADO
    // =============================================

    private final int zeroTicks;

    private double targetAngle = 0;
    private double lastAngle = 0;
    private double filteredVelocity = 0;
    private double motorPower = 0;

    private long lastTime = 0;

    private boolean enabled = false;
    private boolean stalled = false;

    private String faultReason = "NONE";

    // =============================================
    // PROTECCION STALL
    // =============================================

    private long stallStart = 0;
    private double stallAngle = 0;

    private static final long STALL_TIMEOUT_NS =
            1_500_000_000L;

    private static final double STALL_MIN_PROGRESS = 0.5;
    private static final double STALL_MIN_POWER = 0.15;
    private static final double STALL_MIN_ERROR = 5.0;

    // =============================================
    // PROTECCION DIRECCION INCORRECTA
    // =============================================

    private long directionStart = 0;
    private double directionStartAngle = 0;
    private double directionTarget = 0;

    private static final long DIRECTION_TIMEOUT_NS =
            350_000_000L;

    private static final double WRONG_DIRECTION_DEG = 3.0;

    // =============================================
    // CONSTRUCTOR
    // =============================================

    public TurretSubsystemV3(HardwareMap hardwareMap) {

        motor = hardwareMap.get(
                DcMotorEx.class, "turret"
        );

        encoder = hardwareMap.get(
                DcMotorEx.class, "mfl"
        );

        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );
        motor.setPower(0);

        // Colocar la torreta en cero fisico
        // antes de presionar INIT.
        zeroTicks = encoder.getCurrentPosition();

        resetState();
    }

    private void resetState() {

        lastAngle = getAngle();
        filteredVelocity = 0;
        lastTime = System.nanoTime();

        stallStart = 0;
        stallAngle = getAngle();

        directionStart = 0;
        directionStartAngle = getAngle();
        directionTarget = getAngle();

        motorPower = 0;
    }

    // =============================================
    // ANGULO
    // =============================================

    public double getAngle() {

        int ticks =
                encoder.getCurrentPosition() - zeroTicks;

        return ENCODER_SIGN * ticks / TICKS_PER_DEG;
    }

    // =============================================
    // ACTIVAR / DESACTIVAR
    // =============================================

    public void enable() {

        if (stalled) {
            return;
        }

        targetAngle = getAngle();
        resetState();
        enabled = true;
    }

    public void disable() {

        enabled = false;
        motor.setPower(0);
        motorPower = 0;

        stallStart = 0;
        directionStart = 0;
    }

    public void clearFault() {

        disable();

        stalled = false;
        faultReason = "NONE";

        resetState();
    }

    // =============================================
    // TARGET
    // =============================================

    public void setTarget(double degrees) {

        if (!Double.isFinite(degrees)) {
            return;
        }

        targetAngle = Range.clip(
                degrees,
                MIN_ANGLE,
                MAX_ANGLE
        );
    }

    public void setMaxPower(double power) {

        if (!Double.isFinite(power)) {
            return;
        }

        maxPower = Range.clip(power, 0.0, 0.90);
    }

    // =============================================
    // ACTIVAR FALLO
    // =============================================

    private void triggerFault(String reason) {

        faultReason = reason;
        stalled = true;
        disable();
    }

    // =============================================
    // CONTROL PID
    // =============================================

    public void update() {

        if (!enabled || stalled) {

            motor.setPower(0);
            motorPower = 0;
            return;
        }

        double actual = getAngle();

        long now = System.nanoTime();

        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        if (dt <= 0 || dt > 0.1) {
            dt = 0.02;
            filteredVelocity = 0;
        }

        double velocity = (actual - lastAngle) / dt;
        lastAngle = actual;

        filteredVelocity =
                0.8 * filteredVelocity + 0.2 * velocity;

        double error = targetAngle - actual;

        double output =
                KP * error - KD * filteredVelocity;

        if (Math.abs(error) <= TOLERANCE) {
            output = 0;
        }

        output = Range.clip(
                output,
                -maxPower,
                maxPower
        );

        // =========================================
        // LIMITES FISICOS DE SOFTWARE
        // =========================================

        if (actual >= MAX_ANGLE && output > 0) {
            output = 0;
        }

        if (actual <= MIN_ANGLE && output < 0) {
            output = 0;
        }

        // =========================================
        // DETECCION DE MOVIMIENTO INCORRECTO
        // =========================================

        boolean significantCommand =
                Math.abs(error) > 5.0
                        && Math.abs(output) >= 0.15;

        if (significantCommand) {

            if (directionStart == 0
                    || Math.abs(targetAngle - directionTarget) > 1.0) {

                directionStart = now;
                directionStartAngle = actual;
                directionTarget = targetAngle;

            } else {

                double expectedDirection =
                        Math.signum(directionTarget - directionStartAngle);

                double displacement =
                        actual - directionStartAngle;

                double wrongMovement =
                        -expectedDirection * displacement;

                if (wrongMovement >= WRONG_DIRECTION_DEG) {

                    triggerFault("WRONG_DIRECTION");
                    return;
                }

                if (now - directionStart >= DIRECTION_TIMEOUT_NS) {

                    directionStart = now;
                    directionStartAngle = actual;
                    directionTarget = targetAngle;
                }
            }

        } else {

            directionStart = 0;
        }

        // =========================================
        // DETECCION DE STALL
        // =========================================

        if (significantCommand) {

            if (stallStart == 0) {

                stallStart = now;
                stallAngle = actual;

            } else {

                double progress =
                        Math.abs(actual - stallAngle);

                if (progress >= STALL_MIN_PROGRESS) {

                    stallStart = now;
                    stallAngle = actual;

                } else if (
                        now - stallStart > STALL_TIMEOUT_NS
                ) {

                    triggerFault("STALL_NO_MOVEMENT");
                    return;
                }
            }

        } else {

            stallStart = 0;
        }

        // =========================================
        // POTENCIA
        // =========================================

        motor.setPower(output);
        motorPower = output;
    }

    // =============================================
    // TELEMETRIA
    // =============================================

    public double getTarget() {
        return targetAngle;
    }

    public double getError() {
        return targetAngle - getAngle();
    }

    public double getPower() {
        return motorPower;
    }

    public double getVelocity() {
        return filteredVelocity;
    }

    public int getTicks() {
        return encoder.getCurrentPosition() - zeroTicks;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isStalled() {
        return stalled;
    }

    public String getFaultReason() {
        return faultReason;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void stop() {
        disable();
    }
}

package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TEST - Turret PID Tuner")
public class TurretPIDTuner extends OpMode {

    private DcMotorEx motor;
    private DcMotorEx encoder;

    private static final double TICKS_PER_DEGREE =
            51940.0 / 360.0;

    // Debe coincidir con el signo verificado
    // en TurretSubsystem.
    private static final double ENCODER_SIGN = -1.0;

    // PID inicial
    private double kP = 0.010;
    private double kI = 0.0;
    private double kD = 0.0;

    // Comenzar con potencia limitada
    private double maxPower = 0.20;

    private double target = 0.0;

    private double integral = 0.0;
    private double previousAngle = 0.0;
    private double filteredVelocity = 0.0;

    private long previousTime;
    private int zeroTicks;

    private boolean enabled = false;

    private boolean prevA, prevB, prevX, prevY;
    private boolean prevUp, prevDown;
    private boolean prevLeft, prevRight;
    private boolean prevLB, prevRB;

    @Override
    public void init() {

        motor = hardwareMap.get(DcMotorEx.class, "turret");
        encoder = hardwareMap.get(DcMotorEx.class, "mfl");

        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);

        zeroTicks = encoder.getCurrentPosition();

        previousAngle = getAngle();
        previousTime = System.nanoTime();
    }

    private double getAngle() {
        return ENCODER_SIGN *
                (encoder.getCurrentPosition() - zeroTicks)
                / TICKS_PER_DEGREE;
    }

    @Override
    public void loop() {

        // =====================================================
        // BOTONES
        // =====================================================

        if (gamepad1.a && !prevA) {
            target = 0;
            enabled = true;
            resetPID();
        }

        if (gamepad1.b && !prevB) {
            target = 30;
            enabled = true;
            resetPID();
        }

        if (gamepad1.x && !prevX) {
            target = -30;
            enabled = true;
            resetPID();
        }

        if (gamepad1.y && !prevY) {
            target = 60;
            enabled = true;
            resetPID();
        }

        if (gamepad1.dpad_up && !prevUp) {
            kP += 0.001;
        }

        if (gamepad1.dpad_down && !prevDown) {
            kP = Math.max(0, kP - 0.001);
        }

        if (gamepad1.dpad_right && !prevRight) {
            kD += 0.0002;
        }

        if (gamepad1.dpad_left && !prevLeft) {
            kD = Math.max(0, kD - 0.0002);
        }

        if (gamepad1.right_bumper && !prevRB) {
            maxPower = Math.min(0.90, maxPower + 0.05);
        }

        if (gamepad1.left_bumper && !prevLB) {
            maxPower = Math.max(0.10, maxPower - 0.05);
        }

        // Gatillo izquierdo: detener
        if (gamepad1.left_trigger > 0.5) {
            enabled = false;
            motor.setPower(0);
        }

        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevX = gamepad1.x;
        prevY = gamepad1.y;

        prevUp = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;
        prevLeft = gamepad1.dpad_left;
        prevRight = gamepad1.dpad_right;

        prevLB = gamepad1.left_bumper;
        prevRB = gamepad1.right_bumper;

        // =====================================================
        // TIEMPO Y VELOCIDAD
        // =====================================================

        long now = System.nanoTime();
        double dt = (now - previousTime) / 1e9;
        previousTime = now;

        if (dt <= 0 || dt > 0.1) {
            dt = 0.02;
        }

        double angle = getAngle();

        double rawVelocity =
                (angle - previousAngle) / dt;

        previousAngle = angle;

        // Filtro para reducir ruido del encoder
        filteredVelocity =
                0.8 * filteredVelocity
                        + 0.2 * rawVelocity;

        double error = target - angle;
        double power = 0.0;

        // =====================================================
        // PID CON DERIVADA SOBRE MEDICION
        // =====================================================

        if (enabled) {

            if (Math.abs(error) > 1.0) {

                integral += error * dt;

                integral = Range.clip(
                        integral, -30.0, 30.0
                );

                double p = kP * error;
                double i = kI * integral;
                double d = -kD * filteredVelocity;

                power = Range.clip(
                        p + i + d,
                        -maxPower,
                        maxPower
                );

            } else {
                integral = 0;
                power = 0;
            }

            // Limites de prueba
            if (angle >= 100 && power > 0) {
                power = 0;
            }

            if (angle <= -100 && power < 0) {
                power = 0;
            }

            motor.setPower(power);

        } else {
            motor.setPower(0);
        }

        // =====================================================
        // TELEMETRIA
        // =====================================================

        telemetry.addLine("===== PID TUNER =====");

        telemetry.addData("Enabled", enabled);
        telemetry.addData("Target", "%.2f", target);
        telemetry.addData("Actual", "%.2f", angle);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Velocity", "%.2f deg/s",
                filteredVelocity);
        telemetry.addData("Motor Power", "%.3f", power);

        telemetry.addLine("===== PID =====");
        telemetry.addData("kP", "%.5f", kP);
        telemetry.addData("kI", "%.5f", kI);
        telemetry.addData("kD", "%.5f", kD);
        telemetry.addData("Max Power", "%.2f", maxPower);

        telemetry.addLine("===== CONTROLS =====");
        telemetry.addLine("A = 0 deg");
        telemetry.addLine("B = +30 deg");
        telemetry.addLine("X = -30 deg");
        telemetry.addLine("Y = +60 deg");
        telemetry.addLine("UP/DOWN = P +/-");
        telemetry.addLine("RIGHT/LEFT = D +/-");
        telemetry.addLine("RB/LB = Max Power +/-");
        telemetry.addLine("LT = STOP");

        telemetry.update();
    }

    private void resetPID() {
        integral = 0;
        filteredVelocity = 0;
        previousAngle = getAngle();
        previousTime = System.nanoTime();
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }
}
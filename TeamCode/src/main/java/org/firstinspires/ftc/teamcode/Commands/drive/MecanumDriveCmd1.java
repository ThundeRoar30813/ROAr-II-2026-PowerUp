package org.firstinspires.ftc.teamcode.Commands.drive;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem1;

public class MecanumDriveCmd1 {

    private final DriveSubsystem1 drive;
    private final Gamepad gamepad;

    // Ajustes del chasis
    private static final double DEADBAND = 0.05;
    private static final double MAX_POWER = 1.0;
    private static final double SLOW_POWER = 0.35;

    private static final double LINEAR_WEIGHT = 0.90;
    private static final double CUBIC_WEIGHT = 0.90;

    // Cambio máximo de potencia por ciclo
    private static final double ACCEL_SLEW_RATE = 0.70;

    private double lastForward = 0;
    private double lastStrafe = 0;
    private double lastTurn = 0;

    private double outputForward = 0;
    private double outputStrafe = 0;
    private double outputTurn = 0;

    public MecanumDriveCmd1(
            DriveSubsystem1 drive,
            Gamepad gamepad
    ) {
        this.drive = drive;
        this.gamepad = gamepad;
    }

    public void execute() {

        // 1. Joysticks
        double forward = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = gamepad.right_stick_x;

        // 2. Deadband
        forward = applyDeadband(forward);
        strafe = applyDeadband(strafe);
        turn = applyDeadband(turn);

        // 3. Curva híbrida
        forward = hybridCurve(forward);
        strafe = hybridCurve(strafe);
        turn = hybridCurve(turn);

        // 4. Modo lento con LB
        double multiplier = gamepad.left_bumper
                ? SLOW_POWER
                : MAX_POWER;

        forward *= multiplier;
        strafe *= multiplier;
        turn *= multiplier;

        // 5. Limitar la aceleración
        forward = accelerationSlew(lastForward, forward);
        strafe = accelerationSlew(lastStrafe, strafe);
        turn = accelerationSlew(lastTurn, turn);

        // Guardar estados
        lastForward = forward;
        lastStrafe = strafe;
        lastTurn = turn;

        outputForward = forward;
        outputStrafe = strafe;
        outputTurn = turn;

        // 6. Mover chasis
        drive.drive(forward, strafe, turn);
    }

    private double applyDeadband(double value) {
        if (Math.abs(value) < DEADBAND) {
            return 0;
        }

        return Math.copySign(
                (Math.abs(value) - DEADBAND) / (1.0 - DEADBAND),
                value
        );
    }

    private double hybridCurve(double value) {
        return LINEAR_WEIGHT * value
                + CUBIC_WEIGHT * value * value * value;
    }

    private double accelerationSlew(
            double previous,
            double target
    ) {
        // Permitir el cambio de dirección inmediatamente
        // cuando ambos valores son distintos de cero.
        if (previous != 0
                && target != 0
                && Math.signum(previous) != Math.signum(target)) {
            return target;
        }

        // Permitir desaceleración inmediata al soltar
        // o reducir la entrada del joystick.
        if (Math.abs(target) <= Math.abs(previous)) {
            return target;
        }

        // Limitar el incremento de potencia
        double change = target - previous;

        change = Range.clip(
                change,
                -ACCEL_SLEW_RATE,
                ACCEL_SLEW_RATE
        );

        return previous + change;
    }

    public void reset() {
        lastForward = 0;
        lastStrafe = 0;
        lastTurn = 0;

        outputForward = 0;
        outputStrafe = 0;
        outputTurn = 0;

        drive.stop();
    }

    public double getForward() { return outputForward; }
    public double getStrafe() { return outputStrafe; }
    public double getTurn() { return outputTurn; }
}

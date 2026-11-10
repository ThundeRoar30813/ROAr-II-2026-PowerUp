
package org.firstinspires.ftc.teamcode.Commands.drive;

import java.util.function.DoubleSupplier;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class MecanumDriveCmd {

    private final DriveSubsystem drive;

    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;

    public static double DEADBAND = 0.05;

    public MecanumDriveCmd(
            DriveSubsystem drive,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            DoubleSupplier turn) {

        this.drive = drive;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < DEADBAND
                ? 0.0
                : Range.clip(value, -1.0, 1.0);
    }

    public void execute() {

        drive.drive(
                applyDeadband(forward.getAsDouble()),
                applyDeadband(strafe.getAsDouble()),
                applyDeadband(turn.getAsDouble())
        );
    }

    public void stop() {
        drive.stop();
    }
}

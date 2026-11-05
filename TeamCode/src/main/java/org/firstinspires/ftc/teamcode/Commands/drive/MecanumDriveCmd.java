package org.firstinspires.ftc.teamcode.Commands.drive;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class MecanumDriveCmd {

    private final DriveSubsystem drive;

    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;


    public MecanumDriveCmd(
            DriveSubsystem drive,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            DoubleSupplier turn
    ) {

        this.drive = drive;

        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
    }


    public void execute() {

        double f =
                forward.getAsDouble();

        double s =
                strafe.getAsDouble();

        double t =
                turn.getAsDouble();


        // Deadband
        f = deadband(f, 0.05);
        s = deadband(s, 0.05);
        t = deadband(t, 0.05);


        drive.drive(
                f,
                s,
                t
        );
    }


    public void end() {

        drive.stop();
    }


    private double deadband(
            double value,
            double deadband
    ) {

        if (Math.abs(value) < deadband) {
            return 0;
        }

        return value;
    }
}
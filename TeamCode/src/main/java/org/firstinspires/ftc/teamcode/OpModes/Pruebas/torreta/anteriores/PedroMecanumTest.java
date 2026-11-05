package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta.anteriores;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
@Disabled
@TeleOp(name = "TEST - Pedro Mecanum")
public class PedroMecanumTest extends OpMode {

    private DriveSubsystem drive;

    private MecanumDriveCmd driveCommand;


    @Override
    public void init() {

        drive =
                new DriveSubsystem(
                        hardwareMap
                );


        driveCommand =
                new MecanumDriveCmd(

                        drive,

                        // Adelante / atrás
                        () -> -gamepad1.left_stick_y,

                        // Strafe
                        () -> gamepad1.left_stick_x,

                        // Giro
                        () -> -gamepad1.right_stick_x
                );


        telemetry.addLine(
                "Pedro Mecanum listo"
        );

        telemetry.addLine(
                "Presiona START"
        );

        telemetry.update();
    }


    @Override
    public void loop() {

        // Ejecutar command de manejo
        driveCommand.execute();


        // MUY IMPORTANTE
        drive.update();


        telemetry.addLine(
                "===== PEDRO 3 ====="
        );

        telemetry.addData(
                "X",
                "%.2f in",
                drive.getX()
        );

        telemetry.addData(
                "Y",
                "%.2f in",
                drive.getY()
        );

        telemetry.addData(
                "Heading",
                "%.2f deg",
                drive.getHeadingDegrees()
        );

        telemetry.update();
    }


    @Override
    public void stop() {

        driveCommand.end();
    }
}
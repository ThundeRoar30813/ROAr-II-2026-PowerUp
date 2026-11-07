package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta.anteriores;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.TurretHeadingCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

@TeleOp(name = "TEST - Turret Heading Lock")
public class TurretHeadingLockTest extends OpMode {

    private DriveSubsystem drive;
    private TurretSubsystem turret;

    private MecanumDriveCmd driveCommand;
    private TurretHeadingCommand turretHeadingCommand;

    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void init() {

        drive =
                new DriveSubsystem(
                        hardwareMap
                );

        turret =
                new TurretSubsystem(
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
                        // Ya corregimos que estaba invertido.
                        () -> -gamepad1.right_stick_x
                );

        turretHeadingCommand =
                new TurretHeadingCommand(
                        drive,
                        turret
                );

        telemetry.addLine(
                "TURRET HEADING LOCK"
        );

        telemetry.addLine(
                "A = activar tracking"
        );

        telemetry.addLine(
                "B = detener tracking"
        );

        telemetry.addLine(
                "ANTES DE A:"
        );

        telemetry.addLine(
                "Pon la torreta apuntando al frente."
        );

        telemetry.update();
    }

    @Override
    public void loop() {

        // =====================================================
        // CHASIS
        // =====================================================

        driveCommand.execute();

        drive.update();

        // =====================================================
        // BOTÓN A - ACTIVAR
        // =====================================================

        boolean currentA =
                gamepad1.a;

        if (currentA && !lastA) {

            turretHeadingCommand.enable();
        }

        lastA = currentA;

        // =====================================================
        // BOTÓN B - DESACTIVAR
        // =====================================================

        boolean currentB =
                gamepad1.b;

        if (currentB && !lastB) {

            turretHeadingCommand.disable();
        }

        lastB = currentB;

        // =====================================================
        // TURRET
        // =====================================================

        turretHeadingCommand.execute();

        // =====================================================
        // TELEMETRY
        // =====================================================

        telemetry.addLine(
                "===== PEDRO ====="
        );

        telemetry.addData(
                "X",
                "%.2f",
                drive.getX()
        );

        telemetry.addData(
                "Y",
                "%.2f",
                drive.getY()
        );

        telemetry.addData(
                "Robot Heading",
                "%.2f deg",
                drive.getHeadingDegrees()
        );

        telemetry.addLine(
                "===== TURRET ====="
        );

        telemetry.addData(
                "Tracking",
                turretHeadingCommand.isEnabled()
                        ? "ON"
                        : "OFF"
        );

        telemetry.addData(
                "Start Heading",
                "%.2f deg",
                turretHeadingCommand
                        .getStartingRobotHeadingDeg()
        );

        telemetry.addData(
                "Target",
                "%.2f deg",
                turret.getTargetAngleDeg()
        );

        telemetry.addData(
                "Actual",
                "%.2f deg",
                turret.getCurrentAngleDeg()
        );

        telemetry.addData(
                "Error",
                "%.2f deg",
                turret.getErrorDeg()
        );

        telemetry.addData(
                "Power",
                "%.3f",
                turret.getMotorPower()
        );

//        telemetry.addData(
//                "Encoder",
//                turret.getRelativeTicks()
//        );

        telemetry.update();
    }

    @Override
    public void stop() {

        driveCommand.end();

        turretHeadingCommand.disable();

        turret.stop();
    }
}
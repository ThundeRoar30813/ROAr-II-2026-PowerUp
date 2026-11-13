
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV5;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV77;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretControlCommand;
import org.firstinspires.ftc.teamcode.Commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.shooter.ShooterTuningCommand;

import org.firstinspires.ftc.teamcode.Subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.Commands.shooter.HoodCommand;


@TeleOp(name = "Tablas Shooter")
public class TablasShooter extends OpMode {

    // DRIVE
    private DriveSubsystem drive;
    private MecanumDriveCmd driveCmd;

    // TURRET
    private TurretSubsystemV4 turret;
    private TurretVisionV5 vision;
    private TurretAimControllerV77 aim;
    private TurretControlCommand turretCmd;

    // INTAKE
    private IntakeSubsystem intake;
    private IntakeCommand intakeCmd;


    private HoodSubsystem hood;
    private HoodCommand hoodCmd;


    // SHOOTER
    private ShooterSubsystem shooter;
    private ShooterTuningCommand shooterCmd;

    @Override
    public void init() {

        // DRIVE
        drive = new DriveSubsystem(hardwareMap);
        drive.setInitialPose(48, 6, 90);


        hood = new HoodSubsystem(hardwareMap);
        hoodCmd = new HoodCommand(hood);


        driveCmd = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );

        // TURRET
        turret = new TurretSubsystemV4(hardwareMap);
        vision = new TurretVisionV5(hardwareMap);

        aim = new TurretAimControllerV77(
                drive,
                turret,
                vision
        );

        turretCmd = new TurretControlCommand(
                drive,
                turret,
                vision,
                aim
        );

        // INTAKE
        intake = new IntakeSubsystem(hardwareMap);
        intakeCmd = new IntakeCommand(intake);

        // SHOOTER
        shooter = new ShooterSubsystem(hardwareMap);
        shooterCmd = new ShooterTuningCommand(shooter);

        telemetry.addLine("TABLAS SHOOTER LISTO");
        telemetry.addLine("G1 X: SHOOTER ON/OFF");
        telemetry.addLine("G1 FLECHAS: P / I / D / F");
        telemetry.addLine("G2 RT/LT: TPS +/-");
        telemetry.addLine("G2 RB/LB: PIDF +/-");
        telemetry.update();
    }

    @Override
    public void start() {
        turretCmd.start();
    }

    @Override
    public void loop() {

        // =====================================
        // DRIVE
        // =====================================
        driveCmd.execute();
        drive.update();


        // HOOD - GAMEPAD 2
        hoodCmd.execute(
                gamepad2.dpad_up,
                gamepad2.dpad_down,
                gamepad2.dpad_left,
                gamepad2.dpad_right
        );


        // =====================================
        // TURRET
        // =====================================
        turretCmd.execute(
                gamepad1.b,
                gamepad1.start,
                gamepad1.back
        );

        // =====================================
        // INTAKE
        // =====================================

// INTAKE + GATE
        intakeCmd.execute(
                gamepad1.y,             // Auto Intake ON/OFF
                gamepad1.right_bumper,  // Alimentar + abrir gate
                gamepad1.a,             // Carga manual
                gamepad2.y              // Reset contador
        );


        // =====================================
        // SHOOTER PIDF
        // =====================================
        shooterCmd.execute(
                gamepad1.x,

                gamepad2.right_trigger,
                gamepad2.left_trigger,

                gamepad1.dpad_up,     // P
                gamepad1.dpad_right,  // I
                gamepad1.dpad_down,   // D
                gamepad1.dpad_left,   // F

                gamepad2.right_bumper, // PIDF +
                gamepad2.left_bumper   // PIDF -
        );

        // =====================================
        // TELEMETRIA
        // =====================================
        telemetry.addLine("===== TABLAS SHOOTER =====");

        telemetry.addData(
                "Zona",
                turretCmd.getFieldSide()
        );

        telemetry.addData(
                "AprilTag ID",
                turretCmd.getActiveTagId()
        );

        telemetry.addData(
                "Tracking",
                turretCmd.isTracking()
        );

        telemetry.addData(
                "Modo",
                aim.getMode()
        );

        // =====================================
        // SHOOTER
        // =====================================
        telemetry.addLine("===== SHOOTER PIDF =====");

        telemetry.addData(
                "Shooter",
                shooter.isEnabled() ? "ON" : "OFF"
        );

        telemetry.addData(
                "Target TPS",
                "%.1f",
                shooter.getTargetTPS()
        );

        telemetry.addData(
                "Actual TPS",
                "%.1f",
                shooter.getActualTPS()
        );

        telemetry.addData(
                "Error TPS",
                "%.1f",
                shooter.getErrorTPS()
        );

        telemetry.addData(
                "Power",
                "%.3f",
                shooter.getPower()
        );

        telemetry.addData(
                "Parametro seleccionado",
                shooterCmd.getSelectedParameter()
        );

        telemetry.addData("P", "%.7f", shooter.kP);
        telemetry.addData("I", "%.7f", shooter.kI);
        telemetry.addData("D", "%.7f", shooter.kD);
        telemetry.addData("F", "%.7f", shooter.kF);

        // =====================================
        // DISTANCIA AL APRILTAG
        // =====================================
        telemetry.addLine("===== DISTANCIA =====");

        double distanceM =
                vision.getDistanceToTagMeters();

        if (vision.hasTarget() &&
                Double.isFinite(distanceM)) {

            telemetry.addData(
                    "Distancia metros",
                    "%.3f",
                    distanceM
            );

            telemetry.addData(
                    "Distancia cm",
                    "%.1f",
                    distanceM * 100.0
            );

            telemetry.addData(
                    "Distancia pulgadas",
                    "%.1f",
                    distanceM * 39.3700787
            );

        } else {
            telemetry.addLine(
                    "Distancia: SIN MEDICION VALIDA"
            );
        }

        telemetry.addData(
                "Tag Visible",
                vision.hasTarget()
        );

        telemetry.addData(
                "Tx",
                "%.2f",
                vision.getTx()
        );

        telemetry.addData(
                "Age ms",
                "%.0f",
                vision.getAgeMs()
        );

        // =====================================
        // TURRET
        // =====================================
        telemetry.addLine("===== TURRET =====");

        telemetry.addData(
                "Angle",
                "%.2f",
                turret.getAngle()
        );

        telemetry.addData(
                "Target",
                "%.2f",
                turret.getTarget()
        );

        telemetry.addData(
                "Error",
                "%.2f",
                turret.getError()
        );

        telemetry.addData(
                "Ready",
                turret.isReadyToShoot(2.5)
        );

        telemetry.addData(
                "Fault",
                turret.getFaultReason()
        );

        // =====================================
        // INTAKE
        // =====================================
        telemetry.addLine("===== INTAKE =====");

        telemetry.addData(
                "Auto Intake",
                intake.isAutoIntakeEnabled()
        );

        telemetry.addData(
                "Ball Count",
                intake.getBallCount()
        );

        telemetry.addData(
                "Bottom V2",
                intake.isBottomDetected()
        );

        telemetry.addData(
                "Top V3",
                intake.isTopDetected()
        );

        // =====================================
        // PEDRO
        // =====================================
        telemetry.addLine("===== PEDRO =====");

        telemetry.addData(
                "Robot X/Y",
                "%.1f / %.1f",
                drive.getX(),
                drive.getY()
        );

        telemetry.addData(
                "Heading",
                "%.2f",
                drive.getHeadingDegrees()
        );


        telemetry.addLine("===== HOOD =====");

        telemetry.addData(
                "Hood conectada",
                hood.isConnected()
        );

        telemetry.addData(
                "Hood nivel",
                "%d / 5",
                hood.getPositionNumber()
        );

        telemetry.addData(
                "Hood servo",
                "%.4f",
                hood.getPosition()
        );


        telemetry.update();
    }

    @Override
    public void stop() {

        if (shooterCmd != null) {
            shooterCmd.stop();
        }

        if (intakeCmd != null) {
            intakeCmd.stop();
        }

        if (turretCmd != null) {
            turretCmd.stop();
        }

        if (driveCmd != null) {
            driveCmd.stop();
        }
    }
}

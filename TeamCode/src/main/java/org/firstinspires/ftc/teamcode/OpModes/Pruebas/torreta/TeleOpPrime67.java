
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV5;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV77;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretControlCommand;
import org.firstinspires.ftc.teamcode.Commands.intake.IntakeCommand;

@TeleOp(name = "TELEOP PRIME 67")
public class TeleOpPrime67 extends OpMode {

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

    @Override
    public void init() {

        // DRIVE
        drive = new DriveSubsystem(hardwareMap);
        drive.setInitialPose(48, 6, 90);

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

        telemetry.addLine("PRIME 67 LISTO");
        telemetry.addLine("B: TRACKING COMBINED");
        telemetry.addLine("START: HOME");
        telemetry.addLine("BACK: RESET FAULT");
        telemetry.addLine("AUTOTAG SIEMPRE ACTIVO");
        telemetry.update();
    }

    @Override
    public void start() {
        turretCmd.start();
    }

    @Override
    public void loop() {

        // DRIVE
        driveCmd.execute();
        drive.update();

        // TURRET
        turretCmd.execute(
                gamepad1.b,
                gamepad1.start,
                gamepad1.back
        );

        // =========================================
        // INTAKE
        // =========================================
        intakeCmd.execute(
                gamepad1.y,  // Auto Intake ON/OFF
                false,       // Shooter sin activar
                gamepad1.a,  // Reversa manual
                gamepad2.y   // Reset contador
        );


        // TELEMETRIA
        telemetry.addLine("===== PRIME 67 =====");

        telemetry.addData("Zona", turretCmd.getFieldSide());
        telemetry.addData("Tag ID", turretCmd.getActiveTagId());

        telemetry.addData("Mode", aim.getMode());
        telemetry.addData("Status", aim.getStatus());
        telemetry.addData("Tracking", turretCmd.isTracking());

        telemetry.addLine("===== TURRET =====");

        telemetry.addData("Angle", "%.2f", turret.getAngle());
        telemetry.addData("Target", "%.2f", turret.getTarget());
        telemetry.addData("Error", "%.2f", turret.getError());
        telemetry.addData("Power", "%.3f", turret.getPower());
        telemetry.addData("Fault", turret.getFaultReason());
        telemetry.addData("Cable Limit", turret.isAtLimit());

        telemetry.addLine("===== PEDRO + VISION =====");

        telemetry.addData(
                "Robot X/Y", "%.1f / %.1f",
                drive.getX(), drive.getY()
        );

        telemetry.addData(
                "Heading", "%.2f",
                drive.getHeadingDegrees()
        );

        telemetry.addData(
                "Vision Offset", "%.2f",
                aim.getVisionOffset()
        );

        telemetry.addData("Tag Visible", vision.hasTarget());
        telemetry.addData("Tx", "%.2f", vision.getTx());
        telemetry.addData("Age ms", "%.0f", vision.getAgeMs());

        telemetry.addLine("===== INTAKE =====");

        telemetry.addData(
                "Auto Intake",
                intake.isAutoIntakeEnabled()
        );

        telemetry.addData(
                "Ball Count",
                "%d / 4",
                intake.getBallCount()
        );

        telemetry.addData(
                "Gate",
                intake.isGateOpen() ? "OPEN" : "CLOSED"
        );

        telemetry.addData(
                "Bottom V2",
                "%s | %.2f cm | Alpha %d",
                intake.isBottomDetected(),
                intake.getBottomCm(),
                intake.getBottomAlpha()
        );

        telemetry.addData(
                "Top V3",
                "%s | %.2f cm | Alpha %d",
                intake.isTopDetected(),
                intake.getTopCm(),
                intake.getTopAlpha()
        );

        telemetry.update();
    }

    @Override
    public void stop() {

        if (turretCmd != null) {
            turretCmd.stop();
        }

        if (intakeCmd != null) {
            intakeCmd.stop();
        }

        if (driveCmd != null) {
            driveCmd.stop();
        }
    }
}

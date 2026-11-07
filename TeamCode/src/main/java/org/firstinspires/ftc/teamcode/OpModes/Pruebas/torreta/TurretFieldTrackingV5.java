
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimingV5;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV4;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV3;

@TeleOp(name = "THUNDERROAR - TRACKING V5")
public class TurretFieldTrackingV5 extends OpMode {

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCommand;

    private TurretSubsystemV3 turret;
    private TurretVisionV4 vision;
    private TurretAimingV5 aiming;

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastVision = false;
    private boolean lastReset = false;

    private String mode = "IDLE";

    @Override
    public void init() {

        drive = new DriveSubsystem(hardwareMap);

        drive.setInitialPose(0, 0, 0);

        driveCommand = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );

        turret = new TurretSubsystemV3(hardwareMap);

        turret.setMaxPower(0.50);

        vision = new TurretVisionV4(hardwareMap);

        aiming = new TurretAimingV5(
                drive,
                turret,
                vision
        );

        aiming.setVisionEnabled(false);

        telemetry.addLine("TRACKING V5");
        telemetry.addLine("B = Tracking ON/OFF");
        telemetry.addLine("A = Stop");
        telemetry.addLine("X = Turret 0");
        telemetry.addLine("Y = Turret +90");
        telemetry.addLine("Gamepad 2 X = Vision ON/OFF");
        telemetry.addLine("Gamepad 2 START = Clear Fault");
        telemetry.update();
    }

    @Override
    public void start() {
        vision.start();
    }

    private void moveTo(double angle) {

        aiming.disable();

        if (turret.isStalled()) {
            mode = "FAULT";
            return;
        }

        turret.enable();
        turret.setTarget(angle);
        mode = "POSITION";
    }

    @Override
    public void loop() {

        driveCommand.execute();
        drive.update();

        vision.update();

        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        boolean visionButton = gamepad2.x;
        boolean reset = gamepad2.start;

        // ==================================
        // DETENER
        // ==================================

        if (a && !lastA) {
            aiming.disable();
            turret.disable();
            mode = "IDLE";
        }

        // ==================================
        // TRACKING
        // ==================================

        if (b && !lastB) {

            if (aiming.isEnabled()) {

                aiming.disable();
                mode = "IDLE";

            } else if (turret.isStalled()) {

                mode = "FAULT";

            } else {

                boolean success = aiming.enable();

                mode = success
                        ? "TRACKING"
                        : "START FAILED";
            }
        }

        // ==================================
        // POSICIONES
        // ==================================

        if (x && !lastX) {
            moveTo(0);
        }

        if (y && !lastY) {
            moveTo(90);
        }

        // ==================================
        // VISION ON/OFF
        // ==================================

        if (visionButton && !lastVision) {

            aiming.setVisionEnabled(
                    !aiming.isVisionEnabled()
            );
        }

        // ==================================
        // REINICIAR FALLA
        // ==================================

        if (reset && !lastReset) {

            aiming.disable();
            turret.clearFault();
            mode = "IDLE";
        }

        lastA = a;
        lastB = b;
        lastX = x;
        lastY = y;
        lastVision = visionButton;
        lastReset = reset;

        // ==================================
        // UPDATE
        // ==================================

        if (aiming.isEnabled()) {
            aiming.update();
        } else {
            turret.update();
        }

        if (turret.isStalled()) {
            mode = "FAULT";
        }

        // ==================================
        // TELEMETRIA
        // ==================================

        telemetry.addLine("===== ROBOT =====");

        telemetry.addData(
                "X", "%.2f", drive.getX()
        );

        telemetry.addData(
                "Y", "%.2f", drive.getY()
        );

        telemetry.addData(
                "Heading", "%.2f",
                drive.getHeadingDegrees()
        );

        telemetry.addLine("===== TURRET =====");

        telemetry.addData("Mode", mode);

        telemetry.addData(
                "Target", "%.2f",
                turret.getTarget()
        );

        telemetry.addData(
                "Actual", "%.2f",
                turret.getAngle()
        );

        telemetry.addData(
                "Error", "%.2f",
                turret.getError()
        );

        telemetry.addData(
                "Power", "%.3f",
                turret.getPower()
        );

        telemetry.addData(
                "Max Power", "%.2f",
                turret.getMaxPower()
        );

        telemetry.addData(
                "Stall", turret.isStalled()
        );

        telemetry.addData(
                "Fault", turret.getFaultReason()
        );

        telemetry.addLine("===== LIMELIGHT =====");

        telemetry.addData(
                "Tag 36 Visible",
                vision.hasTarget()
        );

        telemetry.addData(
                "Tx", "%.2f",
                vision.getTx()
        );

        telemetry.addData(
                "Ty", "%.2f",
                vision.getTy()
        );

        telemetry.addLine("===== TRACKING V5 =====");

        telemetry.addData(
                "Enabled", aiming.isEnabled()
        );

        telemetry.addData(
                "Vision Enabled",
                aiming.isVisionEnabled()
        );

        telemetry.addData(
                "Heading Change", "%.2f",
                aiming.getHeadingChange()
        );

        telemetry.addData(
                "Desired", "%.2f",
                aiming.getDesiredAngle()
        );

        telemetry.addData(
                "Commanded", "%.2f",
                aiming.getCommandedAngle()
        );

        telemetry.addData(
                "Vision Offset", "%.2f",
                aiming.getVisionOffset()
        );

        telemetry.addData(
                "Vision Correction", "%.3f",
                aiming.getVisionCorrection()
        );

        telemetry.update();
    }

    @Override
    public void stop() {

        if (aiming != null) aiming.disable();
        if (turret != null) turret.stop();
        if (vision != null) vision.stop();
        if (drive != null) drive.stop();
    }
}

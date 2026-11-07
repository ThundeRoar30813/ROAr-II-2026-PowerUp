
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimingV4;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV4;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV3;

@TeleOp(name = "THUNDERROAR - FIELD TRACKING V4")
public class TurretFieldTrackingV4 extends OpMode {

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCommand;

    private TurretSubsystemV3 turret;
    private TurretVisionV4 vision;
    private TurretAimingV4 aiming;

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastReset = false;

    private String mode = "IDLE";

    @Override
    public void init() {

        drive = new DriveSubsystem(hardwareMap);

        // Origen relativo para pruebas.
        drive.setInitialPose(0, 0, 0);

        driveCommand = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );

        turret = new TurretSubsystemV3(hardwareMap);

        // Potencia maxima del 50%.
        turret.setMaxPower(0.50);

        vision = new TurretVisionV4(hardwareMap);

        aiming = new TurretAimingV4(
                drive,
                turret,
                vision
        );

        telemetry.addLine("FIELD TRACKING V4");
        telemetry.addLine("B: Tracking ON/OFF");
        telemetry.addLine("A: Stop turret");
        telemetry.addLine("X: 0 grados");
        telemetry.addLine("Y: +90 grados");
        telemetry.addLine("Gamepad 2 START: Clear Fault");
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

        // Actualizar odometria y conduccion.
        driveCommand.execute();
        drive.update();

        // Actualizar Limelight.
        vision.update();

        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        boolean reset = gamepad2.start;

        // =================================
        // STOP
        // =================================

        if (a && !lastA) {
            aiming.disable();
            turret.disable();
            mode = "IDLE";
        }

        // =================================
        // TRACKING
        // =================================

        if (b && !lastB) {

            if (aiming.isEnabled()) {

                aiming.disable();
                mode = "IDLE";

            } else if (turret.isStalled()) {

                mode = "FAULT";

            } else if (!vision.hasTarget()) {

                mode = "TAG 36 NOT VISIBLE";

            } else if (Math.abs(vision.getTx()) > 3.0) {

                mode = "CENTER TAG FIRST";

            } else {

                boolean started = aiming.enable();

                mode = started
                        ? "TRACKING"
                        : "START FAILED";
            }
        }

        // =================================
        // POSICIONES MANUALES
        // =================================

        if (x && !lastX) {
            moveTo(0);
        }

        if (y && !lastY) {
            moveTo(90);
        }

        // =================================
        // REINICIAR FALLA
        // =================================

        if (reset && !lastReset) {

            aiming.disable();
            turret.clearFault();

            mode = "IDLE";
        }

        lastA = a;
        lastB = b;
        lastX = x;
        lastY = y;
        lastReset = reset;

        // =================================
        // ACTUALIZAR TORRETA
        // =================================

        if (aiming.isEnabled()) {
            aiming.update();
        } else {
            turret.update();
        }

        if (turret.isStalled()) {
            mode = "FAULT";
        }

        // =================================
        // TELEMETRIA
        // =================================

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
                "Target", "%.2f", turret.getTarget()
        );

        telemetry.addData(
                "Actual", "%.2f", turret.getAngle()
        );

        telemetry.addData(
                "Error", "%.2f", turret.getError()
        );

        telemetry.addData(
                "Power", "%.3f", turret.getPower()
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
                "Tag 36 Visible", vision.hasTarget()
        );

        telemetry.addData(
                "Tx", "%.2f", vision.getTx()
        );

        telemetry.addData(
                "Ty", "%.2f", vision.getTy()
        );

        telemetry.addData(
                "Age ms", vision.getAgeMs()
        );

        telemetry.addLine("===== TRACKING =====");

        telemetry.addData(
                "Enabled", aiming.isEnabled()
        );

        telemetry.addData(
                "Target Angle", "%.2f",
                aiming.getTargetAngle()
        );

        telemetry.addData(
                "Heading Delta", "%.2f",
                aiming.getHeadingDelta()
        );

        telemetry.addData(
                "Vision Correction", "%.3f",
                aiming.getVisionCorrection()
        );

        telemetry.update();
    }

    @Override
    public void stop() {

        if (aiming != null) {
            aiming.disable();
        }

        if (turret != null) {
            turret.stop();
        }

        if (vision != null) {
            vision.stop();
        }

        if (drive != null) {
            drive.stop();
        }
    }
}

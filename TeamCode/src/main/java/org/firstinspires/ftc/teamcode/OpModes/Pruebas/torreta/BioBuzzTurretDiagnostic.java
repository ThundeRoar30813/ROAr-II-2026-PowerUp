
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;

import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV5;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV76;

@TeleOp(name = "DIAGNOSTICO TURRET V7.4")
public class BioBuzzTurretDiagnostic extends OpMode {

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCmd;

    private TurretSubsystemV4 turret;
    private TurretVisionV5 vision;
    private TurretAimControllerV76 aim;

    // Pon el ID real de tu AprilTag
    public static int TEST_TAG_ID = 34;

    // Mantener potencia limitada
    public static double TEST_POWER = 0.85;

    private boolean tracking = false;
    private boolean home = false;

    private boolean lastB = false;
    private boolean lastL1 = false;
    private boolean lastR1 = false;

    private boolean lastLeft = false;
    private boolean lastRight = false;
    private boolean lastUp = false;

    @Override
    public void init() {

        drive = new DriveSubsystem(hardwareMap);

        // Pose provisional para diagnostico
        drive.setInitialPose(55, 2, 0);

        driveCmd = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );

        turret = new TurretSubsystemV4(hardwareMap);
        turret.maxPower = TEST_POWER;

        vision = new TurretVisionV5(hardwareMap);

        aim = new TurretAimControllerV76(
                drive,
                turret,
                vision
        );

        aim.setMode(
                TurretAimControllerV76.Mode.VISION_ONLY
        );

        telemetry.addLine("V7.4 FAST FUSION");
        telemetry.addLine("IZQUIERDA: SOLO LIMELIGHT");
        telemetry.addLine("DERECHA: SOLO PEDRO");
        telemetry.addLine("ARRIBA: FAST FUSION");
        telemetry.addLine("B: TRACKING ON/OFF");
        telemetry.addLine("L1: HOME");
        telemetry.addLine("R1: RESET FAULT");
        telemetry.addData("AprilTag", TEST_TAG_ID);
        telemetry.update();
    }

    @Override
    public void start() {
        vision.start();
    }

    @Override
    public void loop() {

        // ==================================
        // DRIVE
        // ==================================

        driveCmd.execute();
        drive.update();

        // ==================================
        // LIMELIGHT
        // ==================================

        vision.setSelectedTag(TEST_TAG_ID);
        vision.update();

        // ==================================
        // SELECCION DE MODO
        // ==================================

        if (gamepad1.dpad_left && !lastLeft) {

            aim.setMode(
                    TurretAimControllerV76.Mode.VISION_ONLY
            );
        }

        if (gamepad1.dpad_right && !lastRight) {

            aim.setMode(
                    TurretAimControllerV76.Mode.PEDRO_ONLY
            );
        }

        if (gamepad1.dpad_up && !lastUp) {

            aim.setMode(
                    TurretAimControllerV76.Mode.COMBINED
            );
        }

        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        lastUp = gamepad1.dpad_up;

        // ==================================
        // TRACKING ON/OFF
        // ==================================

        if (gamepad1.b && !lastB) {

            if (tracking) {

                aim.disable();
                tracking = false;

            } else {

                home = false;

                aim.enable();
                tracking = aim.isEnabled();
            }
        }

        lastB = gamepad1.b;

        // ==================================
        // HOME
        // ==================================

        if (gamepad1.left_bumper && !lastL1) {

            aim.disable();
            tracking = false;

            turret.enableHold();
            turret.setTarget(0);

            home = turret.isEnabled();
        }

        lastL1 = gamepad1.left_bumper;

        // ==================================
        // RESET FAULT
        // ==================================

        if (gamepad1.right_bumper && !lastR1) {

            aim.disable();

            tracking = false;
            home = false;

            turret.clearFault();
        }

        lastR1 = gamepad1.right_bumper;

        // ==================================
        // UPDATE TURRET
        // ==================================

        if (tracking) {

            aim.update();

            if (!aim.isEnabled()
                    || turret.isFault()) {

                tracking = false;
            }

        } else if (home) {

            turret.setVelocityFeedforward(0);
            turret.update();

            if (turret.isFault()) {

                home = false;

            } else if (
                    Math.abs(turret.getError()) <= 1.0
                            && Math.abs(turret.getVelocity()) < 8.0) {

                turret.disable();
                home = false;
            }

        } else {

            turret.update();
        }

        // ==================================
        // TELEMETRIA
        // ==================================

        telemetry.addLine("===== V7.4 =====");

        telemetry.addData("Mode", aim.getMode());
        telemetry.addData("Tracking", tracking);
        telemetry.addData("Status", aim.getStatus());

        telemetry.addLine("===== TURRET =====");

        telemetry.addData(
                "Actual", "%.2f",
                turret.getAngle()
        );

        telemetry.addData(
                "Target", "%.2f",
                turret.getTarget()
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
                "Velocity", "%.2f",
                turret.getVelocity()
        );

        telemetry.addData(
                "Base Target", "%.2f",
                aim.getBaseTarget()
        );

        telemetry.addData(
                "Smooth Target", "%.2f",
                aim.getSmoothTarget()
        );

        telemetry.addData(
                "Reference Velocity", "%.2f",
                aim.getSmoothVelocity()
        );

        telemetry.addData(
                "Feedforward", "%.2f",
                turret.getVelocityFeedforward()
        );

        telemetry.addData(
                "Fault",
                turret.getFaultReason()
        );

        telemetry.addLine("===== PEDRO =====");

        telemetry.addData(
                "Heading", "%.2f",
                drive.getHeadingDegrees()
        );

        telemetry.addData(
                "Chassis Rate", "%.2f",
                aim.getChassisRate()
        );

        telemetry.addData(
                "Vision Rate", "%.2f",
                aim.getVisionRate()
        );

        telemetry.addData(
                "Chassis Rate", "%.2f",
                aim.getChassisRate()
        );

        telemetry.addLine("===== LIMELIGHT =====");

        telemetry.addData(
                "Selected Tag", TEST_TAG_ID
        );

        telemetry.addData(
                "Visible", vision.hasTarget()
        );

        telemetry.addData(
                "Raw Tx", "%.2f",
                vision.getTx()
        );

        telemetry.addData(
                "Filtered Tx", "%.2f",
                aim.getFilteredTx()
        );

        telemetry.addData(
                "Age", "%.0f",
                vision.getAgeMs()
        );

        telemetry.addData(
                "New Frame",
                vision.isNewFrame()
        );

        telemetry.addData(
                "Frame Sequence",
                vision.getFrameSequence()
        );

        telemetry.update();
    }

    @Override
    public void stop() {

        if (aim != null) {
            aim.disable();
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

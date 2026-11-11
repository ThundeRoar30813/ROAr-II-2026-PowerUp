
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.turret.ZoneTargetSelector;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV5;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV7;

@TeleOp(name = "BIOBUZZ V7 - VISION + SHOOTER")
public class BioBuzzVisionShooterV7 extends OpMode {

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCmd;

    private TurretSubsystemV4 turret;
    private TurretVisionV5 vision;
    private TurretAimControllerV7 aim;
    private ZoneTargetSelector zones;

    private DcMotorEx shooter;

    private boolean shooterOn = false;
    private boolean trackingOn = false;
    private boolean homeOn = false;

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastL1 = false;
    private boolean lastR1 = false;
    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastLeft = false;
    private boolean lastRight = false;

    // FTC setVelocity utiliza ticks/segundo.
    private double shooterVelocity = 1100;
    private long lastSpeedNs = 0;

    // Limite inicial de prueba.
    // No subir hasta verificar feedforward.
    public static double TEST_TURRET_POWER = 0.35;

    @Override
    public void init() {

        drive = new DriveSubsystem(hardwareMap);

        // Pose provisional: ajustar a posicion real.
        drive.setInitialPose(72, 72, 0);

        driveCmd = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );

        turret = new TurretSubsystemV4(hardwareMap);
        turret.maxPower = TEST_TURRET_POWER;

        vision = new TurretVisionV5(hardwareMap);

        aim = new TurretAimControllerV7(
                drive, turret, vision
        );

        zones = new ZoneTargetSelector();

        shooter = hardwareMap.get(
                DcMotorEx.class, "shoot"
        );

        shooter.setDirection(
                DcMotor.Direction.FORWARD
        );

        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        shooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        shooter.setVelocityPIDFCoefficients(
                8.0, 0.0, 0.5, 18.0
        );

        shooter.setPower(0);

        telemetry.addLine("BIOBUZZ V7 READY");
        telemetry.addLine("VISION + SHOOTER TEST");
        telemetry.addLine("Turret must face front at INIT");
        telemetry.addData(
                "Turret Power Limit",
                turret.maxPower
        );
        telemetry.update();
    }

    @Override
    public void start() {
        vision.start();
    }

    @Override
    public void loop() {

        long now = System.nanoTime();

        // ==================================
        // DRIVE / PEDRO
        // ==================================

        driveCmd.execute();
        drive.update();

        // ==================================
        // ZONAS
        // ==================================

        zones.update(drive.getX(), drive.getY());

        // X: cambiar alianza
        if (gamepad1.x && !lastX) {

            if (zones.getAlliance() ==
                    ZoneTargetSelector.Alliance.RED) {

                zones.setAlliance(
                        ZoneTargetSelector.Alliance.BLUE
                );

            } else {

                zones.setAlliance(
                        ZoneTargetSelector.Alliance.RED
                );
            }

            zones.setAutomatic(true);
        }

        lastX = gamepad1.x;

        if (gamepad1.dpad_up && !lastUp) {
            zones.setAutomatic(true);
        }

        if (gamepad1.dpad_left && !lastLeft) {
            zones.selectLeft();
        }

        if (gamepad1.dpad_right && !lastRight) {
            zones.selectRight();
        }

        if (gamepad1.dpad_down && !lastDown) {
            zones.selectUpperLeft();
        }

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;

        int tagId = zones.getTagId();

        vision.setSelectedTag(tagId);
        vision.update();
        aim.setSelectedTag(tagId);

        // ==================================
        // SHOOTER - A
        // ==================================

        if (gamepad1.a && !lastA) {
            shooterOn = !shooterOn;
        }

        lastA = gamepad1.a;

        // R2: aumentar velocidad
        // L2: disminuir velocidad
        if (now - lastSpeedNs > 200_000_000L) {

            if (gamepad1.right_trigger > 0.5) {

                shooterVelocity += 5;
                lastSpeedNs = now;

            } else if (gamepad1.left_trigger > 0.5) {

                shooterVelocity -= 5;
                lastSpeedNs = now;
            }
        }

        shooterVelocity = Range.clip(
                shooterVelocity, 0, 3000
        );

        if (shooterOn) {
            shooter.setVelocity(shooterVelocity);
        } else {
            shooter.setPower(0);
        }

        // ==================================
        // TRACKING - B
        // ==================================

        if (gamepad1.b && !lastB) {

            if (trackingOn) {

                aim.disable();
                trackingOn = false;

            } else {

                homeOn = false;

                aim.enable();
                trackingOn = aim.isEnabled();
            }
        }

        lastB = gamepad1.b;

        // ==================================
        // HOME - L1
        // ==================================

        if (gamepad1.left_bumper && !lastL1) {

            aim.disable();
            trackingOn = false;

            turret.enableHold();
            turret.setTarget(0);

            homeOn = turret.isEnabled();
        }

        lastL1 = gamepad1.left_bumper;

        // ==================================
        // RESET FAULT - R1
        // ==================================

        if (gamepad1.right_bumper && !lastR1) {

            aim.disable();
            trackingOn = false;
            homeOn = false;

            turret.clearFault();
        }

        lastR1 = gamepad1.right_bumper;

        // ==================================
        // TURRET UPDATE
        // ==================================

        if (trackingOn) {

            aim.update();

            if (turret.isFault() ||
                    !aim.isEnabled()) {

                trackingOn = false;
            }

        } else if (homeOn) {

            turret.setVelocityFeedforward(0);
            turret.update();

            if (turret.isFault() ||
                    (Math.abs(turret.getError()) <= 1.0
                            && Math.abs(turret.getVelocity()) < 8.0)) {

                turret.disable();
                homeOn = false;
            }

        } else {

            turret.setVelocityFeedforward(0);
            turret.update();
        }

        // ==================================
        // TELEMETRIA
        // ==================================

        telemetry.addLine("===== BIOBUZZ V7 =====");

        telemetry.addData(
                "Alliance", zones.getAlliance()
        );
        telemetry.addData(
                "Zone", zones.getZone()
        );
        telemetry.addData(
                "Tag", tagId
        );

        telemetry.addLine("===== PEDRO =====");

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

        telemetry.addData(
                "Tracking", trackingOn
        );
        telemetry.addData(
                "Aim Status", aim.getStatus()
        );
        telemetry.addData(
                "Actual", "%.2f", turret.getAngle()
        );
        telemetry.addData(
                "Target", "%.2f", turret.getTarget()
        );
        telemetry.addData(
                "Error", "%.2f", turret.getError()
        );
        telemetry.addData(
                "Power", "%.2f", turret.getPower()
        );
        telemetry.addData(
                "Max Power", "%.2f", turret.maxPower
        );
        telemetry.addData(
                "Velocity", "%.2f", turret.getVelocity()
        );
        telemetry.addData(
                "Chassis Rate", "%.2f",
                aim.getChassisAngularVelocity()
        );
//        telemetry.addData(
//                "Velocity FF", "%.2f",
//                turret.getVelocityFeedforward()
//        );
        telemetry.addData(
                "Heading Lead", "%.2f",
                aim.getHeadingLead()
        );
        telemetry.addData(
                "Vision Offset", "%.2f",
                aim.getVisionOffset()
        );
        telemetry.addData(
                "Fault", turret.getFaultReason()
        );

        telemetry.addLine("===== LIMELIGHT =====");

//        telemetry.addData(
//                "Vision Status", vision.getStatus()
//        );
//        telemetry.addData(
//                "Tag Visible", vision.hasTarget()
//        );
//        telemetry.addData(
//                "Detected Tag", vision.getDetectedTag()
//        );
        telemetry.addData(
                "Raw Tx", "%.2f", vision.getTx()
        );
        telemetry.addData(
                "Filtered Tx", "%.2f",
                aim.getFilteredTx()
        );
        telemetry.addData(
                "Frame Age", "%.0f",
                vision.getAgeMs()
        );

        telemetry.addLine("===== SHOOTER =====");

        telemetry.addData(
                "Shooter ON", shooterOn
        );
        telemetry.addData(
                "Target ticks/s", "%.0f",
                shooterVelocity
        );
        telemetry.addData(
                "Actual ticks/s", "%.0f",
                shooter.getVelocity()
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

        if (shooter != null) {
            shooter.setPower(0);
        }

        if (drive != null) {
            drive.stop();
        }
    }
}


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
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV6;

@TeleOp(name = "BIOBUZZ V5 - HYBRID TRACKING")
public class BioBuzzTurretTeleOp extends OpMode {

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCmd;

    private TurretSubsystemV4 turret;
    private TurretVisionV5 vision;
    private TurretAimControllerV6 aim;
    private ZoneTargetSelector zones;

    private DcMotorEx shooter;

    private boolean shooterOn = false;
    private boolean trackingOn = false;
    private boolean homeOn = false;
    private boolean hoodOpen = false;

    private boolean lastA, lastB, lastX, lastY;
    private boolean lastL1, lastR1;
    private boolean lastLeft, lastRight;
    private boolean lastUp, lastDown;

    private double shooterVelocity = 1100;
    private long lastSpeedNs = 0;

    @Override
    public void init() {

        drive = new DriveSubsystem(hardwareMap);

        // Pose provisional: reemplazar por
        // posicion inicial real del robot.
        drive.setInitialPose(72, 72, 0);

        driveCmd = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );

        turret = new TurretSubsystemV4(hardwareMap);

        // Primera prueba: limitar potencia.
        // Cuando compruebes signos y limites,
        // puedes subir progresivamente a 0.80.
        turret.maxPower = 0.85;

        vision = new TurretVisionV5(hardwareMap);

        aim = new TurretAimControllerV6(
                drive,
                turret,
                vision
        );

        zones = new ZoneTargetSelector();

        shooter = hardwareMap.get(
                DcMotorEx.class,
                "shoot"
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
                8.0,
                0.0,
                0.5,
                18.0
        );

        shooter.setPower(0);

        telemetry.addLine("BIOBUZZ V5 READY");
        telemetry.addLine("HYBRID PEDRO + LIMELIGHT");
        telemetry.addLine("TURRET FRONT BEFORE INIT");
        telemetry.addLine("FIRST TEST POWER: 25%");
        telemetry.update();
    }

    @Override
    public void start() {
        vision.start();
    }

    @Override
    public void loop() {

        // ======================================
        // DRIVE
        // ======================================

        driveCmd.execute();
        drive.update();

        // ======================================
        // ZONE SELECTION
        // ======================================

        zones.update(
                drive.getX(),
                drive.getY()
        );

        // Cuadrado: cambiar alianza
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

        // Cruceta
        if (gamepad1.dpad_left && !lastLeft) {
            zones.selectLeft();
        }

        if (gamepad1.dpad_right && !lastRight) {
            zones.selectRight();
        }

        if (gamepad1.dpad_up && !lastUp) {
            zones.setAutomatic(true);
        }

        if (gamepad1.dpad_down && !lastDown) {
            zones.selectUpperLeft();
        }

        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        int tagId = zones.getTagId();

        vision.setSelectedTag(tagId);
        vision.update();

        aim.setSelectedTag(tagId);

        // ======================================
        // SHOOTER
        // ======================================

        if (gamepad1.a && !lastA) {
            shooterOn = !shooterOn;
        }

        lastA = gamepad1.a;

        long now = System.nanoTime();

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
                shooterVelocity,
                0,
                3000
        );

        if (shooterOn) {
            shooter.setVelocity(shooterVelocity);
        } else {
            shooter.setPower(0);
        }

        // ======================================
        // TRACKING - CIRCULO
        // ======================================

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

        // ======================================
        // HOME - L1
        // ======================================

        if (gamepad1.left_bumper && !lastL1) {

            aim.disable();
            trackingOn = false;

            turret.enableHold();
            turret.setTarget(0);

            homeOn = turret.isEnabled();
        }

        lastL1 = gamepad1.left_bumper;

        // ======================================
        // RESET FAULT - R1
        // ======================================

        if (gamepad1.right_bumper && !lastR1) {

            aim.disable();

            trackingOn = false;
            homeOn = false;

            turret.clearFault();
        }

        lastR1 = gamepad1.right_bumper;

        // ======================================
        // HOOD - TRIANGULO
        // ======================================

        if (gamepad1.y && !lastY) {
            hoodOpen = !hoodOpen;
        }

        lastY = gamepad1.y;

        // ======================================
        // TURRET UPDATE
        // ======================================

        if (trackingOn) {

            aim.update();

            if (turret.isFault() ||
                    !aim.isEnabled()) {
                trackingOn = false;
            }

        } else if (homeOn) {

            turret.update();

            if (turret.isFault() ||
                    Math.abs(turret.getAngle()) <= 1.0) {

                turret.disable();
                homeOn = false;
            }

        } else {

            turret.update();
        }

        // ======================================
        // TELEMETRY
        // ======================================

        telemetry.addLine("===== BIOBUZZ V5 =====");

        telemetry.addData(
                "Alliance",
                zones.getAlliance()
        );

        telemetry.addData(
                "Zone",
                zones.getZone()
        );

        telemetry.addData(
                "Tag Mode",
                zones.getMode()
        );

        telemetry.addData(
                "Selected Tag",
                tagId
        );

        telemetry.addLine("===== PEDRO =====");

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
                "Heading",
                "%.2f",
                drive.getHeadingDegrees()
        );

        telemetry.addLine("===== TURRET =====");

        telemetry.addData(
                "Tracking",
                trackingOn
        );

        telemetry.addData(
                "Home",
                homeOn
        );

        telemetry.addData(
                "Aim Status",
                aim.getStatus()
        );

        telemetry.addData(
                "Base Target",
                "%.2f",
                aim.getBaseTarget()
        );

        telemetry.addData(
                "Vision Offset",
                "%.2f",
                aim.getVisionOffset()
        );

        telemetry.addData(
                "Actual Angle",
                "%.2f",
                turret.getAngle()
        );

        telemetry.addData(
                "Target Angle",
                "%.2f",
                turret.getTarget()
        );

        telemetry.addData(
                "Error",
                "%.2f",
                turret.getError()
        );

        telemetry.addData(
                "Power",
                "%.2f",
                turret.getPower()
        );

        telemetry.addData(
                "Max Power",
                "%.2f",
                turret.maxPower
        );

        telemetry.addData(
                "Velocity deg/s",
                "%.2f",
                turret.getVelocity()
        );

        telemetry.addData(
                "Fault",
                turret.getFaultReason()
        );


        telemetry.addLine("===== LIMELIGHT =====");

//        telemetry.addData(
//                "Vision Status",
//                vision.getStatus()
//        );
//
//        telemetry.addData(
//                "Result Valid",
//                vision.isResultValid()
//        );
//
//        telemetry.addData(
//                "Any Tag Visible",
//                vision.hasAnyTag()
//        );

        telemetry.addData(
                "Selected Tag",
                vision.getSelectedTag()
        );

//        telemetry.addData(
//                "Detected Tag",
//                vision.getDetectedTag()
//        );
//
//        telemetry.addData(
//                "Tag Count",
//                vision.getDetectedTagCount()
//        );

        telemetry.addData(
                "Tag Visible",
                vision.hasTarget()
        );

        telemetry.addData(
                "Raw Tx",
                "%.2f",
                vision.getTx()
        );

        telemetry.addData(
                "Frame Age ms",
                "%.0f",
                vision.getAgeMs()
        );


        telemetry.addLine("===== SHOOTER =====");

        telemetry.addData(
                "Shooter ON",
                shooterOn
        );

        telemetry.addData(
                "Target Velocity",
                "%.0f",
                shooterVelocity
        );

        telemetry.addData(
                "Actual Velocity",
                "%.0f",
                shooter.getVelocity()
        );

        telemetry.addData(
                "Hood Toggle",
                hoodOpen
        );

        telemetry.addData(
                "Hood Hardware",
                "NOT CONFIGURED"
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

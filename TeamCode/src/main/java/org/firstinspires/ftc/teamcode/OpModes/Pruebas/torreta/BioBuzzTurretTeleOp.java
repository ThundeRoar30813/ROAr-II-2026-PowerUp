
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;

import org.firstinspires.ftc.teamcode.Commands.turret.ZoneTargetSelector;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV5;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV4;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

@TeleOp(name = "BIOBUZZ - Turret V4 + Shooter")
public class BioBuzzTurretTeleOp extends OpMode {

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCmd;

    private TurretSubsystemV4 turret;
    private TurretVisionV5 vision;
    private TurretAimControllerV4 aim;
    private ZoneTargetSelector zones;

    private DcMotorEx shooter;

    private boolean shooterOn;
    private boolean trackingOn;
    private boolean homeOn;

    private boolean lastA;
    private boolean lastB;
    private boolean lastX2;
    private boolean lastB2;
    private boolean lastBack2;
    private boolean lastStart2;

    private long lastSpeedChangeNs;

    private double shooterVelocity = 1100;

    private double previousTx = Double.NaN;
    private double txJitter = 0;

    @Override
    public void init() {

        // =====================================
        // CHASIS
        // =====================================

        drive = new DriveSubsystem(hardwareMap);

        // Posicion inicial provisional.

        drive.setInitialPose(72, 72, 0);

        driveCmd = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );

        // =====================================
        // TURRET
        // =====================================

        turret = new TurretSubsystemV4(hardwareMap);

        vision = new TurretVisionV5(hardwareMap);

        aim = new TurretAimControllerV4(
                turret,
                vision
        );

        zones = new ZoneTargetSelector();

        // =====================================
        // SHOOTER
        // =====================================

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

        shooter.setPower(0);

        shooter.setVelocityPIDFCoefficients(
                8.0,
                0.0,
                0.5,
                18.0
        );

        telemetry.addLine(
                "BIOBUZZ TURRET V4 READY"
        );

        telemetry.addLine(
                "Place turret at 0 before INIT"
        );

        telemetry.addLine(
                "G1 Sticks: Drive"
        );

        telemetry.addLine(
                "G1 A: Shooter"
        );

        telemetry.addLine(
                "G1 B: Tracking"
        );

        telemetry.addLine(
                "G2 X: RED | B: BLUE"
        );

        telemetry.addLine(
                "G2 BACK: Home"
        );

        telemetry.addLine(
                "G2 START: Fault Reset"
        );

        telemetry.update();
    }

    @Override
    public void start() {
        vision.start();
    }

    @Override
    public void loop() {

        // =====================================
        // DRIVE
        // =====================================

        driveCmd.execute();
        drive.update();

        // =====================================
        // ZONE SELECTOR
        // =====================================

        zones.update(
                drive.getX(),
                drive.getY()
        );

        if (gamepad2.x && !lastX2) {
            zones.setAlliance(
                    ZoneTargetSelector.Alliance.RED
            );
        }

        if (gamepad2.b && !lastB2) {
            zones.setAlliance(
                    ZoneTargetSelector.Alliance.BLUE
            );
        }

        lastX2 = gamepad2.x;
        lastB2 = gamepad2.b;

        // =====================================
        // LIMELIGHT
        // =====================================

        vision.setSelectedTag(
                zones.getTagId()
        );

        vision.update();

        // =====================================
        // SHOOTER
        // =====================================

        if (gamepad1.a && !lastA) {
            shooterOn = !shooterOn;
        }

        lastA = gamepad1.a;

        // =====================================
        // TRACKING
        // =====================================

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

        // =====================================
        // FAULT RESET
        // =====================================

        if (gamepad2.start && !lastStart2) {

            aim.disable();

            trackingOn = false;
            homeOn = false;

            turret.clearFault();
        }

        lastStart2 = gamepad2.start;

        // =====================================
        // RETURN HOME
        // =====================================

        if (gamepad2.back && !lastBack2) {

            aim.disable();

            trackingOn = false;

            turret.enableHold();
            turret.setTarget(0);

            homeOn = turret.isEnabled();
        }

        lastBack2 = gamepad2.back;

        // =====================================
        // SHOOTER VELOCITY
        // =====================================

        long now = System.nanoTime();

        if (now - lastSpeedChangeNs >
                200_000_000L) {

            if (gamepad1.right_trigger > 0.5) {

                shooterVelocity += 5;
                lastSpeedChangeNs = now;

            } else if (gamepad1.left_trigger > 0.5) {

                shooterVelocity -= 5;
                lastSpeedChangeNs = now;
            }
        }

        shooterVelocity = Range.clip(
                shooterVelocity,
                0,
                3000
        );

        if (shooterOn) {

            shooter.setVelocity(
                    shooterVelocity
            );

        } else {

            shooter.setPower(0);
        }

        // =====================================
        // TURRET UPDATE
        // =====================================

        if (trackingOn) {

            aim.update();

            if (turret.isFault() ||
                    !aim.isEnabled()) {

                trackingOn = false;
            }

        } else if (homeOn) {

            turret.update();

            if (turret.isFault() ||
                    Math.abs(turret.getAngle()) <= 1) {

                turret.disable();
                homeOn = false;
            }

        } else {

            turret.update();
        }

        // =====================================
        // LIMELIGHT VIBRATION DIAGNOSTIC
        // =====================================

        if (vision.hasTarget()) {

            double tx = vision.getTx();

            if (Double.isFinite(previousTx)) {

                txJitter =
                        0.9 * txJitter
                                + 0.1 * Math.abs(
                                tx - previousTx
                        );
            }

            previousTx = tx;

        } else {

            previousTx = Double.NaN;
        }

        // =====================================
        // TELEMETRY
        // =====================================

        telemetry.addLine(
                "===== BIOBUZZ V4 ====="
        );

        telemetry.addData(
                "Alliance",
                zones.getAlliance()
        );

        telemetry.addData(
                "Zone",
                zones.getZone()
        );

        telemetry.addData(
                "Selected Tag",
                zones.getTagId()
        );

        telemetry.addData(
                "Pose X/Y",
                "%.1f / %.1f",
                drive.getX(),
                drive.getY()
        );

        telemetry.addData(
                "Heading",
                "%.1f",
                drive.getHeadingDegrees()
        );

        telemetry.addLine(
                "===== TURRET ====="
        );

        telemetry.addData(
                "Tracking",
                trackingOn
        );

        telemetry.addData(
                "Aim Status",
                aim.getStatus()
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
                "Velocity deg/s",
                "%.1f",
                turret.getVelocity()
        );

        telemetry.addData(
                "Fault",
                turret.getFaultReason()
        );

        telemetry.addLine(
                "===== LIMELIGHT ====="
        );

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
                "Filtered Tx",
                "%.2f",
                aim.getFilteredTx()
        );

        telemetry.addData(
                "Tx Jitter",
                "%.3f",
                txJitter
        );

        telemetry.addData(
                "Frame Age ms",
                "%.0f",
                vision.getAgeMs()
        );

        telemetry.addLine(
                "===== SHOOTER ====="
        );

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

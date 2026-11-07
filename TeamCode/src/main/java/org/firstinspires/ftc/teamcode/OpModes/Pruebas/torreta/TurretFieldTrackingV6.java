
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimingV7;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV4;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV3;

@TeleOp(name = "ThunderROAR V6 - Tracking + Shooter")
public class TurretFieldTrackingV6 extends OpMode {

    // ==========================================
    // CHASIS
    // ==========================================

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCommand;

    // ==========================================
    // TORRETA Y LIMELIGHT
    // ==========================================

    private TurretSubsystemV3 turret;
    private TurretVisionV4 vision;
    private TurretAimingV7 aiming;

    private boolean trackingEnabled = false;
    private boolean visionEnabled = false;

    // ==========================================
    // SHOOTER
    // ==========================================

    private DcMotorEx shooter;

    // Unidad: ticks por segundo
    private double targetVelocity = 1100.0;

    private double p = 8.0;
    private double i = 0.0;
    private double d = 0.5;
    private double f = 18.0;

    private boolean shooterEnabled = false;

    // ==========================================
    // BOTONES ANTERIORES
    // ==========================================

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastVisionX = false;
    private boolean lastStart = false;

    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastRight = false;
    private boolean lastLeft = false;

    private long lastVelocityChange = 0;

    // ==========================================
    // INIT
    // ==========================================

    @Override
    public void init() {

        // CHASIS
        drive = new DriveSubsystem(hardwareMap);

        driveCommand = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> gamepad1.left_stick_x,
                () -> gamepad1.right_stick_x
        );

        drive.setInitialPose(0, 0, 0);

        // TORRETA
        turret = new TurretSubsystemV3(hardwareMap);
        turret.setMaxPower(0.80);

        // LIMELIGHT
        vision = new TurretVisionV4(hardwareMap);

        // CONSTRUCTOR CORREGIDO
        aiming = new TurretAimingV7(
                drive,
                turret,
                vision
        );

        // SHOOTER
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

        applyPIDF();

        telemetry.addLine("THUNDERROAR V6");
        telemetry.addLine("Tracking + Limelight + Shooter");
        telemetry.addLine("INIT COMPLETADO");
        telemetry.update();
    }

    // ==========================================
    // START
    // ==========================================

    @Override
    public void start() {
        vision.start();
    }

    // ==========================================
    // LOOP
    // ==========================================

    @Override
    public void loop() {

        // ======================================
        // CHASIS
        // ======================================

        driveCommand.execute();
        drive.update();

        // ======================================
        // LIMELIGHT
        // ======================================

        vision.update();

        // ======================================
        // GAMEPAD 1 - A: SHOOTER
        // ======================================

        boolean a = gamepad1.a;

        if (a && !lastA) {
            shooterEnabled = !shooterEnabled;
        }

        lastA = a;

        // ======================================
        // GAMEPAD 1 - B: TRACKING
        // ======================================

        boolean b = gamepad1.b;

        if (b && !lastB) {

            if (trackingEnabled) {

                aiming.disable();
                trackingEnabled = false;

            } else {

                aiming.enable();
                trackingEnabled = turret.isEnabled();
            }
        }

        lastB = b;

        // ======================================
        // GAMEPAD 2 - X: VISION
        // ======================================

        boolean visionX = gamepad2.x;

        if (visionX && !lastVisionX) {

            visionEnabled = !visionEnabled;

            aiming.setVisionEnabled(
                    visionEnabled
            );
        }

        lastVisionX = visionX;

        // ======================================
        // GAMEPAD 2 - START: RESET FAULT
        // ======================================

        boolean startButton = gamepad2.start;

        if (startButton && !lastStart) {

            aiming.disable();
            trackingEnabled = false;

            turret.clearFault();
        }

        lastStart = startButton;

        // ======================================
        // SHOOTER - VELOCIDAD
        // ======================================

        long now = System.nanoTime();

        if (now - lastVelocityChange > 200_000_000L) {

            if (gamepad2.right_trigger > 0.5) {

                targetVelocity += 5.0;
                lastVelocityChange = now;

            } else if (gamepad2.left_trigger > 0.5) {

                targetVelocity -= 5.0;
                lastVelocityChange = now;
            }
        }

        targetVelocity = Range.clip(
                targetVelocity,
                0,
                3000
        );

        // ======================================
        // SHOOTER - PIDF TUNING
        // ======================================

        boolean up = gamepad2.dpad_up;
        boolean down = gamepad2.dpad_down;
        boolean right = gamepad2.dpad_right;
        boolean left = gamepad2.dpad_left;

        boolean pidChanged = false;

        // P +0.2
        if (up && !lastUp) {
            p += 0.2;
            pidChanged = true;
        }

        // F +0.5
        if (down && !lastDown) {
            f += 0.5;
            pidChanged = true;
        }

        // D +0.1
        if (right && !lastRight) {
            d += 0.1;
            pidChanged = true;
        }

        // I +0.01
        if (left && !lastLeft) {
            i += 0.01;
            pidChanged = true;
        }

        lastUp = up;
        lastDown = down;
        lastRight = right;
        lastLeft = left;

        if (pidChanged) {
            applyPIDF();
        }

        // ======================================
        // SHOOTER - CONTROL DE VELOCIDAD
        // ======================================

        if (shooterEnabled) {

            shooter.setVelocity(
                    targetVelocity
            );

        } else {

            shooter.setPower(0);
        }

        // ======================================
        // TORRETA - TRACKING
        // ======================================

        if (trackingEnabled) {

            aiming.update();

            if (!turret.isEnabled()
                    || turret.isStalled()) {

                trackingEnabled = false;
            }

        } else {

            turret.update();
        }

        // ======================================
        // TELEMETRIA
        // ======================================

        telemetry.addLine("=== THUNDERROAR V6 ===");

        telemetry.addLine("--- CHASIS ---");

        telemetry.addData(
                "Heading",
                "%.2f",
                drive.getHeadingDegrees()
        );

        telemetry.addLine("--- TRACKING ---");

        telemetry.addData(
                "Tracking Enabled",
                trackingEnabled
        );

        telemetry.addData(
                "Vision Enabled",
                visionEnabled
        );

        telemetry.addData(
                "Tag 36 Visible",
                vision.hasTarget()
        );

        telemetry.addData(
                "Tx",
                "%.2f",
                vision.getTx()
        );

        telemetry.addLine("--- Cosa nueva agregue ---");

        telemetry.addData("Tx filtrado", "%.2f",
                aiming.getFilteredTx());

        telemetry.addData("Vision Corrigiendo",
                aiming.isCorrectingVision());

        telemetry.addData("Vision Offset", "%.2f",
                aiming.getVisionOffset());
        telemetry.addData("Desired Angle", "%.2f",
                aiming.getDesiredAngle());

        telemetry.addData("Commanded Angle", "%.2f",
                aiming.getCommandedAngle());

        telemetry.addData("Turret Actual", "%.2f",
                turret.getAngle());

        telemetry.addData("Turret Error", "%.2f",
                turret.getError());


        telemetry.addData("Chassis Moving",
                aiming.isChassisMoving());

        telemetry.addData("Heading Rate", "%.2f",
                aiming.getHeadingRate());

        telemetry.addData("Vision Scale", "%.2f",
                aiming.getVisionScale());

        telemetry.addData("Tx Filtered", "%.2f",
                aiming.getFilteredTx());

        telemetry.addData("Vision Correcting",
                aiming.isCorrectingVision());

        telemetry.addData("Vision Correction", "%.2f",
                aiming.getVisionCorrection());
        telemetry.addLine("--- Neuvas  ---");

        telemetry.addLine("--- TURRET ---");

        telemetry.addData(
                "Target",
                "%.2f",
                turret.getTarget()
        );

        telemetry.addData(
                "Actual",
                "%.2f",
                turret.getAngle()
        );

        telemetry.addData(
                "Error",
                "%.2f",
                turret.getError()
        );

        telemetry.addData(
                "Power",
                "%.3f",
                turret.getPower()
        );

        telemetry.addData(
                "Stall",
                turret.isStalled()
        );

        telemetry.addData(
                "Fault",
                turret.getFaultReason()
        );

        telemetry.addLine("--- SHOOTER ---");

        telemetry.addData(
                "Shooter Enabled",
                shooterEnabled
        );

        telemetry.addData(
                "Target ticks/s",
                "%.1f",
                targetVelocity
        );

        telemetry.addData(
                "Actual ticks/s",
                "%.1f",
                shooter.getVelocity()
        );

        telemetry.addData("P", "%.3f", p);
        telemetry.addData("I", "%.3f", i);
        telemetry.addData("D", "%.3f", d);
        telemetry.addData("F", "%.3f", f);


        telemetry.addLine("=== DIAGNOSTICO HEADING ===");

        telemetry.addData("Drive Heading", "%.3f",
                drive.getHeadingDegrees());

        telemetry.addData("Aiming Heading Change", "%.3f",
                aiming.getHeadingChange());

        telemetry.addData("Heading Rate", "%.3f",
                aiming.getHeadingRate());

        telemetry.addData("Chassis Moving",
                aiming.isChassisMoving());

        telemetry.addData("Vision Scale", "%.3f",
                aiming.getVisionScale());

        telemetry.update();
    }

    // ==========================================
    // SHOOTER PIDF
    // ==========================================

    private void applyPIDF() {

        shooter.setVelocityPIDFCoefficients(
                p,
                i,
                d,
                f
        );
    }

    // ==========================================
    // STOP
    // ==========================================

    @Override
    public void stop() {

        shooter.setPower(0);

        aiming.disable();
        turret.stop();

        vision.stop();
        drive.stop();
    }
}


package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.TurretFieldTrackingCommand;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

@TeleOp(name = "ThunderROAR - Full TeleOp")
public class ThunderROARFullTeleOp extends OpMode {

    // =====================================================
    // CHASIS
    // =====================================================

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCommand;

    // =====================================================
    // TORRETA
    // =====================================================

    private TurretSubsystem turret;
    private TurretFieldTrackingCommand tracking;

    private static final double TARGET_X = 57.0;
    private static final double TARGET_Y = 59.0;

    // =====================================================
    // SHOOTER
    // =====================================================

    private DcMotorEx shooter;

    private double targetVelocity = 800;

    private double p = 4.0;
    private double i = 0.0;
    private double d = 0.5;
    private double f = 12.0;

    private boolean shooterEnabled = false;

    // =====================================================
    // INTAKE / INDEXER
    // =====================================================

    private DcMotor intake;
    private DcMotor indexer;

    // =====================================================
    // MEMORIA DE BOTONES
    // =====================================================

    private boolean previousA = false;
    private boolean previousB = false;

    private boolean previousUp = false;
    private boolean previousDown = false;
    private boolean previousLeft = false;
    private boolean previousRight = false;

    // =====================================================
    // INIT
    // =====================================================

    @Override
    public void init() {

        // CHASIS
        drive = new DriveSubsystem(hardwareMap);

        driveCommand = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );

        // TORRETA
        turret = new TurretSubsystem(hardwareMap);

        // Potencia reducida para verificar movimiento
        turret.setMaxPower(0.20);

        tracking = new TurretFieldTrackingCommand(
                drive,
                turret,
                TARGET_X,
                TARGET_Y
        );

        // SHOOTER
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

        shooter.setPower(0);

        applyPIDF();

        // INTAKE
        indexer = hardwareMap.get(
                DcMotor.class, "intake1"
        );

        intake = hardwareMap.get(
                DcMotor.class, "intake2"
        );

        indexer.setPower(0);
        intake.setPower(0);

        intake.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        // TELEMETRIA INICIAL
        telemetry.addLine("===== ThunderROAR FULL TELEOP =====");
        telemetry.addLine("A = Shooter ON/OFF");
        telemetry.addLine("B = Turret Tracking ON/OFF");
        telemetry.addLine("X = Intake adelante");
        telemetry.addLine("Y = Intake reversa");
        telemetry.addLine("RT = subir shooter velocity");
        telemetry.addLine("LT = bajar shooter velocity");
        telemetry.addLine("DPAD UP = P +");
        telemetry.addLine("DPAD DOWN = F +");
        telemetry.addLine("DPAD RIGHT = D +");
        telemetry.addLine("DPAD LEFT = I +");

        telemetry.addData(
                "Target",
                "(%.1f, %.1f)",
                TARGET_X,
                TARGET_Y
        );

        telemetry.update();
    }

    // =====================================================
    // LOOP
    // =====================================================

    @Override
    public void loop() {

        // =================================================
        // 1. CHASIS
        // =================================================

        driveCommand.execute();
        drive.update();

        // =================================================
        // 2. SHOOTER TOGGLE
        // =================================================

        boolean currentA = gamepad1.a;

        if (currentA && !previousA) {
            shooterEnabled = !shooterEnabled;
        }

        previousA = currentA;

        // =================================================
        // 3. AJUSTAR VELOCIDAD SHOOTER
        // =================================================

        targetVelocity +=
                gamepad1.right_trigger * 5.0;

        targetVelocity -=
                gamepad1.left_trigger * 5.0;

        targetVelocity = Math.max(
                0,
                Math.min(6000, targetVelocity)
        );

        // =================================================
        // 4. APLICAR SHOOTER
        // =================================================

        if (shooterEnabled) {

            shooter.setVelocity(targetVelocity);

        } else {

            shooter.setVelocity(0);
        }

        // =================================================
        // 5. INTAKE / INDEXER
        // =================================================

        if (gamepad1.x) {

            intake.setPower(1.00);
            indexer.setPower(1.00);

        } else if (gamepad1.y) {

            intake.setPower(-1.00);
            indexer.setPower(-1.00);

        } else {

            intake.setPower(0);
            indexer.setPower(0);
        }

        // =================================================
        // 6. TRACKING TOGGLE - BOTON B
        // =================================================

        boolean currentB = gamepad1.b;

        if (currentB && !previousB) {

            if (tracking.isEnabled()) {

                tracking.disable();

            } else {

                // Antes de activar, apuntar fisicamente
                // la torreta hacia el objetivo.
                tracking.enable();
            }
        }

        previousB = currentB;

        // =================================================
        // 7. EJECUTAR TRACKING
        // =================================================

        tracking.execute();

        // =================================================
        // 8. AJUSTES PIDF SHOOTER
        // =================================================

        boolean currentUp = gamepad1.dpad_up;
        boolean currentDown = gamepad1.dpad_down;
        boolean currentRight = gamepad1.dpad_right;
        boolean currentLeft = gamepad1.dpad_left;

        // P
        if (currentUp && !previousUp) {

            p += 0.2;
            applyPIDF();
        }

        // F
        if (currentDown && !previousDown) {

            f += 0.5;
            applyPIDF();
        }

        // D
        if (currentRight && !previousRight) {

            d += 0.1;
            applyPIDF();
        }

        // I
        if (currentLeft && !previousLeft) {

            i += 0.01;
            applyPIDF();
        }

        previousUp = currentUp;
        previousDown = currentDown;
        previousRight = currentRight;
        previousLeft = currentLeft;

        // =================================================
        // 9. TELEMETRIA CHASIS
        // =================================================

        telemetry.addLine("========== ROBOT ==========");

        telemetry.addData(
                "X", "%.2f in", drive.getX()
        );

        telemetry.addData(
                "Y", "%.2f in", drive.getY()
        );

        telemetry.addData(
                "Heading",
                "%.2f deg",
                drive.getHeadingDegrees()
        );

        // =================================================
        // TELEMETRIA TORRETA
        // =================================================

        telemetry.addLine("========== TURRET ==========");

        telemetry.addData(
                "Tracking",
                tracking.isEnabled() ? "ON" : "OFF"
        );

        telemetry.addData(
                "Target X",
                "%.2f",
                tracking.getTargetX()
        );

        telemetry.addData(
                "Target Y",
                "%.2f",
                tracking.getTargetY()
        );

        telemetry.addData(
                "Distance",
                "%.2f in",
                tracking.getDistanceToTarget()
        );

        telemetry.addData(
                "Field Angle",
                "%.2f deg",
                tracking.getFieldAngleDeg()
        );

        telemetry.addData(
                "Target Angle",
                "%.2f deg",
                turret.getTargetAngleDeg()
        );

        telemetry.addData(
                "Actual Angle",
                "%.2f deg",
                turret.getCurrentAngleDeg()
        );

        telemetry.addData(
                "Turret Error",
                "%.2f deg",
                turret.getErrorDeg()
        );

        telemetry.addData(
                "Turret Power",
                "%.3f",
                turret.getMotorPower()
        );

        telemetry.addData(
                "Turret Control",
                turret.isControlEnabled()
        );

        telemetry.addData(
                "Turret Stall",
                turret.isStalled()
        );

        telemetry.addData(
                "Turret Max Power",
                "%.2f",
                turret.getMaxPower()
        );

        telemetry.addData(
                "Turret Unwrapped Target",
                "%.2f",
                tracking.getUnwrappedDesiredAngle()
        );

        telemetry.addData(
                "Turret Ticks",
                turret.getRelativeTicks()
        );

        // =================================================
        // TELEMETRIA SHOOTER
        // =================================================

        telemetry.addLine("========== SHOOTER ==========");

        telemetry.addData(
                "Shooter",
                shooterEnabled ? "ON" : "OFF"
        );

        telemetry.addData(
                "Target Velocity",
                "%.1f ticks/s",
                targetVelocity
        );

        telemetry.addData(
                "Actual Velocity",
                "%.1f ticks/s",
                shooter.getVelocity()
        );

        telemetry.addData(
                "Velocity Error",
                "%.1f ticks/s",
                targetVelocity - shooter.getVelocity()
        );

        // =================================================
        // TELEMETRIA PIDF
        // =================================================

        telemetry.addLine("========== PIDF ==========");

        telemetry.addData("P", "%.3f", p);
        telemetry.addData("I", "%.3f", i);
        telemetry.addData("D", "%.3f", d);
        telemetry.addData("F", "%.3f", f);

        // =================================================
        // TELEMETRIA INTAKE
        // =================================================

        telemetry.addLine("========== INTAKE ==========");

        if (gamepad1.x) {

            telemetry.addData("Intake", "FORWARD");

        } else if (gamepad1.y) {

            telemetry.addData("Intake", "REVERSE");

        } else {

            telemetry.addData("Intake", "OFF");
        }

        telemetry.update();
    }

    // =====================================================
    // PIDF SHOOTER
    // =====================================================

    private void applyPIDF() {

        shooter.setVelocityPIDFCoefficients(
                p, i, d, f
        );
    }

    // =====================================================
    // STOP
    // =====================================================

    @Override
    public void stop() {

        if (shooter != null) {

            shooter.setVelocity(0);
            shooter.setPower(0);
        }

        if (intake != null) {
            intake.setPower(0);
        }

        if (indexer != null) {
            indexer.setPower(0);
        }

        if (tracking != null) {
            tracking.disable();
        }

        if (turret != null) {
            turret.stop();
        }

        if (driveCommand != null) {

            //driveCommand.end();

        } else if (drive != null) {

            drive.stop();
        }
    }
}

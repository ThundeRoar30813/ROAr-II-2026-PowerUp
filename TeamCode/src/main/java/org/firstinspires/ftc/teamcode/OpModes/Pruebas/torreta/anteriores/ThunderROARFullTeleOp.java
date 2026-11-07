
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta.anteriores;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.TurretFieldTrackingCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

@TeleOp(name = "ThunderROAR - Full TeleOp SIN SENSORES")
public class ThunderROARFullTeleOp extends OpMode {

    // CHASIS
    private DriveSubsystem drive;
    private MecanumDriveCmd driveCommand;

    // TORRETA
    private TurretSubsystem turret;
    private TurretFieldTrackingCommand tracking;

    // Coordenadas de la versión anterior.
    // Verificar que correspondan al objetivo real.
    private static final double TARGET_X = 57.0;
    private static final double TARGET_Y = 59.0;

    // SHOOTER
    private DcMotorEx shooter;

    private double targetVelocity = 800.0;

    private double p = 4.0;
    private double i = 0.0;
    private double d = 0.5;
    private double f = 12.0;

    private boolean shooterEnabled = false;

    // INTAKE
    private DcMotor intake;
    private DcMotor indexer;

    private static final double INTAKE_POWER = 1.0;
    private static final double INDEXER_POWER = 1.0;

    private boolean intakeEnabled = false;

    // GATE
    private Servo gate;

    private static final double GATE_CLOSED_DEG = 90.0;
    private static final double GATE_OPEN_DEG = 20.0;

    private double currentGateAngle = GATE_CLOSED_DEG;

    // BOTONES
    private boolean previousA = false;
    private boolean previousB = false;
    private boolean previousY = false;

    private boolean previousUp = false;
    private boolean previousDown = false;
    private boolean previousLeft = false;
    private boolean previousRight = false;

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

        tracking = new TurretFieldTrackingCommand(
                drive,
                turret,
                TARGET_X,
                TARGET_Y
        );

        // SHOOTER
        shooter = hardwareMap.get(
                DcMotorEx.class,
                "shoot"
        );

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        shooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        shooter.setPower(0);
        applyPIDF();

        // INTAKE
        intake = hardwareMap.get(
                DcMotor.class,
                "intake2"
        );

        intake.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        intake.setPower(0);

        // INDEXER
        indexer = hardwareMap.get(
                DcMotor.class,
                "intake1"
        );

        indexer.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        indexer.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        indexer.setPower(0);

        // GATE
        gate = hardwareMap.get(
                Servo.class,
                "gate"
        );

        gate.setDirection(Servo.Direction.REVERSE);

        // No mover gate durante INIT.

        telemetry.addLine("THUNDERROAR READY");
        telemetry.addLine("A = Shooter");
        telemetry.addLine("B = Turret Tracking");
        telemetry.addLine("Y = Intake ON/OFF");
        telemetry.addLine("X = Disparar");
        telemetry.addLine("RT/LT = Velocidad shooter");
        telemetry.update();
    }

    @Override
    public void loop() {

        // =========================================
        // CHASIS
        // =========================================

        driveCommand.execute();
        drive.update();

        // =========================================
        // INTAKE TOGGLE - Y
        // =========================================

        boolean currentY = gamepad1.y;

        if (currentY && !previousY) {
            intakeEnabled = !intakeEnabled;
        }

        previousY = currentY;

        // =========================================
        // DISPARAR - X
        // =========================================

        boolean feeding = gamepad1.x;

        if (feeding) {

            setGateAngle(GATE_OPEN_DEG);

            intake.setPower(INTAKE_POWER);
            indexer.setPower(INDEXER_POWER);

        } else if (intakeEnabled) {

            setGateAngle(GATE_CLOSED_DEG);

            intake.setPower(INTAKE_POWER);
            indexer.setPower(INDEXER_POWER);

        } else {

            intake.setPower(0);
            indexer.setPower(0);
        }

        // =========================================
        // SHOOTER TOGGLE - A
        // =========================================

        boolean currentA = gamepad1.a;

        if (currentA && !previousA) {
            shooterEnabled = !shooterEnabled;
        }

        previousA = currentA;

        // =========================================
        // VELOCIDAD SHOOTER
        // =========================================

        targetVelocity += gamepad1.right_trigger * 5.0;
        targetVelocity -= gamepad1.left_trigger * 5.0;

        targetVelocity = Math.max(
                0.0,
                Math.min(6000.0, targetVelocity)
        );

        if (shooterEnabled) {
            shooter.setVelocity(targetVelocity);
        } else {
            shooter.setVelocity(0);
        }

        // =========================================
        // TRACKING TORRETA - B
        // =========================================

        boolean currentB = gamepad1.b;

        if (currentB && !previousB) {

            if (tracking.isEnabled()) {
                tracking.disable();
            } else {
                tracking.enable();
            }
        }

        previousB = currentB;

        tracking.execute();

        // =========================================
        // PIDF SHOOTER
        // =========================================

        boolean currentUp = gamepad1.dpad_up;
        boolean currentDown = gamepad1.dpad_down;
        boolean currentRight = gamepad1.dpad_right;
        boolean currentLeft = gamepad1.dpad_left;

        if (currentUp && !previousUp) {
            p += 0.2;
            applyPIDF();
        }

        if (currentDown && !previousDown) {
            f += 0.5;
            applyPIDF();
        }

        if (currentRight && !previousRight) {
            d += 0.1;
            applyPIDF();
        }

        if (currentLeft && !previousLeft) {
            i += 0.01;
            applyPIDF();
        }

        previousUp = currentUp;
        previousDown = currentDown;
        previousRight = currentRight;
        previousLeft = currentLeft;

        // =========================================
        // TELEMETRÍA
        // =========================================

        telemetry.addLine("===== ROBOT =====");

        telemetry.addData("Robot X", "%.2f", drive.getX());
        telemetry.addData("Robot Y", "%.2f", drive.getY());

        telemetry.addData(
                "Heading",
                "%.2f",
                drive.getHeadingDegrees()
        );

        telemetry.addLine("===== TURRET =====");

        telemetry.addData(
                "Tracking",
                tracking.isEnabled() ? "ON" : "OFF"
        );

        telemetry.addData(
                "Distance",
                "%.2f",
                tracking.getDistanceToTarget()
        );

//        telemetry.addData(
//                "Desired",
//                "%.2f",
////                tracking.getDesiredAngleDeg()
//        );

        telemetry.addData(
                "Target",
                "%.2f",
                turret.getTargetAngleDeg()
        );

        telemetry.addData(
                "Actual",
                "%.2f",
                turret.getCurrentAngleDeg()
        );

        telemetry.addData(
                "Error",
                "%.2f",
                turret.getErrorDeg()
        );

        telemetry.addData(
                "Motor Power",
                "%.3f",
                turret.getMotorPower()
        );

        telemetry.addLine("===== SHOOTER =====");

        telemetry.addData(
                "Shooter",
                shooterEnabled ? "ON" : "OFF"
        );

        telemetry.addData(
                "Target Velocity",
                "%.1f",
                targetVelocity
        );

        telemetry.addData(
                "Actual Velocity",
                "%.1f",
                shooter.getVelocity()
        );

        telemetry.addData("P", "%.3f", p);
        telemetry.addData("I", "%.3f", i);
        telemetry.addData("D", "%.3f", d);
        telemetry.addData("F", "%.3f", f);

        telemetry.addLine("===== INTAKE =====");

        telemetry.addData(
                "Intake",
                intakeEnabled ? "ON" : "OFF"
        );

        telemetry.addData(
                "Feeding",
                feeding ? "YES" : "NO"
        );

        telemetry.addData(
                "Gate Angle",
                "%.1f",
                currentGateAngle
        );

        telemetry.update();
    }

    private void setGateAngle(double degrees) {

        currentGateAngle = degrees;

        double position = Math.max(
                0.0,
                Math.min(1.0, degrees / 180.0)
        );

        gate.setPosition(position);
    }

    private void applyPIDF() {

        shooter.setVelocityPIDFCoefficients(
                p, i, d, f
        );
    }

    @Override
    public void stop() {

        tracking.disable();
        turret.stop();

        drive.stop();

        shooter.setPower(0);
        intake.setPower(0);
        indexer.setPower(0);
    }
}

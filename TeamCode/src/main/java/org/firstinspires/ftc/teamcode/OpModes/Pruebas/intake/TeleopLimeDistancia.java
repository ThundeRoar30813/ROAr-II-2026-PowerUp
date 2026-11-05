
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.intake;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Shooter PIDF + Intake Test", group = "Pruebas")
public class TeleopLimeDistancia extends OpMode {

    // =========================
    // HARDWARE
    // =========================

    private DcMotorEx shooter;
    private DcMotor intake;
    private DcMotor indexer;

    // =========================
    // SHOOTER VELOCITY
    // =========================

    private double targetVelocity = 800;

    private static final double VELOCITY_STEP = 15;
    private static final double SHOOTER_MAX = 6500;

    // =========================
    // PIDF
    // =========================

    private double p = 4.0;
    private double i = 0.0;
    private double d = 0.5;
    private double f = 12.0;

    // =========================
    // BUTTON MEMORY
    // =========================

    private boolean lastA = false;
    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastLeft = false;
    private boolean lastRight = false;

    private boolean shooterEnabled = false;

    @Override
    public void init() {

        // =========================
        // INIT MOTORS
        // =========================

        shooter = hardwareMap.get(
                DcMotorEx.class, "mShooter");

        intake = hardwareMap.get(
                DcMotor.class, "mIntake1");

        indexer = hardwareMap.get(
                DcMotor.class, "mIntake2");

        // =========================
        // CONFIGURE SHOOTER
        // =========================

        shooter.setDirection(DcMotor.Direction.FORWARD);

        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT);

        shooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setPower(0);

        // =========================
        // CONFIGURE INTAKE
        // =========================

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        indexer.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setPower(0);
        indexer.setPower(0);

        applyPIDF();

        telemetry.addLine("SHOOTER PIDF + INTAKE");
        telemetry.addLine("");
        telemetry.addLine("GAMEPAD 1:");
        telemetry.addLine("A = Toggle shooter");
        telemetry.addLine("RT / LT = Adjust velocity");
        telemetry.addLine("UP = Increase P");
        telemetry.addLine("DOWN = Increase F");
        telemetry.addLine("RIGHT = Increase D");
        telemetry.addLine("LEFT = Increase I");
        telemetry.addLine("");
        telemetry.addLine("GAMEPAD 2:");
        telemetry.addLine("A = Intake forward");
        telemetry.addLine("Y = Indexer forward");
        telemetry.addLine("X = Intake + indexer forward");
        telemetry.addLine("B = Intake + indexer reverse");
        telemetry.update();
    }

    @Override
    public void loop() {

        // =========================
        // SHOOTER ON / OFF
        // GAMEPAD 1 - A
        // =========================

        if (gamepad1.a && !lastA) {
            shooterEnabled = !shooterEnabled;
        }

        lastA = gamepad1.a;

        // =========================
        // ADJUST TARGET VELOCITY
        // GAMEPAD 1 - RT / LT
        // =========================

        targetVelocity += gamepad1.right_trigger * VELOCITY_STEP;
        targetVelocity -= gamepad1.left_trigger * VELOCITY_STEP;

        targetVelocity = Math.max(
                0,
                Math.min(SHOOTER_MAX, targetVelocity));

        // =========================
        // TUNE PIDF
        // =========================

        // UP = P
        if (gamepad1.dpad_up && !lastUp) {
            p += 0.2;
            applyPIDF();
        }

        // DOWN = F
        if (gamepad1.dpad_down && !lastDown) {
            f += 0.5;
            applyPIDF();
        }

        // RIGHT = D
        if (gamepad1.dpad_right && !lastRight) {
            d += 0.1;
            applyPIDF();
        }

        // LEFT = I
        if (gamepad1.dpad_left && !lastLeft) {
            i += 0.01;
            applyPIDF();
        }

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;

        // =========================
        // RUN SHOOTER
        // =========================

        if (shooterEnabled) {
            shooter.setVelocity(targetVelocity);
        } else {
            shooter.setPower(0);
        }

        // =========================
        // INTAKE + INDEXER
        // GAMEPAD 2
        // =========================

        double intakePower = 0;
        double indexerPower = 0;

        if (gamepad2.b) {

            // B = BOTH REVERSE
            intakePower = -1.0;
            indexerPower = 1.0;

        } else if (gamepad2.x) {

            // X = BOTH FORWARD
            intakePower = 1.0;
            indexerPower = -0.5;

        } else {

            // A = INTAKE ONLY
            if (gamepad2.a) {
                intakePower = 1.0;
            }

            // Y = INDEXER ONLY
            if (gamepad2.y) {
                indexerPower = -0.5;
            }
        }

        // Motors stop when buttons are released
        intake.setPower(intakePower);
        indexer.setPower(indexerPower);

        // =========================
        // TELEMETRY
        // =========================

        telemetry.addLine("===== SHOOTER PIDF =====");

        telemetry.addData("Enabled", shooterEnabled);
        telemetry.addData("Target velocity", "%.1f", targetVelocity);
        telemetry.addData("Actual velocity", "%.1f", shooter.getVelocity());
        telemetry.addData(
                "Velocity error",
                "%.1f",
                targetVelocity - shooter.getVelocity());

        telemetry.addLine("");
        telemetry.addLine("===== PIDF =====");

        telemetry.addData("P", "%.3f", p);
        telemetry.addData("I", "%.3f", i);
        telemetry.addData("D", "%.3f", d);
        telemetry.addData("F", "%.3f", f);

        telemetry.addLine("");
        telemetry.addLine("===== INTAKE =====");

        telemetry.addData("Intake power", intake.getPower());
        telemetry.addData("Indexer power", indexer.getPower());

        telemetry.update();
    }

    // =========================
    // APPLY PIDF
    // =========================

    private void applyPIDF() {

        shooter.setVelocityPIDFCoefficients(
                p, i, d, f);
    }

    @Override
    public void stop() {

        shooter.setPower(0);
        intake.setPower(0);
        indexer.setPower(0);
    }
}

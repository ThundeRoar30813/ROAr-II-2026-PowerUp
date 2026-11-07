
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimingV10;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV4;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV3;

@TeleOp(name = "ThunderROAR V10 - Strafe Tracking")
public class TurretFieldTrackingV9 extends OpMode {

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCommand;

    private TurretSubsystemV3 turret;
    private TurretVisionV4 vision;
    private TurretAimingV10 aiming;

    private DcMotorEx shooter;

    private boolean trackingEnabled = false;
    private boolean visionEnabled = false;
    private boolean shooterEnabled = false;

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastVisionX = false;
    private boolean lastStart = false;
    private boolean lastY = false;

    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastRight = false;
    private boolean lastLeft = false;

    private double targetVelocity = 1100.0;

    private double p = 8.0;
    private double i = 0.0;
    private double d = 0.5;
    private double f = 18.0;

    private long lastVelocityChange = 0;

    // Distancia horizontal al AprilTag 36.
    private static final double MEASURED_TAG_DISTANCE_IN = 56.0;

    @Override
    public void init() {

        drive = new DriveSubsystem(hardwareMap);

        driveCommand = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> gamepad1.left_stick_x,
                () -> gamepad1.right_stick_x
        );

        drive.setInitialPose(0, 0, 0);

        turret = new TurretSubsystemV3(hardwareMap);

        // Potencia de prueba.
        turret.setMaxPower(0.60);

        vision = new TurretVisionV4(hardwareMap);

        aiming = new TurretAimingV10(
                drive,
                turret,
                vision
        );

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

        telemetry.addLine("THUNDERROAR V10");
        telemetry.addLine("STRAFE TRACKING");
        telemetry.addLine("Shooter OFF");
        telemetry.addLine("B: Tracking");
        telemetry.addLine("X: Vision");
        telemetry.addLine("Y: Acquire Tag 36");
        telemetry.update();
    }

    @Override
    public void start() {

        vision.start();
    }

    @Override
    public void loop() {

        // ========================================
        // DRIVE
        // ========================================

        driveCommand.execute();
        drive.update();

        // ========================================
        // LIMELIGHT
        // ========================================

        vision.update();

        // ========================================
        // GAMEPAD 1 - A: SHOOTER
        // ========================================

        boolean a = gamepad1.a;

        if (a && !lastA) {

            shooterEnabled = !shooterEnabled;
        }

        lastA = a;

        // ========================================
        // GAMEPAD 1 - B: TRACKING
        // ========================================

        boolean b = gamepad1.b;

        if (b && !lastB) {

            if (trackingEnabled) {

                aiming.disable();
                trackingEnabled = false;

            } else {

                aiming.enable();
                trackingEnabled = aiming.isEnabled();
            }
        }

        lastB = b;

        // ========================================
        // GAMEPAD 2 - X: VISION
        // ========================================

        boolean x = gamepad2.x;

        if (x && !lastVisionX) {

            visionEnabled = !visionEnabled;
            aiming.setVisionEnabled(visionEnabled);
        }

        lastVisionX = x;

        // ========================================
        // GAMEPAD 2 - Y: ACQUIRE TAG 36
        // ========================================

        boolean y = gamepad2.y;

        if (y && !lastY && trackingEnabled) {

            aiming.acquireTarget(
                    MEASURED_TAG_DISTANCE_IN
            );
        }

        lastY = y;

        // ========================================
        // GAMEPAD 2 - START: RESET FAULT
        // ========================================

        boolean reset = gamepad2.start;

        if (reset && !lastStart) {

            aiming.disable();
            trackingEnabled = false;
            turret.clearFault();
        }

        lastStart = reset;

        // ========================================
        // SHOOTER VELOCITY
        // ========================================

        long now = System.nanoTime();

        if (now - lastVelocityChange > 200_000_000L) {

            if (gamepad2.right_trigger > 0.5) {

                targetVelocity += 5;
                lastVelocityChange = now;

            } else if (gamepad2.left_trigger > 0.5) {

                targetVelocity -= 5;
                lastVelocityChange = now;
            }
        }

        targetVelocity = Range.clip(
                targetVelocity,
                0,
                3000
        );

        // ========================================
        // SHOOTER PIDF TUNING
        // ========================================

        boolean up = gamepad2.dpad_up;
        boolean down = gamepad2.dpad_down;
        boolean right = gamepad2.dpad_right;
        boolean left = gamepad2.dpad_left;

        boolean pidChanged = false;

        if (up && !lastUp) {

            p += 0.2;
            pidChanged = true;
        }

        if (down && !lastDown) {

            f += 0.5;
            pidChanged = true;
        }

        if (right && !lastRight) {

            d += 0.1;
            pidChanged = true;
        }

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

        // ========================================
        // SHOOTER CONTROL
        // ========================================

        if (shooterEnabled) {

            shooter.setVelocity(targetVelocity);

        } else {

            shooter.setPower(0);
        }

        // ========================================
        // TURRET TRACKING
        // ========================================

        if (trackingEnabled) {

            aiming.update();

            if (!aiming.isEnabled()) {
                trackingEnabled = false;
            }

        } else {

            turret.update();
        }

        // ========================================
        // TRACKING DIAGNOSTICS
        // ========================================

        double desiredAngle = aiming.getDesiredAngle();
        double commandedAngle = aiming.getCommandedAngle();
        double actualAngle = turret.getAngle();

        double targetLag = desiredAngle - commandedAngle;
        double motorLag = commandedAngle - actualAngle;
        double totalLag = desiredAngle - actualAngle;

        // ========================================
        // TELEMETRY
        // ========================================

        telemetry.addLine("========== THUNDERROAR V10 ==========");

        telemetry.addData(
                "Tracking",
                trackingEnabled
        );

        telemetry.addData(
                "Vision",
                visionEnabled
        );

        telemetry.addData(
                "Tag 36 Visible",
                vision.hasTarget()
        );

        telemetry.addData(
                "Target Locked",
                aiming.hasLock()
        );

        telemetry.addData(
                "Tracking Status",
                aiming.getStatus()
        );

        telemetry.addLine("========== ROBOT POSE ==========");

        telemetry.addData(
                "Robot X",
                "%.2f",
                drive.getX()
        );

        telemetry.addData(
                "Robot Y",
                "%.2f",
                drive.getY()
        );

        telemetry.addData(
                "Heading (deg)",
                "%.2f",
                drive.getHeadingDegrees()
        );

        telemetry.addData(
                "Robot Vx (in/s)",
                "%.2f",
                aiming.getVx()
        );

        telemetry.addData(
                "Robot Vy (in/s)",
                "%.2f",
                aiming.getVy()
        );

        telemetry.addData(
                "Robot Speed (in/s)",
                "%.2f",
                aiming.getSpeed()
        );

        telemetry.addLine("========== LIMELIGHT ==========");

        telemetry.addData(
                "Tag ID",
                36
        );

        telemetry.addData(
                "Tx (deg)",
                "%.2f",
                vision.getTx()
        );

        telemetry.addData(
                "Ty (deg)",
                "%.2f",
                vision.getTy()
        );

        telemetry.addData(
                "Frame Age (ms)",
                vision.getAgeMs()
        );

        telemetry.addData(
                "Vision Offset (deg)",
                "%.2f",
                aiming.getVisionOffset()
        );

        telemetry.addData(
                "Vision Rate (deg/s)",
                "%.2f",
                aiming.getVisionRate()
        );

        telemetry.addLine("========== PREDICTION ==========");

        telemetry.addData(
                "Prediction (deg)",
                "%.3f",
                aiming.getPredictionDegrees()
        );

        telemetry.addData(
                "Estimated Distance (in)",
                "%.2f",
                aiming.getDistance()
        );

        telemetry.addData(
                "Target X",
                "%.2f",
                aiming.getTargetX()
        );

        telemetry.addData(
                "Target Y",
                "%.2f",
                aiming.getTargetY()
        );

        telemetry.addLine("========== STRAFE DIAGNOSTIC ==========");

        telemetry.addData(
                "Desired Angle (deg)",
                "%.2f",
                desiredAngle
        );

        telemetry.addData(
                "Commanded Angle (deg)",
                "%.2f",
                commandedAngle
        );

        telemetry.addData(
                "Actual Angle (deg)",
                "%.2f",
                actualAngle
        );

        telemetry.addData(
                "Target Lag (deg)",
                "%.2f",
                targetLag
        );

        telemetry.addData(
                "Motor Lag (deg)",
                "%.2f",
                motorLag
        );

        telemetry.addData(
                "Total Lag (deg)",
                "%.2f",
                totalLag
        );

        telemetry.addData(
                "Turret Power",
                "%.2f",
                turret.getPower()
        );

        telemetry.addData(
                "Turret Error",
                "%.2f",
                turret.getError()
        );

        telemetry.addData(
                "Turret Fault",
                turret.getFaultReason()
        );

        telemetry.addLine("========== SHOOTER ==========");

        telemetry.addData(
                "Shooter Enabled",
                shooterEnabled
        );

        telemetry.addData(
                "Shooter Target",
                "%.1f",
                targetVelocity
        );

        telemetry.addData(
                "Shooter Actual",
                "%.1f",
                shooter.getVelocity()
        );

        telemetry.addData(
                "Shooter PIDF",
                "P %.2f I %.2f D %.2f F %.2f",
                p, i, d, f
        );

        telemetry.addData(
                "Visual P (deg)",
                "%.2f",
                aiming.getDirectOffset()
        );

        telemetry.addData(
                "Visual I (deg)",
                "%.2f",
                aiming.getIntegralOffset()
        );

        telemetry.update();
    }

    private void applyPIDF() {

        shooter.setVelocityPIDFCoefficients(
                p, i, d, f
        );
    }

    @Override
    public void stop() {

        shooter.setPower(0);

        aiming.disable();
        turret.stop();
        vision.stop();
        drive.stop();
    }
}

package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimingV8;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV4;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV3;

@TeleOp(name = "ThunderROAR V7 - Hybrid Tracking TEST")
public class TurretFieldTrackingV7 extends OpMode {
    private DriveSubsystem drive;
    private MecanumDriveCmd driveCommand;
    private TurretSubsystemV3 turret;
    private TurretVisionV4 vision;
    private TurretAimingV8 aiming;
    private DcMotorEx shooter;

    private boolean trackingEnabled = false, visionEnabled = false, shooterEnabled = false;
    private boolean lastA, lastB, lastVisionX, lastStart, lastY;
    private boolean lastUp, lastDown, lastRight, lastLeft;
    private long lastVelocityChange;

    private double targetVelocity = 1100.0;
    private double p = 8.0, i = 0.0, d = 0.5, f = 18.0;

    // Measure horizontal distance from turret rotation axis to AprilTag 36.
    // Update this before testing. This is NOT a fixed field coordinate.
    private static final double MEASURED_TAG_DISTANCE_IN = 72.0;

    @Override public void init() {
        drive = new DriveSubsystem(hardwareMap);
        driveCommand = new MecanumDriveCmd(drive,
                () -> -gamepad1.left_stick_y,
                () -> gamepad1.left_stick_x,
                () -> gamepad1.right_stick_x);
        drive.setInitialPose(0, 0, 0);
        turret = new TurretSubsystemV3(hardwareMap);
        turret.setMaxPower(0.70); // conservative first test; restore 0.80 after verifying
        vision = new TurretVisionV4(hardwareMap);
        aiming = new TurretAimingV8(drive, turret, vision);
        shooter = hardwareMap.get(DcMotorEx.class, "shoot");
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPower(0);
        applyPIDF();
        telemetry.addLine("V7 HYBRID TEST - Shooter disabled until manual A toggle");
        telemetry.addLine("Measure tag distance and align camera/shooter before lock");
        telemetry.update();
    }

    @Override public void start() { vision.start(); }

    @Override public void loop() {
        driveCommand.execute();
        drive.update();
        vision.update();

        boolean a = gamepad1.a;
        if (a && !lastA) shooterEnabled = !shooterEnabled;
        lastA = a;

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

        boolean x = gamepad2.x;
        if (x && !lastVisionX) {
            visionEnabled = !visionEnabled;
            aiming.setVisionEnabled(visionEnabled);
        }
        lastVisionX = x;

        // New control: gamepad2 Y captures tag location using a measured distance.
        boolean y = gamepad2.y;
        if (y && !lastY && trackingEnabled) {
            aiming.acquireTarget(MEASURED_TAG_DISTANCE_IN);
        }
        lastY = y;

        boolean reset = gamepad2.start;
        if (reset && !lastStart) {
            aiming.disable();
            trackingEnabled = false;
            turret.clearFault();
        }
        lastStart = reset;

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
        targetVelocity = Range.clip(targetVelocity, 0, 3000);

        boolean up = gamepad2.dpad_up, down = gamepad2.dpad_down;
        boolean right = gamepad2.dpad_right, left = gamepad2.dpad_left;
        boolean pidChanged = false;
        if (up && !lastUp) { p += 0.2; pidChanged = true; }
        if (down && !lastDown) { f += 0.5; pidChanged = true; }
        if (right && !lastRight) { d += 0.1; pidChanged = true; }
        if (left && !lastLeft) { i += 0.01; pidChanged = true; }
        lastUp = up; lastDown = down; lastRight = right; lastLeft = left;
        if (pidChanged) applyPIDF();

        if (shooterEnabled) shooter.setVelocity(targetVelocity);
        else shooter.setPower(0);

        if (trackingEnabled) {
            aiming.update();
            if (!aiming.isEnabled()) trackingEnabled = false;
        } else {
            turret.update();
        }

        telemetry.addLine("=== THUNDERROAR HYBRID V8 TEST ===");
        telemetry.addData("Pose X / Y", "%.2f / %.2f", drive.getX(), drive.getY());
        telemetry.addData("Heading", "%.2f", drive.getHeadingDegrees());
        telemetry.addData("Tracking / Vision", "%s / %s", trackingEnabled, visionEnabled);
        telemetry.addData("Tag 36 / Age ms", "%s / %d", vision.hasTarget(), vision.getAgeMs());
        telemetry.addData("Tx / Ty", "%.2f / %.2f", vision.getTx(), vision.getTy());
        telemetry.addData("Lock / Status", "%s / %s", aiming.hasLock(), aiming.getStatus());
        telemetry.addData("Estimated target X / Y", "%.2f / %.2f", aiming.getTargetX(), aiming.getTargetY());
        telemetry.addData("Estimated distance", "%.2f", aiming.getDistance());
        telemetry.addData("Vision offset", "%.2f", aiming.getVisionOffset());
        telemetry.addData("Desired / Commanded", "%.2f / %.2f", aiming.getDesiredAngle(), aiming.getCommandedAngle());
        telemetry.addData("Turret actual / error", "%.2f / %.2f", turret.getAngle(), turret.getError());
        telemetry.addData("Turret power / fault", "%.2f / %s", turret.getPower(), turret.getFaultReason());
        telemetry.addData("Shooter enabled / target / actual", "%s / %.1f / %.1f", shooterEnabled, targetVelocity, shooter.getVelocity());
        telemetry.addData("PIDF", "P %.2f I %.2f D %.2f F %.2f", p, i, d, f);
        telemetry.update();
    }

    private void applyPIDF() { shooter.setVelocityPIDFCoefficients(p, i, d, f); }

    @Override public void stop() {
        shooter.setPower(0);
        aiming.disable();
        turret.stop();
        vision.stop();
        drive.stop();
    }
}

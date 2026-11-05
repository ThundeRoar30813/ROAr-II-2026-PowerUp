package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.TurretFieldTrackingCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

@Disabled
@TeleOp(name = "ThunderROAR - FULL + Limelight PI")
public class ThunderROAR_Complete_PI_TeleOp extends OpMode {

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCommand;
    private TurretSubsystem turret;
    private TurretFieldTrackingCommand tracking;
    private Limelight3A limelight;
    private DcMotorEx shooter;
    private DcMotor intake, indexer;
    private Servo gate;

    // These are the values in your existing test code; verify actual field target.
    private static final double TARGET_X = 57.0, TARGET_Y = 59.0;
    private static final int PIPELINE = 0, TAG_34 = 34, TAG_36 = 36;
    private static final double MAX_RESULT_AGE_MS = 150.0;
    private static final double VISION_SIGN = -1.0;
    private static final double VISION_KP = 0.35, VISION_KI = 0.45;
    private static final double VISION_DEADBAND_DEG = 0.8;
    private static final double MAX_VISION_OFFSET_DEG = 12.0;
    private static final double MAX_OFFSET_RATE = 8.0;
    private double offset34Deg = 0.0, offset36Deg = 0.0;
    private double accumulatedVisionCorrection = 0.0;
    private double proportionalVisionCorrection = 0.0;
    private double requestedVisionCorrection = 0.0;
    private double appliedVisionCorrection = 0.0;
    private long previousVisionTime;
    private int selectedTag = TAG_36;
    private boolean visionEnabled = false, tagVisible = false, frameFresh = false;
    private double tx = Double.NaN, visualErrorDeg = Double.NaN;
    private String visionStatus = "OFF";

    // Shooter units are motor encoder ticks/second, NOT RPM.
    private double targetVelocity = 800.0;
    private double p = 4.0, i = 0.0, d = 0.5, f = 12.0;
    private boolean shooterEnabled = false, intakeEnabled = false;
    private static final double INTAKE_POWER = 1.0, INDEXER_POWER = 1.0;
    private static final double GATE_CLOSED_DEG = 90.0, GATE_OPEN_DEG = 20.0;
    private double currentGateAngle = GATE_CLOSED_DEG;

    private boolean previousA1, previousB1, previousY1;
    private boolean previousUp1, previousDown1, previousLeft1, previousRight1;
    private boolean previousA2, previousLeft2, previousRight2;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap);
        driveCommand = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );
        turret = new TurretSubsystem(hardwareMap);
        tracking = new TurretFieldTrackingCommand(drive, turret, TARGET_X, TARGET_Y);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE);

        shooter = hardwareMap.get(DcMotorEx.class, "shoot");
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPower(0);
        applyPIDF();

        intake = hardwareMap.get(DcMotor.class, "intake2");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);

        indexer = hardwareMap.get(DcMotor.class, "intake1");
        indexer.setDirection(DcMotorSimple.Direction.REVERSE);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setPower(0);

        gate = hardwareMap.get(Servo.class, "gate");
        gate.setDirection(Servo.Direction.REVERSE);
        // Preserve original behavior: do not move gate in INIT.

        telemetry.addLine("THUNDERROAR FULL TELEOP READY");
        telemetry.addLine("GP1 A shooter | B turret | Y intake | hold X feed");
        telemetry.addLine("GP1 triggers shooter speed | dpad PIDF");
        telemetry.addLine("GP2 A vision PI | left tag34 | right tag36");
        telemetry.addLine("Position turret safely before INIT; verify cable clearance.");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
        resetVisionCorrection();
    }

    @Override
    public void loop() {
        // Drive is updated once per cycle, as in the two source TeleOps.
        driveCommand.execute();
        drive.update();

        // GP1: shooter toggle, turret tracking toggle, intake toggle.
        if (gamepad1.a && !previousA1) shooterEnabled = !shooterEnabled;
        if (gamepad1.y && !previousY1) intakeEnabled = !intakeEnabled;
        if (gamepad1.b && !previousB1) {
            if (tracking.isEnabled()) {
                tracking.disable();
                visionEnabled = false;
                resetVisionCorrection();
            } else {
                resetVisionCorrection();
                tracking.enable();
            }
        }
        previousA1 = gamepad1.a;
        previousY1 = gamepad1.y;
        previousB1 = gamepad1.b;

        // GP2: vision toggle and tag selection; no conflict with shooter PIDF.
        if (gamepad2.a && !previousA2 && tracking.isEnabled()) {
            visionEnabled = !visionEnabled;
            resetVisionCorrection();
        }
        if (gamepad2.dpad_left && !previousLeft2) {
            selectedTag = TAG_34;
            resetVisionCorrection();
        }
        if (gamepad2.dpad_right && !previousRight2) {
            selectedTag = TAG_36;
            resetVisionCorrection();
        }
        previousA2 = gamepad2.a;
        previousLeft2 = gamepad2.dpad_left;
        previousRight2 = gamepad2.dpad_right;

        // Shooter speed (same per-loop increments as original source code).
        targetVelocity += gamepad1.right_trigger * 5.0;
        targetVelocity -= gamepad1.left_trigger * 5.0;
        targetVelocity = clamp(targetVelocity, 0.0, 6000.0);
        shooter.setVelocity(shooterEnabled ? targetVelocity : 0.0);

        // Original PIDF tuning controls on GP1 dpad.
        if (gamepad1.dpad_up && !previousUp1) { p += 0.2; applyPIDF(); }
        if (gamepad1.dpad_down && !previousDown1) { f += 0.5; applyPIDF(); }
        if (gamepad1.dpad_right && !previousRight1) { d += 0.1; applyPIDF(); }
        if (gamepad1.dpad_left && !previousLeft1) { i += 0.01; applyPIDF(); }
        previousUp1 = gamepad1.dpad_up;
        previousDown1 = gamepad1.dpad_down;
        previousRight1 = gamepad1.dpad_right;
        previousLeft1 = gamepad1.dpad_left;

        // Intake/indexer/gate: preserve original manual feeding behavior.
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
            intake.setPower(0.0);
            indexer.setPower(0.0);
            // Preserve previous gate position when idle.
        }

        // Limelight: read selected AprilTag and reject stale results.
        tagVisible = false;
        frameFresh = false;
        tx = Double.NaN;
        visualErrorDeg = Double.NaN;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ageMs = result.getStaleness();
            frameFresh = ageMs >= 0.0 && ageMs <= MAX_RESULT_AGE_MS;
            if (frameFresh && result.getFiducialResults() != null) {
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    if (tag.getFiducialId() == selectedTag) {
                        tagVisible = true;
                        tx = tag.getTargetXDegrees();
                        break;
                    }
                }
            }
        }
        if (tagVisible) {
            visualErrorDeg = tx - (selectedTag == TAG_34 ? offset34Deg : offset36Deg);
        }

        long now = System.nanoTime();
        double dt = previousVisionTime == 0 ? 0.02 : (now - previousVisionTime) / 1_000_000_000.0;
        previousVisionTime = now;
        dt = clamp(dt, 0.001, 0.05);

        // Preserve the CURRENT PI implementation, including its behavior in deadband.
        if (!tracking.isEnabled()) {
            visionStatus = "PINPOINT OFF";
            resetVisionCorrection();
        } else if (!visionEnabled) {
            visionStatus = "VISION OFF";
            resetVisionCorrection();
        } else if (!frameFresh) {
            visionStatus = "FRAME NO RECIENTE";
            proportionalVisionCorrection = 0.0;
            requestedVisionCorrection = appliedVisionCorrection;
        } else if (!tagVisible) {
            visionStatus = "TAG NO VISIBLE";
            proportionalVisionCorrection = 0.0;
            requestedVisionCorrection = appliedVisionCorrection;
        } else {
            double error = visualErrorDeg;
            if (Math.abs(error) <= VISION_DEADBAND_DEG) {
                proportionalVisionCorrection = 0.0;
                requestedVisionCorrection = appliedVisionCorrection;
                visionStatus = "CENTRADO";
            } else {
                proportionalVisionCorrection = VISION_SIGN * VISION_KP * error;
                double integralChange = VISION_SIGN * VISION_KI * error * dt;
                double proposedIntegral = accumulatedVisionCorrection + integralChange;
                double proposedOutput = proposedIntegral + proportionalVisionCorrection;
                boolean pushingUpper = proposedOutput > MAX_VISION_OFFSET_DEG && integralChange > 0.0;
                boolean pushingLower = proposedOutput < -MAX_VISION_OFFSET_DEG && integralChange < 0.0;
                if (!pushingUpper && !pushingLower) {
                    accumulatedVisionCorrection = clamp(proposedIntegral,
                            -MAX_VISION_OFFSET_DEG, MAX_VISION_OFFSET_DEG);
                }
                requestedVisionCorrection = clamp(
                        accumulatedVisionCorrection + proportionalVisionCorrection,
                        -MAX_VISION_OFFSET_DEG, MAX_VISION_OFFSET_DEG);
                visionStatus = Math.abs(requestedVisionCorrection) >= MAX_VISION_OFFSET_DEG - 0.01
                        ? "OFFSET LIMITADO" : "PI CORRIGIENDO";
            }
        }

        if (visionEnabled && tracking.isEnabled()) {
            double maxChange = MAX_OFFSET_RATE * dt;
            double correctionDifference = requestedVisionCorrection - appliedVisionCorrection;
            double limitedChange = clamp(correctionDifference, -maxChange, maxChange);
            // Same version as latest test: no additional 0.20 multiplier.
            appliedVisionCorrection = clamp(appliedVisionCorrection + limitedChange,
                    -MAX_VISION_OFFSET_DEG, MAX_VISION_OFFSET_DEG);
        } else {
            appliedVisionCorrection = 0.0;
        }
        //tracking.setVisionCorrectionDeg(appliedVisionCorrection);
        if (tracking.isEnabled()) tracking.execute();
        else turret.stop();

        telemetry.addLine("===== THUNDERROAR FULL TELEOP =====");
        telemetry.addData("Shooter", shooterEnabled ? "ON" : "OFF");
        telemetry.addData("Shooter target (ticks/s)", "%.1f", targetVelocity);
        telemetry.addData("Shooter actual (ticks/s)", "%.1f", shooter.getVelocity());
        telemetry.addData("Shooter error (ticks/s)", "%.1f", targetVelocity - shooter.getVelocity());
        telemetry.addData("PIDF P/I/D/F", "%.2f / %.3f / %.2f / %.2f", p, i, d, f);
        telemetry.addData("Intake toggle", intakeEnabled ? "ON" : "OFF");
        telemetry.addData("Feeding X", feeding ? "ON" : "OFF");
        telemetry.addData("Intake / Indexer", "%.2f / %.2f", intake.getPower(), indexer.getPower());
        telemetry.addData("Gate deg (command)", "%.1f", currentGateAngle);
        telemetry.addData("Tracking", tracking.isEnabled() ? "ON" : "OFF");
        telemetry.addData("Vision PI", visionEnabled ? "ON" : "OFF");
        telemetry.addData("Tag / visible / fresh", "%d / %b / %b", selectedTag, tagVisible, frameFresh);
        telemetry.addData("TX / error", "%.2f / %.2f", tx, visualErrorDeg);
        telemetry.addData("P / I vision", "%.2f / %.2f", proportionalVisionCorrection, accumulatedVisionCorrection);
        telemetry.addData("Offset requested / applied", "%.2f / %.2f", requestedVisionCorrection, appliedVisionCorrection);
        telemetry.addData("Vision status", visionStatus);
        telemetry.addData("Turret actual / target", "%.2f / %.2f", turret.getCurrentAngleDeg(), turret.getTargetAngleDeg());
        telemetry.addData("Turret error / power", "%.2f / %.3f", turret.getErrorDeg(), turret.getMotorPower());
        telemetry.addData("Distance to target (in)", "%.2f", tracking.getDistanceToTarget());
        telemetry.addData("Pose X/Y/H", "%.2f / %.2f / %.2f", drive.getX(), drive.getY(), drive.getHeadingDegrees());
        telemetry.update();
    }

    private void setGateAngle(double degrees) {
        degrees = clamp(degrees, 0.0, 180.0);
        gate.setPosition(degrees / 180.0);
        currentGateAngle = degrees;
    }

    private void applyPIDF() {
        shooter.setVelocityPIDFCoefficients(p, i, d, f);
    }

    private void resetVisionCorrection() {
        accumulatedVisionCorrection = 0.0;
        proportionalVisionCorrection = 0.0;
        requestedVisionCorrection = 0.0;
        appliedVisionCorrection = 0.0;
        previousVisionTime = System.nanoTime();
        //if (tracking != null) tracking.setVisionCorrectionDeg(0.0);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void stop() {
        visionEnabled = false;
        resetVisionCorrection();
        if (limelight != null) limelight.stop();
        if (shooter != null) { shooter.setVelocity(0); shooter.setPower(0); }
        if (intake != null) intake.setPower(0);
        if (indexer != null) indexer.setPower(0);
        if (tracking != null) tracking.disable();
        if (turret != null) turret.stop();
        if (driveCommand != null) driveCommand.end();
        else if (drive != null) drive.stop();
    }
}

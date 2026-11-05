
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.TurretFieldTrackingCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;
@Disabled
@TeleOp(name = "ThunderROAR - DOS GOALS FULL PI")
public class ThunderROAR_DosGoals_FullPI extends OpMode {

    // ============================================
    // CONFIGURAR COORDENADAS PEDRO PATHING
    // ============================================
    // NO son coordenadas oficiales verificadas.
    // Sustituir NaN por las coordenadas medidas.
    private static final double GOAL_1_X = Double.NaN;
    private static final double GOAL_1_Y = Double.NaN;

    private static final double GOAL_2_X = Double.NaN;
    private static final double GOAL_2_Y = Double.NaN;

    // DECODE oficial: azul 20, rojo 24.
    // Si usas etiquetas personalizadas,
    // reemplaza estos IDs por los reales.
    private static final int TAG_1 = 20;
    private static final int TAG_2 = 24;

    private int selectedGoal = 1;
    private int selectedTag = TAG_1;

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCommand;
    private TurretSubsystem turret;
    private TurretFieldTrackingCommand tracking;
    private Limelight3A limelight;

    private DcMotorEx shooter;
    private DcMotor intake, indexer;
    private Servo gate, hood;

    // ============================================
    // SHOOTER PIDF
    // ============================================
    private double targetVelocity = 800;
    private double p = 4.0;
    private double i = 0.0;
    private double d = 0.5;
    private double f = 12.0;

    private boolean shooterEnabled = false;
    private boolean intakeEnabled = false;

    // ============================================
    // CAPUCHA
    // ============================================
    private static final double[] HOOD_POSITIONS = {
            0.4444, 0.5560, 0.6667, 0.7778, 0.8890
    };

    private int hoodIndex = 2;
    private boolean gateOpened = false;

    // ============================================
    // LIMELIGHT PI
    // ============================================
    private static final double VISION_SIGN = -1.0;
    private static final double VISION_KP = 0.35;
    private static final double VISION_KI = 0.45;
    private static final double DEADBAND = 0.8;
    private static final double MAX_OFFSET = 12.0;
    private static final double MAX_OFFSET_RATE = 8.0;

    private boolean visionEnabled = false;
    private boolean tagVisible = false;
    private double tx = Double.NaN;
    private double visionError = Double.NaN;

    private double visionP = 0;
    private double visionI = 0;
    private double requestedOffset = 0;
    private double appliedOffset = 0;

    private long previousTime = 0;

    // Calibrar offset del centro de disparo.
    private double offsetTag1 = 0;
    private double offsetTag2 = 0;

    // ============================================
    // BOTONES ANTERIORES
    // ============================================
    private boolean oldA, oldB, oldY;
    private boolean oldLB, oldRB;
    private boolean oldHoodUp, oldHoodDown;
    private boolean oldVisionA;
    private boolean oldP, oldI, oldD, oldF;

    private double goalX() {
        return selectedGoal == 1 ? GOAL_1_X : GOAL_2_X;
    }

    private double goalY() {
        return selectedGoal == 1 ? GOAL_1_Y : GOAL_2_Y;
    }

    private boolean goalConfigured() {
        return Double.isFinite(goalX())
                && Double.isFinite(goalY());
    }

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

        // Crear tracking solo cuando haya coordenadas.
        if (goalConfigured()) {
            tracking = new TurretFieldTrackingCommand(
                    drive, turret, goalX(), goalY());
        }

        limelight = hardwareMap.get(
                Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        shooter = hardwareMap.get(
                DcMotorEx.class, "shoot");
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPower(0);
        applyPIDF();

        intake = hardwareMap.get(
                DcMotor.class, "intake2");
        indexer = hardwareMap.get(
                DcMotor.class, "intake1");

        intake.setDirection(DcMotor.Direction.FORWARD);
        indexer.setDirection(DcMotor.Direction.REVERSE);

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setPower(0);
        indexer.setPower(0);

        gate = hardwareMap.get(Servo.class, "gate");
        gate.setDirection(Servo.Direction.REVERSE);

        hood = hardwareMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.FORWARD);

        // No mover servos durante INIT.
        telemetry.addLine("THUNDERROAR DOS GOALS");
        telemetry.addData("Coordenadas", goalConfigured()
                ? "CONFIGURADAS" : "FALTAN");
        telemetry.addLine("G1 LB/RB = cambiar GOAL");
        telemetry.addLine("G1 B = tracking");
        telemetry.addLine("G2 A = vision PI");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
        resetVision();
    }

    @Override
    public void loop() {

        // ========================================
        // CHASIS
        // ========================================
        driveCommand.execute();
        drive.update();

        // ========================================
        // CAMBIAR GOAL
        // ========================================
        if (gamepad1.left_bumper && !oldLB) {
            selectGoal(1);
        }

        if (gamepad1.right_bumper && !oldRB) {
            selectGoal(2);
        }

        oldLB = gamepad1.left_bumper;
        oldRB = gamepad1.right_bumper;

        // ========================================
        // TRACKING
        // ========================================
        if (gamepad1.b && !oldB) {
            if (tracking != null && tracking.isEnabled()) {
                tracking.disable();
                visionEnabled = false;
                resetVision();
            } else if (goalConfigured()) {
                if (tracking == null) {
                    tracking = new TurretFieldTrackingCommand(
                            drive, turret, goalX(), goalY());
                }
                tracking.enable();
                resetVision();
            }
        }
        oldB = gamepad1.b;

        // ========================================
        // VISION ON/OFF
        // ========================================
        if (gamepad2.a && !oldVisionA
                && tracking != null
                && tracking.isEnabled()) {
            visionEnabled = !visionEnabled;
            resetVision();
        }
        oldVisionA = gamepad2.a;

        // ========================================
        // SHOOTER
        // ========================================
        if (gamepad1.a && !oldA) {
            shooterEnabled = !shooterEnabled;
        }
        oldA = gamepad1.a;

        targetVelocity += gamepad1.right_trigger * 5.0;
        targetVelocity -= gamepad1.left_trigger * 5.0;
        targetVelocity = clamp(targetVelocity, 0, 6000);

        shooter.setVelocity(
                shooterEnabled ? targetVelocity : 0);

        // ========================================
        // PIDF SHOOTER - GAMEPAD 2
        // ========================================
        if (gamepad2.dpad_up && !oldP) {
            p += 0.2;
            applyPIDF();
        }
        if (gamepad2.dpad_left && !oldI) {
            i += 0.01;
            applyPIDF();
        }
        if (gamepad2.dpad_right && !oldD) {
            d += 0.1;
            applyPIDF();
        }
        if (gamepad2.dpad_down && !oldF) {
            f += 0.5;
            applyPIDF();
        }

        oldP = gamepad2.dpad_up;
        oldI = gamepad2.dpad_left;
        oldD = gamepad2.dpad_right;
        oldF = gamepad2.dpad_down;

        // ========================================
        // CAPUCHA
        // ========================================
        if (gamepad1.dpad_up && !oldHoodUp) {
            setHood(hoodIndex + 1);
        }
        if (gamepad1.dpad_down && !oldHoodDown) {
            setHood(hoodIndex - 1);
        }
        oldHoodUp = gamepad1.dpad_up;
        oldHoodDown = gamepad1.dpad_down;

        // ========================================
        // INTAKE / INDEXER / GATE
        // ========================================
        if (gamepad1.y && !oldY) {
            intakeEnabled = !intakeEnabled;
        }
        oldY = gamepad1.y;

        boolean feeding = gamepad1.x;

        if (feeding) {
            setGate(20);
            gateOpened = true;
            intake.setPower(1);
            indexer.setPower(1);
        } else if (intakeEnabled) {
            setGate(90);
            gateOpened = false;
            intake.setPower(1);
            indexer.setPower(1);
        } else {
            intake.setPower(0);
            indexer.setPower(0);

            if (gateOpened) {
                setGate(90);
                gateOpened = false;
            }
        }

        // ========================================
        // LEER APRILTAG SELECCIONADO
        // ========================================
        tagVisible = false;
        tx = Double.NaN;
        visionError = Double.NaN;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()
                && result.getStaleness() >= 0
                && result.getStaleness() <= 150
                && result.getFiducialResults() != null) {

            for (LLResultTypes.FiducialResult tag :
                    result.getFiducialResults()) {

                if (tag.getFiducialId() == selectedTag) {
                    tagVisible = true;
                    tx = tag.getTargetXDegrees();
                    break;
                }
            }
        }

        if (tagVisible) {
            double tagOffset = selectedGoal == 1
                    ? offsetTag1 : offsetTag2;
            visionError = tx - tagOffset;
        }

        // ========================================
        // CONTROL PI VISUAL
        // ========================================
        long now = System.nanoTime();
        double dt = previousTime == 0
                ? 0.02
                : (now - previousTime) / 1e9;
        previousTime = now;
        dt = clamp(dt, 0.001, 0.05);

        boolean trackingOn = tracking != null
                && tracking.isEnabled();

        if (trackingOn && visionEnabled && tagVisible) {

            if (Math.abs(visionError) > DEADBAND) {

                visionP = VISION_SIGN
                        * VISION_KP * visionError;

                double deltaI = VISION_SIGN
                        * VISION_KI * visionError * dt;

                double proposedI = visionI + deltaI;

                double proposedOutput = visionP + proposedI;

                boolean saturating =
                        (proposedOutput > MAX_OFFSET
                                && deltaI > 0)
                                || (proposedOutput < -MAX_OFFSET
                                && deltaI < 0);

                if (!saturating) {
                    visionI = clamp(
                            proposedI,
                            -MAX_OFFSET, MAX_OFFSET);
                }

                requestedOffset = clamp(
                        visionP + visionI,
                        -MAX_OFFSET, MAX_OFFSET);
            } else {
                visionP = 0;
                requestedOffset = appliedOffset;
            }

        } else {
            visionP = 0;
            requestedOffset = appliedOffset;
        }

        if (trackingOn && visionEnabled) {
            double maxStep = MAX_OFFSET_RATE * dt;

            appliedOffset += clamp(
                    requestedOffset - appliedOffset,
                    -maxStep, maxStep);

            appliedOffset = clamp(
                    appliedOffset, -MAX_OFFSET, MAX_OFFSET);
        } else {
            appliedOffset = 0;
        }

        // ========================================
        // EJECUTAR TORRETA
        // ========================================
        if (trackingOn) {
           // tracking.setVisionCorrectionDeg(appliedOffset);
            tracking.execute();
        } else {
            turret.stop();
        }

        // ========================================
        // TELEMETRIA
        // ========================================
        telemetry.addLine("===== THUNDERROAR =====");

        telemetry.addData("GOAL", selectedGoal);
        telemetry.addData("AprilTag", selectedTag);
        telemetry.addData("Coordenadas validas",
                goalConfigured());
        telemetry.addData("Goal X", goalX());
        telemetry.addData("Goal Y", goalY());

        telemetry.addData("Tracking", trackingOn);
        telemetry.addData("Vision PI", visionEnabled);
        telemetry.addData("Tag visible", tagVisible);
        telemetry.addData("TX", "%.2f", tx);
        telemetry.addData("Vision error", "%.2f", visionError);
        telemetry.addData("Vision P/I", "%.2f / %.2f",
                visionP, visionI);
        telemetry.addData("Vision offset", "%.2f",
                appliedOffset);

        telemetry.addData("Turret actual", "%.2f",
                turret.getCurrentAngleDeg());
        telemetry.addData("Turret target", "%.2f",
                turret.getTargetAngleDeg());
        telemetry.addData("Turret error", "%.2f",
                turret.getErrorDeg());
        telemetry.addData("Turret power", "%.3f",
                turret.getMotorPower());

        telemetry.addData("Shooter", shooterEnabled);
        telemetry.addData("Shooter target ticks/s",
                "%.1f", targetVelocity);
        telemetry.addData("Shooter actual ticks/s",
                "%.1f", shooter.getVelocity());
        telemetry.addData("PIDF", "%.2f %.3f %.2f %.2f",
                p, i, d, f);

        telemetry.addData("Intake", intakeEnabled);
        telemetry.addData("Feeding", feeding);
        telemetry.addData("Hood preset", hoodIndex + 1);
        telemetry.addData("Hood position", "%.4f",
                HOOD_POSITIONS[hoodIndex]);

        telemetry.addData("Robot X/Y", "%.2f / %.2f",
                drive.getX(), drive.getY());
        telemetry.addData("Robot Heading", "%.2f",
                drive.getHeadingDegrees());

        telemetry.update();
    }

    // ============================================
    // SELECCIONAR OTRO OBJETIVO
    // ============================================
    private void selectGoal(int goal) {
        if (selectedGoal == goal) return;

        selectedGoal = goal;
        selectedTag = goal == 1 ? TAG_1 : TAG_2;
        resetVision();

        if (tracking != null && tracking.isEnabled()) {
            if (goalConfigured()) {
                // No llamar enable() de nuevo.
                // Conservar referencia del campo.
                tracking.setTarget(goalX(), goalY());
            } else {
                tracking.disable();
                visionEnabled = false;
            }
        } else if (goalConfigured()) {
            tracking = new TurretFieldTrackingCommand(
                    drive, turret, goalX(), goalY());
        }
    }

    private void resetVision() {
        visionP = 0;
        visionI = 0;
        requestedOffset = 0;
        appliedOffset = 0;
        previousTime = System.nanoTime();

        if (tracking != null) {
            //tracking.setVisionCorrectionDeg(0);
        }
    }

    private void setHood(int index) {
        hoodIndex = (int) clamp(
                index, 0, HOOD_POSITIONS.length - 1);
        hood.setPosition(HOOD_POSITIONS[hoodIndex]);
    }

    private void setGate(double degrees) {
        gate.setPosition(
                clamp(degrees, 0, 180) / 180.0);
    }

    private void applyPIDF() {
        shooter.setVelocityPIDFCoefficients(
                p, i, d, f);
    }

    private static double clamp(
            double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();

        if (shooter != null) {
            shooter.setVelocity(0);
            shooter.setPower(0);
        }

        if (intake != null) intake.setPower(0);
        if (indexer != null) indexer.setPower(0);

        if (tracking != null) tracking.disable();
        if (turret != null) turret.stop();

        if (driveCommand != null) driveCommand.end();
        else if (drive != null) drive.stop();
    }
}

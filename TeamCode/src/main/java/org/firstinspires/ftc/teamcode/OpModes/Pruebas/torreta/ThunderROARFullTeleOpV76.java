
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV76;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV5;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

@TeleOp(name = "ThunderROAR FULL V76AAAAAAAAAAAAAAAAAAAAA")
public class ThunderROARFullTeleOpV76 extends OpMode {

    // =========================================
    // PEDRO
    // =========================================

    public static double START_X = 72;
    public static double START_Y = 72;
    public static double START_HEADING = 0;

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCmd;

    // =========================================
    // TURRET V76 ORIGINAL
    // =========================================

    private TurretSubsystemV4 turret;
    private TurretVisionV5 vision;
    private TurretAimControllerV76 aiming;

    public static double TURRET_MIN_DEG = -80;
    public static double TURRET_MAX_DEG = 230;
    public static double LIMIT_MARGIN_DEG = 10;

    private boolean trackingEnabled = false;
    private boolean returningHome = false;

    private int selectedTag = 34;
    private String turretStatus = "READY";

    // =========================================
    // SHOOTER
    // =========================================

    private DcMotorEx shooter;

    private boolean shooterEnabled = false;

    // INICIAR EN 1000 TICKS/S
    private double targetVelocity = 1000;

    private double p = 30.0;
    private double i = 0.0;
    private double d = 0.5;
    private double f = 16.0;

    private long lastVelocityChange = 0;

    // =========================================
    // INTAKE / INDEXER
    // =========================================

    private DcMotor intake;
    private DcMotor indexer;

    private boolean autoIntakeEnabled = false;

    private double intakePower = 0;
    private double indexerPower = 0;

    // =========================================
    // GATE
    // =========================================

    private Servo gate;

    private static final double GATE_CLOSED =
            90.0 / 180.0;

    private static final double GATE_OPEN =
            20.0 / 180.0;

    private boolean gateOpen = false;

    // =========================================
    // HOOD
    // =========================================

    private Servo hood;

    private static final double[] HOOD_POSITIONS = {
            0.4444,
            0.5560,
            0.6667,
            0.7778,
            0.8890
    };

    private int hoodIndex = 2;

    // =========================================
    // COLOR SENSORS
    // =========================================

    private ColorSensor colorBottom;
    private ColorSensor colorTop;

    private static final int BOTTOM_THRESHOLD = 300;
    private static final int TOP_THRESHOLD = 300;

    private int ballCount = 0;

    private boolean bottomDetected = false;
    private boolean topDetected = false;

    private boolean previousBottomDetected = false;
    private boolean previousTopDetected = false;

    private int bottomAlpha = 0;
    private int topAlpha = 0;

    private long lastBottomCount = 0;
    private long lastTopCount = 0;

    // =========================================
    // BUTTON MEMORY
    // =========================================

    private boolean lastShooter = false;
    private boolean lastTracking = false;
    private boolean lastAutoIntake = false;

    private boolean lastHoodUp = false;
    private boolean lastHoodDown = false;

    private boolean lastCombined = false;
    private boolean lastVision = false;
    private boolean lastPedro = false;

    private boolean lastTag = false;
    private boolean lastHome = false;
    private boolean lastReset = false;

    private boolean lastPidUp = false;
    private boolean lastPidDown = false;
    private boolean lastPidRight = false;
    private boolean lastPidLeft = false;

    // =========================================
    // INIT
    // =========================================

    @Override
    public void init() {

        // DRIVE
        drive = new DriveSubsystem(hardwareMap);

        drive.setInitialPose(
                START_X,
                START_Y,
                START_HEADING
        );

        driveCmd = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );

        // TURRET ORIGINAL
        turret = new TurretSubsystemV4(hardwareMap);

        // Mantener potencia de prueba
        turret.maxPower = 0.35;

        // LIMELIGHT ORIGINAL
        vision = new TurretVisionV5(hardwareMap);
        vision.setSelectedTag(selectedTag);

        // CONTROLADOR V76 ORIGINAL
        aiming = new TurretAimControllerV76(
                drive,
                turret,
                vision
        );

        aiming.setMode(
                TurretAimControllerV76.Mode.COMBINED
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

        // INTAKE
        intake = hardwareMap.get(
                DcMotor.class,
                "intake2"
        );

        indexer = hardwareMap.get(
                DcMotor.class,
                "intake1"
        );

        intake.setDirection(
                DcMotor.Direction.FORWARD
        );

        indexer.setDirection(
                DcMotor.Direction.REVERSE
        );

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        indexer.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        intake.setPower(0);
        indexer.setPower(0);

        // GATE
        gate = hardwareMap.tryGet(
                Servo.class,
                "gate"
        );

        if (gate != null) {
            gate.setDirection(
                    Servo.Direction.REVERSE
            );
        }

        setGate(false);

        // HOOD
        hood = hardwareMap.tryGet(
                Servo.class,
                "hood"
        );

        if (hood != null) {

            hood.setDirection(
                    Servo.Direction.FORWARD
            );

            hood.setPosition(
                    HOOD_POSITIONS[hoodIndex]
            );
        }

        // COLOR SENSORS
        colorBottom = hardwareMap.tryGet(
                ColorSensor.class,
                "colorBottom"
        );

        colorTop = hardwareMap.tryGet(
                ColorSensor.class,
                "colorTop"
        );

        telemetry.addLine("THUNDERROAR FULL V76");
        telemetry.addLine("Original V76 tracking");
        telemetry.addLine("No automatic HOME");
        telemetry.addData("Shooter", "%.0f ticks/s",
                targetVelocity);
        telemetry.update();
    }

    // =========================================
    // START
    // =========================================

    @Override
    public void start() {
        vision.start();
    }

    // =========================================
    // LOOP
    // =========================================

    @Override
    public void loop() {

        long now = System.nanoTime();

        // DRIVE
        driveCmd.execute();
        drive.update();

        // VISION
        vision.update();

        // =====================================
        // G1 A - SHOOTER
        // =====================================

        if (gamepad1.a && !lastShooter) {
            shooterEnabled = !shooterEnabled;
        }

        lastShooter = gamepad1.a;

        // =====================================
        // G1 B - TRACKING
        // =====================================

        if (gamepad1.b && !lastTracking) {

            if (trackingEnabled || returningHome) {

                aiming.disable();

                trackingEnabled = false;
                returningHome = false;

                turretStatus = "TRACKING_OFF";

            } else if (!turret.isFault()) {

                aiming.enable();

                trackingEnabled = aiming.isEnabled();

                turretStatus = trackingEnabled
                        ? "TRACKING_ON"
                        : "TRACKING_BLOCKED";
            }
        }

        lastTracking = gamepad1.b;

        // =====================================
        // G1 LB - AUTO INTAKE
        // =====================================

        if (gamepad1.left_bumper
                && !lastAutoIntake) {

            autoIntakeEnabled = !autoIntakeEnabled;
        }

        lastAutoIntake = gamepad1.left_bumper;

        // =====================================
        // G1 DPAD - HOOD
        // =====================================

        if (gamepad1.dpad_up && !lastHoodUp) {
            setHoodIndex(hoodIndex + 1);
        }

        if (gamepad1.dpad_down && !lastHoodDown) {
            setHoodIndex(hoodIndex - 1);
        }

        lastHoodUp = gamepad1.dpad_up;
        lastHoodDown = gamepad1.dpad_down;

        // =====================================
        // G2 A - COMBINED
        // =====================================

        if (gamepad2.a && !lastCombined) {

            aiming.setMode(
                    TurretAimControllerV76.Mode.COMBINED
            );
        }

        lastCombined = gamepad2.a;

        // =====================================
        // G2 LB - VISION ONLY
        // =====================================

        if (gamepad2.left_bumper
                && !lastVision) {

            aiming.setMode(
                    TurretAimControllerV76.Mode.VISION_ONLY
            );
        }

        lastVision = gamepad2.left_bumper;

        // =====================================
        // G2 RB - PEDRO ONLY
        // =====================================

        if (gamepad2.right_bumper
                && !lastPedro) {

            aiming.setMode(
                    TurretAimControllerV76.Mode.PEDRO_ONLY
            );
        }

        lastPedro = gamepad2.right_bumper;

        // =====================================
        // G2 R3 - CHANGE TAG
        // =====================================

        if (gamepad2.right_stick_button
                && !lastTag) {

            selectedTag =
                    selectedTag == 34 ? 37 : 34;

            vision.setSelectedTag(selectedTag);
        }

        lastTag = gamepad2.right_stick_button;

        // =====================================
        // G2 START - RESET FAULT
        // =====================================

        if (gamepad2.start && !lastReset) {

            aiming.disable();

            trackingEnabled = false;
            returningHome = false;

            turret.clearFault();

            turretStatus = "FAULT_RESET";
        }

        lastReset = gamepad2.start;

        // =====================================
        // G2 BACK - MANUAL HOME ONLY
        // =====================================

        if (gamepad2.back && !lastHome) {

            aiming.disable();

            trackingEnabled = false;

            if (!turret.isFault()) {

                turret.enableHold();

                turret.setVelocityFeedforward(0);
                turret.setTarget(0);

                returningHome = turret.isEnabled();

                turretStatus = "MANUAL_HOME";

            } else {

                returningHome = false;
                turretStatus = "HOME_BLOCKED";
            }
        }

        lastHome = gamepad2.back;

        // =====================================
        // SHOOTER VELOCITY +/- 5
        // =====================================

        if (now - lastVelocityChange
                > 200_000_000L) {

            if (gamepad1.right_trigger > 0.5) {

                targetVelocity += 5;
                lastVelocityChange = now;

            } else if (gamepad1.left_trigger > 0.5) {

                targetVelocity -= 5;
                lastVelocityChange = now;
            }
        }

        targetVelocity = Range.clip(
                targetVelocity,
                0,
                3000
        );

        // =====================================
        // SHOOTER PIDF - GAMEPAD 2
        // =====================================

        boolean pidChanged = false;

        if (gamepad2.dpad_up && !lastPidUp) {
            p += 0.2;
            pidChanged = true;
        }

        if (gamepad2.dpad_down && !lastPidDown) {
            f += 0.5;
            pidChanged = true;
        }

        if (gamepad2.dpad_right && !lastPidRight) {
            d += 0.1;
            pidChanged = true;
        }

        if (gamepad2.dpad_left && !lastPidLeft) {
            i += 0.01;
            pidChanged = true;
        }

        lastPidUp = gamepad2.dpad_up;
        lastPidDown = gamepad2.dpad_down;
        lastPidRight = gamepad2.dpad_right;
        lastPidLeft = gamepad2.dpad_left;

        if (pidChanged) {
            applyPIDF();
        }

        // =====================================
        // SHOOTER MOTOR
        // =====================================

        if (shooterEnabled) {

            shooter.setVelocity(targetVelocity);

        } else {

            shooter.setPower(0);
        }

        // =====================================
        // COLOR SENSORS
        // =====================================

        updateBallSensors(now);

        // =====================================
        // INTAKE - TODO GAMEPAD 1
        // =====================================

        boolean feeding = gamepad1.x;
        boolean manualForward = gamepad1.y;
        boolean manualReverse =
                gamepad1.right_bumper;

        intakePower = 0;
        indexerPower = 0;

        if (feeding) {

            setGate(true);

            intakePower = 1;
            indexerPower = 1;

        } else if (manualReverse) {

            setGate(false);

            intakePower = -1;
            indexerPower = -1;

        } else if (manualForward) {

            setGate(false);

            intakePower = 1;
            indexerPower = 1;

        } else if (autoIntakeEnabled) {

            setGate(false);

            if (colorBottom != null
                    && colorTop != null
                    && ballCount < 4) {

                intakePower = 1;

                indexerPower = topDetected
                        ? 0
                        : 1;
            }

        } else {

            setGate(false);
        }

        intake.setPower(intakePower);
        indexer.setPower(indexerPower);

        // =====================================
        // TURRET V76 - SIN HOME AUTOMATICO
        // =====================================

        updateTurretTracking();

        // =====================================
        // TELEMETRY
        // =====================================

        telemetry.addLine(
                "===== THUNDERROAR FULL V76 ====="
        );

        telemetry.addData("Robot X", "%.2f",
                drive.getX());

        telemetry.addData("Robot Y", "%.2f",
                drive.getY());

        telemetry.addData("Heading", "%.2f",
                drive.getHeadingDegrees());

        telemetry.addLine("===== TURRET =====");

        telemetry.addData("Tracking",
                trackingEnabled);

        telemetry.addData("Mode",
                aiming.getMode());

        telemetry.addData("Status",
                turretStatus);

        telemetry.addData("Actual", "%.2f",
                turret.getAngle());

        telemetry.addData("Target", "%.2f",
                turret.getTarget());

        telemetry.addData("Virtual Target", "%.2f",
                aiming.getBaseTarget());

        telemetry.addData("Smooth Target", "%.2f",
                aiming.getSmoothTarget());

        telemetry.addData("Error", "%.2f",
                turret.getError());

        telemetry.addData("Velocity", "%.2f",
                turret.getVelocity());

        telemetry.addData("Power", "%.2f",
                turret.getPower());

        telemetry.addData("Fault",
                turret.getFaultReason());

        telemetry.addData("Limits",
                "-80 / +230");

        telemetry.addLine("===== VISION =====");

        telemetry.addData("Tag", selectedTag);

        telemetry.addData("Visible",
                vision.hasTarget());

        telemetry.addData("Tx", "%.2f",
                vision.getTx());

        telemetry.addData("Filtered Tx", "%.2f",
                aiming.getFilteredTx());

        telemetry.addLine("===== SHOOTER =====");

        telemetry.addData("Shooter ON",
                shooterEnabled);

        telemetry.addData("Target ticks/s", "%.1f",
                targetVelocity);

        telemetry.addData("Actual ticks/s", "%.1f",
                shooter.getVelocity());

        telemetry.addData(
                "PIDF",
                "P %.2f I %.2f D %.2f F %.2f",
                p, i, d, f
        );

        telemetry.addLine("===== INTAKE =====");

        telemetry.addData("Auto Intake",
                autoIntakeEnabled);

        telemetry.addData("Ball Count",
                ballCount);

        telemetry.addData("Bottom Alpha",
                bottomAlpha);

        telemetry.addData("Top Alpha",
                topAlpha);

        telemetry.addData("Intake Power", "%.2f",
                intakePower);

        telemetry.addData("Indexer Power", "%.2f",
                indexerPower);

        telemetry.addData("Gate Open",
                gateOpen);

        telemetry.addLine("===== HOOD =====");

        telemetry.addData("Hood Index",
                hoodIndex);

        telemetry.addData("Hood Position", "%.4f",
                HOOD_POSITIONS[hoodIndex]);

        telemetry.update();
    }

    // =========================================
    // TRACKING V76 - CAMBIO SOLICITADO
    // =========================================

    private void updateTurretTracking() {

        if (trackingEnabled) {

            // Mantener el controlador V76 original.
            aiming.update();

            if (turret.isFault()
                    || !aiming.isEnabled()) {

                trackingEnabled = false;

                turretStatus = turret.isFault()
                        ? "TURRET_FAULT"
                        : aiming.getStatus();

                return;
            }

            double desired =
                    aiming.getSmoothTarget();

            double actual =
                    turret.getAngle();

            double minSafe =
                    TURRET_MIN_DEG
                            + LIMIT_MARGIN_DEG;

            double maxSafe =
                    TURRET_MAX_DEG
                            - LIMIT_MARGIN_DEG;

            // Buscar angulo equivalente
            // sin atravesar topes mecanicos.
            double best = Double.NaN;
            double bestDistance =
                    Double.POSITIVE_INFINITY;

            for (int k = -2; k <= 2; k++) {

                double candidate =
                        desired + 360.0 * k;

                if (candidate < minSafe
                        || candidate > maxSafe) {
                    continue;
                }

                double distance =
                        Math.abs(candidate - actual);

                if (distance < bestDistance) {

                    best = candidate;
                    bestDistance = distance;
                }
            }

            if (Double.isFinite(best)) {

                turret.setTarget(best);

                turretStatus =
                        "TRACKING_EQUIVALENT";

            } else {

                // No existe equivalente dentro
                // del recorrido de 310 grados.
                // No mandar HOME ni cruzar topes.
                double safeTarget = Range.clip(
                        desired,
                        minSafe,
                        maxSafe
                );

                turret.setTarget(safeTarget);

                turretStatus =
                        "TARGET_OUTSIDE_SAFE_ARC";
            }

        } else if (returningHome) {

            // HOME EXCLUSIVAMENTE MANUAL
            if (turret.isFault()
                    || !turret.isEnabled()) {

                returningHome = false;
                turretStatus = "HOME_FAULT";

            } else {

                turret.setVelocityFeedforward(0);
                turret.setTarget(0);
                turret.update();

                if (Math.abs(turret.getError()) <= 1.5
                        && Math.abs(
                        turret.getVelocity()
                ) < 8) {

                    turret.disable();

                    returningHome = false;
                    turretStatus = "HOME_REACHED";
                }
            }

        } else {

            turret.update();
        }
    }

    // =========================================
    // SHOOTER PIDF
    // =========================================

    private void applyPIDF() {

        shooter.setVelocityPIDFCoefficients(
                p, i, d, f
        );
    }

    // =========================================
    // GATE
    // =========================================

    private void setGate(boolean open) {

        gateOpen = open;

        if (gate == null) return;

        gate.setPosition(
                open
                        ? GATE_OPEN
                        : GATE_CLOSED
        );
    }

    // =========================================
    // HOOD
    // =========================================

    private void setHoodIndex(int index) {

        hoodIndex = Math.max(
                0,
                Math.min(
                        HOOD_POSITIONS.length - 1,
                        index
                )
        );

        if (hood != null) {

            hood.setPosition(
                    HOOD_POSITIONS[hoodIndex]
            );
        }
    }

    // =========================================
    // BALL SENSORS
    // =========================================

    private void updateBallSensors(long now) {

        bottomAlpha = colorBottom == null
                ? 0
                : colorBottom.alpha();

        topAlpha = colorTop == null
                ? 0
                : colorTop.alpha();

        bottomDetected =
                colorBottom != null
                        && bottomAlpha >= BOTTOM_THRESHOLD;

        topDetected =
                colorTop != null
                        && topAlpha >= TOP_THRESHOLD;

        // BALL ENTERING
        if (autoIntakeEnabled
                && !gamepad1.x
                && bottomDetected
                && !previousBottomDetected
                && now - lastBottomCount
                > 150_000_000L) {

            ballCount = Math.min(
                    4,
                    ballCount + 1
            );

            lastBottomCount = now;
        }

        // BALL LEAVING
        if (gamepad1.x
                && previousTopDetected
                && !topDetected
                && now - lastTopCount
                > 150_000_000L) {

            ballCount = Math.max(
                    0,
                    ballCount - 1
            );

            lastTopCount = now;
        }

        previousBottomDetected = bottomDetected;
        previousTopDetected = topDetected;
    }

    // =========================================
    // STOP
    // =========================================

    @Override
    public void stop() {

        if (shooter != null) {
            shooter.setPower(0);
        }

        if (intake != null) {
            intake.setPower(0);
        }

        if (indexer != null) {
            indexer.setPower(0);
        }

        setGate(false);

        if (aiming != null) {
            aiming.disable();
        }

        if (turret != null) {
            turret.stop();
        }

        if (vision != null) {
            vision.stop();
        }

        if (drive != null) {
            drive.stop();
        }
    }
}

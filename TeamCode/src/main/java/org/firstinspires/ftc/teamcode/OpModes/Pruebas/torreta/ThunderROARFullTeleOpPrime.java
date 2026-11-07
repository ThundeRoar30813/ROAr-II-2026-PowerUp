package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimingV9;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV4;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV3;

@TeleOp(name = "ThunderROAR PRIME V9 - FULL prubeaaaaaaaaa")
public class ThunderROARFullTeleOpPrime extends OpMode {

    // =====================================================
    // CONFIGURACION
    // =====================================================

    private static final String SHOOTER_NAME = "shoot";
    private static final String INTAKE_NAME = "intake2";
    private static final String INDEXER_NAME = "intake1";
    private static final String GATE_NAME = "gate";
    private static final String HOOD_NAME = "hood";

    // Verificar nombres en Robot Configuration.
    private static final String COLOR_BOTTOM_NAME = "colorBottom";
    private static final String COLOR_TOP_NAME = "colorTop";

    private static final double TAG_DISTANCE_IN = 56.0;

    // Margen provisional: verificar recorrido fisico.
    private static final double SAFE_MIN = -100.0;
    private static final double SAFE_MAX = 220.0;

    private static final double HOME_ANGLE = 0.0;
    private static final double HOME_TOLERANCE = 1.0;

    private static final double TURRET_POWER = 0.60;

    // =====================================================
    // CHASIS
    // =====================================================

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCommand;

    // =====================================================
    // TORRETA V9
    // =====================================================

    private TurretSubsystemV3 turret;
    private TurretVisionV4 vision;
    private TurretAimingV9 aiming;

    private boolean trackingEnabled = false;
    private boolean visionEnabled = false;
    private boolean returningHome = false;

    private String turretStatus = "NORMAL";

    // =====================================================
    // SHOOTER
    // =====================================================

    private DcMotorEx shooter;

    private boolean shooterEnabled = false;

    private double targetVelocity = 1100.0;

    private double p = 8.0;
    private double i = 0.0;
    private double d = 0.5;
    private double f = 18.0;

    private long lastVelocityChange = 0;

    // =====================================================
    // INTAKE / INDEXER
    // =====================================================

    private DcMotor intake;
    private DcMotor indexer;

    private boolean autoIntakeEnabled = false;

    private double intakePower = 0.0;
    private double indexerPower = 0.0;

    // =====================================================
    // GATE
    // =====================================================

    private Servo gate;

    private static final double GATE_CLOSED = 90.0 / 180.0;
    private static final double GATE_OPEN = 20.0 / 180.0;

    private boolean gateOpen = false;

    // =====================================================
    // CAPUCHA - SERVO POSICIONAL
    // =====================================================

    private Servo hood;

    private static final double[] HOOD_POSITIONS = {
            0.4444,
            0.5560,
            0.6667,
            0.7778,
            0.8890
    };

    private static final String[] HOOD_NAMES = {
            "ABAJO",
            "BAJA",
            "MEDIA",
            "ALTA",
            "ARRIBA"
    };

    private int hoodIndex = 2;
    private boolean hoodCommanded = false;

    // =====================================================
    // SENSORES
    // =====================================================

    private ColorSensor colorBottom;
    private ColorSensor colorTop;

    private static final int BOTTOM_THRESHOLD = 300;
    private static final int TOP_THRESHOLD = 300;

    private static final int MAX_BALLS = 4;

    private int ballCount = 0;

    private boolean bottomDetected = false;
    private boolean topDetected = false;

    private boolean previousBottomDetected = false;
    private boolean previousTopDetected = false;

    private int bottomAlpha = 0;
    private int topAlpha = 0;

    private static final long DEBOUNCE_NS = 150_000_000L;

    private long lastBottomCount = 0;
    private long lastTopCount = 0;

    // =====================================================
    // BOTONES ANTERIORES
    // =====================================================

    private boolean lastA1 = false;
    private boolean lastB1 = false;
    private boolean lastY1 = false;

    private boolean lastHoodUp = false;
    private boolean lastHoodDown = false;

    private boolean lastVisionA = false;
    private boolean lastAcquireY = false;
    private boolean lastHome = false;
    private boolean lastReset = false;

    private boolean lastPidUp = false;
    private boolean lastPidDown = false;
    private boolean lastPidRight = false;
    private boolean lastPidLeft = false;

    // =====================================================
    // INIT
    // =====================================================

    @Override
    public void init() {

        // ---------------- CHASIS ----------------

        drive = new DriveSubsystem(hardwareMap);

        // Strafe y giro corregidos respecto al V9.
        driveCommand = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );

        drive.setInitialPose(0, 0, 0);

        // ---------------- TORRETA ----------------

        turret = new TurretSubsystemV3(hardwareMap);

        turret.setMaxPower(TURRET_POWER);

        vision = new TurretVisionV4(hardwareMap);

        aiming = new TurretAimingV9(
                drive,
                turret,
                vision
        );

        // ---------------- SHOOTER ----------------

        shooter = hardwareMap.get(
                DcMotorEx.class,
                SHOOTER_NAME
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

        // ---------------- INTAKE ----------------

        intake = hardwareMap.get(
                DcMotor.class,
                INTAKE_NAME
        );

        indexer = hardwareMap.get(
                DcMotor.class,
                INDEXER_NAME
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

        // ---------------- GATE ----------------

        gate = hardwareMap.tryGet(
                Servo.class,
                GATE_NAME
        );

        if (gate != null) {
            gate.setDirection(Servo.Direction.REVERSE);
            // No mover gate durante INIT.
        }

        // ---------------- CAPUCHA ----------------

        hood = hardwareMap.tryGet(
                Servo.class,
                HOOD_NAME
        );

        if (hood != null) {
            hood.setDirection(Servo.Direction.FORWARD);
            // No mover hood durante INIT.
        }

        // ---------------- SENSORES ----------------

        colorBottom = hardwareMap.tryGet(
                ColorSensor.class,
                COLOR_BOTTOM_NAME
        );

        colorTop = hardwareMap.tryGet(
                ColorSensor.class,
                COLOR_TOP_NAME
        );

        telemetry.addLine("THUNDERROAR PRIME V9 READY");
        telemetry.addLine("G1 A Shooter | B Tracking");
        telemetry.addLine("G1 Y Auto Intake | X Disparar");
        telemetry.addLine("G1 DPAD UP/DOWN Capucha");
        telemetry.addLine("G2 A Vision | Y Acquire Tag");
        telemetry.addLine("G2 BACK Home | START Reset");
        telemetry.addData("Gate", gate != null);
        telemetry.addData("Hood", hood != null);
        telemetry.addData("Color Bottom", colorBottom != null);
        telemetry.addData("Color Top", colorTop != null);
        telemetry.update();
    }

    // =====================================================
    // START
    // =====================================================

    @Override
    public void start() {
        vision.start();
    }

    // =====================================================
    // LOOP
    // =====================================================

    @Override
    public void loop() {

        long now = System.nanoTime();

        // =================================================
        // 1. CHASIS
        // =================================================

        driveCommand.execute();
        drive.update();

        // =================================================
        // 2. LIMELIGHT
        // =================================================

        vision.update();

        // =================================================
        // 3. SHOOTER - G1 A
        // =================================================

        if (gamepad1.a && !lastA1) {
            shooterEnabled = !shooterEnabled;
        }

        lastA1 = gamepad1.a;

        // =================================================
        // 4. TRACKING - G1 B
        // =================================================

        if (gamepad1.b && !lastB1) {

            if (trackingEnabled) {

                aiming.disable();
                trackingEnabled = false;
                turretStatus = "TRACKING_OFF";

            } else if (!returningHome
                    && !turret.isStalled()
                    && turret.getAngle() > SAFE_MIN
                    && turret.getAngle() < SAFE_MAX) {

                aiming.enable();

                trackingEnabled = aiming.isEnabled();

                if (trackingEnabled) {
                    turretStatus = "TRACKING_ON";
                }
            }
        }

        lastB1 = gamepad1.b;

        // =================================================
        // 5. AUTO INTAKE - G1 Y
        // =================================================

        if (gamepad1.y && !lastY1) {
            autoIntakeEnabled = !autoIntakeEnabled;
        }

        lastY1 = gamepad1.y;

        // =================================================
        // 6. CAPUCHA - G1 DPAD
        // =================================================

        if (gamepad1.dpad_up && !lastHoodUp) {
            setHoodIndex(hoodIndex + 1);
        }

        if (gamepad1.dpad_down && !lastHoodDown) {
            setHoodIndex(hoodIndex - 1);
        }

        lastHoodUp = gamepad1.dpad_up;
        lastHoodDown = gamepad1.dpad_down;

        // =================================================
        // 7. VISION - G2 A
        // =================================================

        if (gamepad2.a && !lastVisionA) {

            visionEnabled = !visionEnabled;

            aiming.setVisionEnabled(visionEnabled);
        }

        lastVisionA = gamepad2.a;

        // =================================================
        // 8. ACQUIRE TAG 36 - G2 Y
        // =================================================

        if (gamepad2.y && !lastAcquireY
                && trackingEnabled
                && !returningHome) {

            aiming.acquireTarget(TAG_DISTANCE_IN);
        }

        lastAcquireY = gamepad2.y;

        // =================================================
        // 9. RESET FAULT - G2 START
        // =================================================

        if (gamepad2.start && !lastReset) {

            aiming.disable();

            trackingEnabled = false;
            returningHome = false;

            turret.clearFault();

            turretStatus = "FAULT_RESET";
        }

        lastReset = gamepad2.start;

        // =================================================
        // 10. RETURN HOME - G2 BACK
        // =================================================

        if (gamepad2.back && !lastHome) {

            aiming.disable();

            trackingEnabled = false;
            returningHome = false;

            if (!turret.isStalled()
                    && turret.getAngle() >= SAFE_MIN
                    && turret.getAngle() <= SAFE_MAX) {

                turret.enable();
                turret.setTarget(HOME_ANGLE);

                returningHome = turret.isEnabled();

                turretStatus = returningHome
                        ? "RETURNING_HOME"
                        : "HOME_DISABLED";

            } else {

                turretStatus = "HOME_BLOCKED";
            }
        }

        lastHome = gamepad2.back;

        // =================================================
        // 11. SHOOTER VELOCITY - G1 TRIGGERS
        // =================================================

        if (now - lastVelocityChange > 200_000_000L) {

            if (gamepad1.right_trigger > 0.5) {

                targetVelocity += 5.0;
                lastVelocityChange = now;

            } else if (gamepad1.left_trigger > 0.5) {

                targetVelocity -= 5.0;
                lastVelocityChange = now;
            }
        }

        targetVelocity = Range.clip(
                targetVelocity,
                0,
                3000
        );

        // =================================================
        // 12. SHOOTER PIDF - G2 DPAD
        // =================================================

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

        // =================================================
        // 13. SHOOTER MOTOR
        // =================================================

        if (shooterEnabled) {
            shooter.setVelocity(targetVelocity);
        } else {
            shooter.setPower(0);
        }

        // =================================================
        // 14. COLOR SENSORS
        // =================================================

        updateBallSensors(now);

        // =================================================
        // 15. INTAKE + INDEXER + GATE
        // =================================================

        boolean feeding = gamepad1.x;

        boolean manualForward = gamepad2.x;
        boolean manualReverse = gamepad2.b;

        intakePower = 0;
        indexerPower = 0;

        if (feeding) {

            setGate(true);

            intakePower = 1.0;
            indexerPower = 1.0;

        } else if (manualReverse) {

            setGate(false);

            intakePower = -1.0;
            indexerPower = -1.0;

        } else if (manualForward) {

            setGate(false);

            intakePower = 1.0;
            indexerPower = 1.0;

        } else if (autoIntakeEnabled) {

            setGate(false);

            if (colorBottom != null
                    && colorTop != null
                    && ballCount < MAX_BALLS) {

                intakePower = 1.0;

                indexerPower = topDetected
                        ? 0.0
                        : 1.0;
            }

        } else {

            setGate(false);
        }

        intake.setPower(intakePower);
        indexer.setPower(indexerPower);

        // =================================================
        // 16. TORRETA V9 + LIMITES
        // =================================================

        double actualAngle = turret.getAngle();

        if (trackingEnabled
                && (actualAngle <= SAFE_MIN
                || actualAngle >= SAFE_MAX)) {

            aiming.disable();
            trackingEnabled = false;

            turretStatus = "ANGLE_LIMIT";
        }

        if (trackingEnabled) {

            aiming.update();

            if (!aiming.isEnabled()) {

                trackingEnabled = false;

                turretStatus = turret.isStalled()
                        ? "TURRET_FAULT"
                        : "TRACKING_STOPPED";

            } else {

                double desired = aiming.getDesiredAngle();

                if (desired <= SAFE_MIN
                        || desired >= SAFE_MAX) {

                    aiming.disable();
                    trackingEnabled = false;

                    turretStatus = "TARGET_OUT_OF_RANGE";
                }
            }

        } else if (returningHome) {

            if (!turret.isEnabled()
                    || turret.isStalled()) {

                returningHome = false;
                turretStatus = "HOME_FAULT";

            } else {

                turret.setTarget(HOME_ANGLE);
                turret.update();

                if (Math.abs(turret.getAngle() - HOME_ANGLE)
                        <= HOME_TOLERANCE) {

                    turret.disable();
                    returningHome = false;

                    turretStatus = "HOME_REACHED";
                }
            }

        } else {

            turret.update();
        }

        // =================================================
        // 17. TELEMETRIA
        // =================================================

        telemetry.addLine("===== THUNDERROAR PRIME V9 =====");

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
                "Heading",
                "%.2f",
                drive.getHeadingDegrees()
        );

        // ---------------- TORRETA ----------------

        telemetry.addLine("===== TORRETA =====");

        telemetry.addData("Tracking", trackingEnabled);
        telemetry.addData("Vision", visionEnabled);
        telemetry.addData("Locked", aiming.hasLock());
        telemetry.addData("Status", aiming.getStatus());

        telemetry.addData(
                "Actual Angle",
                "%.2f",
                turret.getAngle()
        );

        telemetry.addData(
                "Desired Angle",
                "%.2f",
                aiming.getDesiredAngle()
        );

        telemetry.addData(
                "Commanded Angle",
                "%.2f",
                aiming.getCommandedAngle()
        );

        telemetry.addData(
                "Motor Lag",
                "%.2f",
                aiming.getCommandedAngle()
                        - turret.getAngle()
        );

        telemetry.addData(
                "Prediction",
                "%.3f",
                aiming.getPredictionDegrees()
        );

        telemetry.addData(
                "Turret Power",
                "%.2f",
                turret.getPower()
        );

        telemetry.addData(
                "Turret Fault",
                turret.getFaultReason()
        );

        telemetry.addData(
                "Turret Safety",
                turretStatus
        );

        telemetry.addData(
                "Returning Home",
                returningHome
        );

        // ---------------- LIMELIGHT ----------------

        telemetry.addLine("===== LIMELIGHT TAG 36 =====");

        telemetry.addData(
                "Tag Visible",
                vision.hasTarget()
        );

        telemetry.addData(
                "Tx",
                "%.2f",
                vision.getTx()
        );

        telemetry.addData(
                "Ty",
                "%.2f",
                vision.getTy()
        );

        telemetry.addData(
                "Frame Age (ms)",
                vision.getAgeMs()
        );

        telemetry.addData(
                "Vision Offset",
                "%.2f",
                aiming.getVisionOffset()
        );

        double distanceIn = aiming.getDistance();

        if (aiming.hasLock()
                && Double.isFinite(distanceIn)) {

            telemetry.addData(
                    "Distance (in)",
                    "%.2f",
                    distanceIn
            );

            telemetry.addData(
                    "Distance (cm)",
                    "%.2f",
                    distanceIn * 2.54
            );

        } else {

            telemetry.addData(
                    "Distance",
                    "NO LOCK"
            );
        }

        // ---------------- SHOOTER ----------------

        telemetry.addLine("===== SHOOTER =====");

        telemetry.addData(
                "Shooter",
                shooterEnabled
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

        telemetry.addData(
                "PIDF",
                "P %.2f I %.2f D %.2f F %.2f",
                p, i, d, f
        );

        // ---------------- INTAKE ----------------

        telemetry.addLine("===== INTAKE =====");

        telemetry.addData(
                "Auto Intake",
                autoIntakeEnabled
        );

        telemetry.addData(
                "Ball Count",
                ballCount
        );

        telemetry.addData(
                "Bottom Alpha",
                bottomAlpha
        );

        telemetry.addData(
                "Top Alpha",
                topAlpha
        );

        telemetry.addData(
                "Bottom Detected",
                bottomDetected
        );

        telemetry.addData(
                "Top Detected",
                topDetected
        );

        telemetry.addData(
                "Intake Power",
                "%.2f",
                intakePower
        );

        telemetry.addData(
                "Indexer Power",
                "%.2f",
                indexerPower
        );

        telemetry.addData(
                "Gate Open",
                gateOpen
        );

        // ---------------- CAPUCHA ----------------

        telemetry.addLine("===== CAPUCHA =====");

        telemetry.addData(
                "Hood Connected",
                hood != null
        );

        telemetry.addData(
                "Hood Preset",
                HOOD_NAMES[hoodIndex]
        );

        telemetry.addData(
                "Hood Position",
                "%.4f",
                HOOD_POSITIONS[hoodIndex]
        );

        telemetry.addData(
                "Hood Commanded",
                hoodCommanded
        );

        telemetry.update();
    }

    // =====================================================
    // CAPUCHA - CINCO POSICIONES
    // =====================================================

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

            hoodCommanded = true;
        }
    }

    // =====================================================
    // GATE
    // =====================================================

    private void setGate(boolean open) {

        gateOpen = open;

        if (gate == null) return;

        gate.setPosition(
                open ? GATE_OPEN : GATE_CLOSED
        );
    }

    // =====================================================
    // SENSORES DE COLOR
    // =====================================================

    private void updateBallSensors(long now) {

        bottomAlpha = 0;
        topAlpha = 0;

        bottomDetected = false;
        topDetected = false;

        if (colorBottom != null) {

            bottomAlpha = colorBottom.alpha();

            bottomDetected =
                    bottomAlpha >= BOTTOM_THRESHOLD;
        }

        if (colorTop != null) {

            topAlpha = colorTop.alpha();

            topDetected =
                    topAlpha >= TOP_THRESHOLD;
        }

        // Entrada de pelota.
        if (autoIntakeEnabled
                && colorBottom != null
                && bottomDetected
                && !previousBottomDetected
                && now - lastBottomCount > DEBOUNCE_NS) {

            ballCount = Math.min(
                    MAX_BALLS,
                    ballCount + 1
            );

            lastBottomCount = now;
        }

        // Salida de pelota al disparar.
        if (gamepad1.x
                && colorTop != null
                && previousTopDetected
                && !topDetected
                && now - lastTopCount > DEBOUNCE_NS) {

            ballCount = Math.max(
                    0,
                    ballCount - 1
            );

            lastTopCount = now;
        }

        previousBottomDetected = bottomDetected;
        previousTopDetected = topDetected;
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
            shooter.setPower(0);
        }

        if (intake != null) {
            intake.setPower(0);
        }

        if (indexer != null) {
            indexer.setPower(0);
        }

        if (gate != null) {
            setGate(false);
        }

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
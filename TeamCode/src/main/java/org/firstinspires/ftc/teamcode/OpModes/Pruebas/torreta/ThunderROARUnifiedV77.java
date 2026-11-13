package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV77;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV5;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

@TeleOp(name = "ThunderROAR UNIFIED V7.7 FINAL")
public class ThunderROARUnifiedV77 extends OpMode {

    // =========================================
    // 1. DRIVE & TORRETA V7.7 (EXACTO A DIAGNOSTIC1)
    // =========================================
    private DriveSubsystem drive;
    private MecanumDriveCmd driveCmd;

    private TurretSubsystemV4 turret;
    private TurretVisionV5 vision;
    private TurretAimControllerV77 aim;

    public static double FIELD_MID_X = 72.0;
    public static double SPLIT_HYSTERESIS_IN = 1.5;

    public static int LEFT_SIDE_TAG_ID = 37;
    public static int RIGHT_SIDE_TAG_ID = 35;
    public static double TEST_POWER = 1.0;

    private int activeTagId = LEFT_SIDE_TAG_ID;
    private boolean autoZoneSelection = true;

    private boolean tracking = false;
    private boolean home = false;

    // =========================================
    // 2. SHOOTER & PIDF
    // =========================================
    private DcMotorEx shooter;
    private boolean shooterEnabled = false;
    private double targetVelocity = 1000.0;

    private double p = 30.0;
    private double i = 0.0;
    private double d = 0.5;
    private double f = 16.0;

    private long lastVelocityChange = 0;

    // =========================================
    // 3. CAPUCHA / HOOD (G1 D-PAD ARRIBA/ABAJO)
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
    // 4. INTAKE, INDEXER, GATE & SENSORES (G1 X/Y)
    // =========================================
    private DcMotor intake;
    private DcMotor indexer;
    private Servo gate;

    private static final double GATE_CLOSED = 40.0 / 180.0;
    private static final double GATE_OPEN = 90.0 / 180.0;
    private boolean gateOpen = false;

    private ColorSensor colorBottom;
    private ColorSensor colorTop;
    private DistanceSensor distBottom;
    private DistanceSensor distTop;

    private static final double BOTTOM_DIST_CM = 2.0;
    private static final double TOP_DIST_CM = 2.0;
    private static final int BOTTOM_ALPHA_MIN = 350;
    private static final int TOP_ALPHA_MIN = 350;

    // Temporizador para no frenar el loop de la torreta con lecturas I2C
    private long lastSensorReadTime = 0;
    private static final long SENSOR_READ_INTERVAL_NS = 40_000_000L; // 40 ms

    private int ballCount = 0;
    private boolean bottomDetected = false;
    private boolean topDetected = false;
    private boolean previousBottomDetected = false;
    private long lastBottomCountTime = 0;
    private boolean wasReleasing = false;

    // =========================================
    // 5. RAMPA (G2 X / A)
    // =========================================
    private CRServo rampap;
    private DcMotor Encoramp;

    private static final double TICKS_POR_REV_ENCODER = 8192.0;
    private static final double DIENTES_ENCODER = 12.0;
    private static final double DIENTES_RAMPA = 26.0;
    private static final double TICKS_POR_GRADO_RAMPA =
            (TICKS_POR_REV_ENCODER * (DIENTES_RAMPA / DIENTES_ENCODER)) / 360.0;

    private static final int TICKS_ARRIBA = 0;
    private static final int TICKS_OBJETIVO = 6484;

    private static final double KP_RAMPA = 0.0008;
    private static final int TOLERANCIA_TICKS = 35;
    private static final double MIN_POWER_RAMPA = 0.15;
    private static final double MAX_POWER_RAMPA = 0.75;

    private int targetTicksRampa = TICKS_ARRIBA;
    private boolean rampaModoAuto = false;
    private int offsetTicksRampa = 0;

    // =========================================
    // MEMORIA DE BOTONES
    // =========================================
    private boolean lastG1B = false;
    private boolean lastG1A = false;
    private boolean lastG1L1 = false;
    private boolean lastG1R1 = false;
    private boolean lastG1Left = false;
    private boolean lastG1Right = false;
    private boolean lastG1Up = false;
    private boolean lastG1Down = false;
    private boolean lastG1Back = false;

    private boolean lastG2A = false;
    private boolean lastG2X = false;
    private boolean lastG2Y = false;
    private boolean lastPidUp = false;
    private boolean lastPidDown = false;
    private boolean lastPidRight = false;
    private boolean lastPidLeft = false;

    @Override
    public void init() {
        // DRIVE (Idéntico a Diagnostic1)
        drive = new DriveSubsystem(hardwareMap);
        drive.setInitialPose(48, 6, 90);

        driveCmd = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );

        // TORRETA V7.7 (Idéntico a Diagnostic1)
        turret = new TurretSubsystemV4(hardwareMap);
        turret.maxPower = TEST_POWER;

        vision = new TurretVisionV5(hardwareMap);

        aim = new TurretAimControllerV77(
                drive,
                turret,
                vision
        );
        aim.setMode(TurretAimControllerV77.Mode.COMBINED);

        // SHOOTER (PIDF)
        shooter = hardwareMap.get(DcMotorEx.class, "shoot");
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPower(0);
        applyPIDF();

        // HOOD
        hood = hardwareMap.tryGet(Servo.class, "hood");
        if (hood != null) {
            hood.setDirection(Servo.Direction.FORWARD);
            hood.setPosition(HOOD_POSITIONS[hoodIndex]);
        }

        // INTAKE E INDEXER
        intake = hardwareMap.get(DcMotor.class, "intake1");
        indexer = hardwareMap.get(DcMotor.class, "intake2");

        intake.setDirection(DcMotor.Direction.REVERSE);
        indexer.setDirection(DcMotor.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setPower(0);
        indexer.setPower(0);

        // GATE
        gate = hardwareMap.tryGet(Servo.class, "gate");
        if (gate != null) {
            gate.setDirection(Servo.Direction.REVERSE);
        }
        setGate(false);

        // SENSORES REV
        colorBottom = hardwareMap.tryGet(ColorSensor.class, "colorV2");
        colorTop = hardwareMap.tryGet(ColorSensor.class, "colorV3");

        if (colorBottom instanceof DistanceSensor) {
            distBottom = (DistanceSensor) colorBottom;
        }
        if (colorTop instanceof DistanceSensor) {
            distTop = (DistanceSensor) colorTop;
        }

        // RAMPA
        rampap = hardwareMap.get(CRServo.class, "rampa");
        rampap.setDirection(DcMotorSimple.Direction.FORWARD);
        rampap.setPower(0.0);

        Encoramp = hardwareMap.get(DcMotor.class, "mbl");
        Encoramp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Encoramp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("THUNDERROAR UNIFIED V7.7 (LOOP RAPIDO)");
        telemetry.update();
    }

    @Override
    public void start() {
        vision.start();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();

        // ==================================
        // 1. DRIVE (EXACTO A DIAGNOSTIC1)
        // ==================================
        driveCmd.execute();
        drive.update();

        // ==================================
        // 2. SELECCIÓN AUTOMÁTICA DE TAG (EXACTO A DIAGNOSTIC1)
        // ==================================
        if (gamepad1.back && !lastG1Back) {
            autoZoneSelection = !autoZoneSelection;
        }
        lastG1Back = gamepad1.back;

        double robotX = drive.getX();
        if (autoZoneSelection && Double.isFinite(robotX)) {
            if (robotX < (FIELD_MID_X - SPLIT_HYSTERESIS_IN)) {
                activeTagId = LEFT_SIDE_TAG_ID;
            } else if (robotX > (FIELD_MID_X + SPLIT_HYSTERESIS_IN)) {
                activeTagId = RIGHT_SIDE_TAG_ID;
            }
        }

        // ==================================
        // 3. LIMELIGHT & MODO DE APUNTADO (EXACTO A DIAGNOSTIC1)
        // ==================================
        vision.setSelectedTag(activeTagId);
        vision.update();

        if (gamepad1.dpad_left && !lastG1Left) {
            aim.setMode(TurretAimControllerV77.Mode.VISION_ONLY);
        }
        if (gamepad1.dpad_right && !lastG1Right) {
            aim.setMode(TurretAimControllerV77.Mode.PEDRO_ONLY);
        }
        lastG1Left = gamepad1.dpad_left;
        lastG1Right = gamepad1.dpad_right;

        // ==================================
        // 4. TRACKING ON/OFF, HOME, RESET & UPDATE TORRETA (EXACTO A DIAGNOSTIC1)
        // ==================================
        if (gamepad1.b && !lastG1B) {
            if (tracking) {
                aim.disable();
                tracking = false;
            } else {
                home = false;
                aim.enable();
                tracking = aim.isEnabled();
            }
        }
        lastG1B = gamepad1.b;

        if (gamepad1.left_bumper && !lastG1L1) {
            aim.disable();
            tracking = false;
            turret.enableHold();
            turret.setTarget(0);
            home = turret.isEnabled();
        }
        lastG1L1 = gamepad1.left_bumper;

        if (gamepad1.right_bumper && !lastG1R1) {
            aim.disable();
            tracking = false;
            home = false;
            turret.clearFault();
        }
        lastG1R1 = gamepad1.right_bumper;

        if (tracking) {
            aim.update();
            if (!aim.isEnabled() || turret.isFault()) {
                tracking = false;
            }
        } else if (home) {
            turret.setVelocityFeedforward(0);
            turret.update();

            if (turret.isFault()) {
                home = false;
            } else if (Math.abs(turret.getError()) <= 1.0 && Math.abs(turret.getVelocity()) < 8.0) {
                turret.disable();
                home = false;
            }
        } else {
            turret.update();
        }

        // ==================================
        // 5. CAPUCHA / HOOD (G1 D-PAD ARRIBA / ABAJO)
        // ==================================
        if (gamepad1.dpad_up && !lastG1Up) {
            setHoodIndex(hoodIndex + 1);
        }
        if (gamepad1.dpad_down && !lastG1Down) {
            setHoodIndex(hoodIndex - 1);
        }
        lastG1Up = gamepad1.dpad_up;
        lastG1Down = gamepad1.dpad_down;

        // ==================================
        // 6. SHOOTER (G1 A & RT/LT | G2 D-PAD PIDF)
        // ==================================
        if (gamepad1.a && !lastG1A) {
            shooterEnabled = !shooterEnabled;
        }
        lastG1A = gamepad1.a;

        if (now - lastVelocityChange > 200_000_000L) {
            if (gamepad1.right_trigger > 0.5) {
                targetVelocity += 20;
                lastVelocityChange = now;
            } else if (gamepad1.left_trigger > 0.5) {
                targetVelocity -= 20;
                lastVelocityChange = now;
            }
        }
        targetVelocity = Range.clip(targetVelocity, 0, 3000);

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

        if (shooterEnabled) {
            shooter.setVelocity(targetVelocity);
        } else {
            shooter.setPower(0);
        }

        // ==================================
        // 7. LECTURA DE SENSORES I2C SIN BLOQUEAR EL LOOP (CADA 40 MS)
        // ==================================
        if (now - lastSensorReadTime >= SENSOR_READ_INTERVAL_NS) {
            lastSensorReadTime = now;

            double bottomCm = (distBottom != null) ? distBottom.getDistance(DistanceUnit.CM) : 999.0;
            double topCm = (distTop != null) ? distTop.getDistance(DistanceUnit.CM) : 999.0;

            int bottomAlpha = (colorBottom != null) ? colorBottom.alpha() : 0;
            int topAlpha = (colorTop != null) ? colorTop.alpha() : 0;

            bottomDetected = (colorBottom != null) && (bottomCm < BOTTOM_DIST_CM || bottomAlpha >= BOTTOM_ALPHA_MIN);
            topDetected = (colorTop != null) && (topCm < TOP_DIST_CM || topAlpha >= TOP_ALPHA_MIN);
        }

        if (!gamepad1.y) {
            if (bottomDetected && !previousBottomDetected && (now - lastBottomCountTime > 180_000_000L)) {
                ballCount++;
                lastBottomCountTime = now;
            }
        }
        previousBottomDetected = bottomDetected;

        // ==================================
        // 8. INTAKE & GATE (G1 X INTAKEAR / Y LIBERAR)
        // ==================================
        double intakePower = 0.0;
        double indexerPower = 0.0;

        if (gamepad1.y) {
            wasReleasing = true;
            setGate(true);
            intakePower = 1.0;
            indexerPower = 1.0;
        } else {
            if (wasReleasing) {
                ballCount = 0;
                wasReleasing = false;
            }

            if (gamepad1.x) {
                setGate(false);
                intakePower = 1.0;
                indexerPower = topDetected ? 0.0 : 0.5;
            } else {
                setGate(false);
                intakePower = 0.0;
                indexerPower = 0.0;
            }
        }

        intake.setPower(intakePower);
        indexer.setPower(indexerPower);

        // ==================================
        // 9. RAMPA (G2 X TARGET / A CERO)
        // ==================================
        int rawTicksRampa = Encoramp.getCurrentPosition();
        int ticksActualesRampa = rawTicksRampa - offsetTicksRampa;
        double gradosRampa = ticksActualesRampa / TICKS_POR_GRADO_RAMPA;

        if (gamepad2.x && !lastG2X) {
            targetTicksRampa = TICKS_OBJETIVO;
            rampaModoAuto = true;
        }
        lastG2X = gamepad2.x;

        if (gamepad2.a && !lastG2A) {
            targetTicksRampa = TICKS_ARRIBA;
            rampaModoAuto = true;
        }
        lastG2A = gamepad2.a;

        if (gamepad2.y && !lastG2Y) {
            offsetTicksRampa = rawTicksRampa;
            targetTicksRampa = TICKS_ARRIBA;
            rampaModoAuto = false;
        }
        lastG2Y = gamepad2.y;

        double rampaPower = 0.0;
        int errorRampa = targetTicksRampa - ticksActualesRampa;

        if (gamepad2.right_bumper) {
            rampaModoAuto = false;
            rampaPower = 0.35;
        } else if (gamepad2.left_bumper) {
            rampaModoAuto = false;
            rampaPower = -0.35;
        } else if (rampaModoAuto) {
            if (Math.abs(errorRampa) <= TOLERANCIA_TICKS) {
                rampaPower = 0.0;
            } else {
                double rawPower = errorRampa * KP_RAMPA;
                double signo = Math.signum(rawPower);
                double magnitud = Range.clip(Math.abs(rawPower), MIN_POWER_RAMPA, MAX_POWER_RAMPA);
                rampaPower = signo * magnitud;
            }
        }

        rampap.setPower(rampaPower);

        // ==================================
        // 10. TELEMETRÍA
        // ==================================
        String fieldSide = (robotX < FIELD_MID_X) ? "IZQ (X<72)" : "DER (X>=72)";

        telemetry.addLine("===== TORRETA V7.7 =====");
        telemetry.addData("Zona | Tag", "%s | Tag %d (Auto: %s)", fieldSide, activeTagId, autoZoneSelection ? "ON" : "OFF");
        telemetry.addData("Mode | Track", "%s | %s", aim.getMode(), tracking ? "ON" : "OFF");
        telemetry.addData("Ang / Tgt / Err", "%.1f / %.1f / %.2f", turret.getAngle(), aim.getBaseTarget(), turret.getError());
        telemetry.addData("Power | Fault", "%.3f | %s", turret.getPower(), turret.getFaultReason());

        telemetry.addLine("===== SHOOTER & HOOD =====");
        telemetry.addData("Shooter (G1 A)", shooterEnabled ? "ON" : "OFF");
        telemetry.addData("Vel Tgt / Act", "%.0f / %.0f", targetVelocity, shooter.getVelocity());
        telemetry.addData("PIDF (G2 DPad)", "P:%.2f I:%.2f D:%.2f F:%.2f", p, i, d, f);
        telemetry.addData("Hood (G1 U/D)", "Idx %d (%.4f)", hoodIndex, HOOD_POSITIONS[hoodIndex]);

        telemetry.addLine("===== INTAKE & RAMPA =====");
        telemetry.addData("Piezas | Gate", "%d | %s", ballCount, gateOpen ? "ABIERTA" : "CERRADA");
        telemetry.addData("Rampa Tgt / Act", "%d / %d (%.1f°)", targetTicksRampa, ticksActualesRampa, gradosRampa);
        telemetry.update();
    }

    private void applyPIDF() {
        shooter.setVelocityPIDFCoefficients(p, i, d, f);
    }

    private void setGate(boolean open) {
        gateOpen = open;
        if (gate != null) {
            gate.setPosition(open ? GATE_OPEN : GATE_CLOSED);
        }
    }

    private void setHoodIndex(int index) {
        hoodIndex = Math.max(0, Math.min(HOOD_POSITIONS.length - 1, index));
        if (hood != null) {
            hood.setPosition(HOOD_POSITIONS[hoodIndex]);
        }
    }

    @Override
    public void stop() {
        if (aim != null) aim.disable();
        if (turret != null) turret.stop();
        if (vision != null) vision.stop();
        if (drive != null) drive.stop();
        if (shooter != null) shooter.setPower(0);
        if (intake != null) intake.setPower(0);
        if (indexer != null) indexer.setPower(0);
        if (rampap != null) rampap.setPower(0);
        setGate(false);
    }
}
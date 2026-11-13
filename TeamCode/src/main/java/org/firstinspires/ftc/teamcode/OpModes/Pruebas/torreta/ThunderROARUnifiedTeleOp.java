package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV77;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV5;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

@TeleOp(name = "ThunderROAR UNIFIED V7.7")
public class ThunderROARUnifiedTeleOp extends OpMode {

    // =========================================
    // DRIVE & PEDRO PATHING
    // =========================================
    private DriveSubsystem drive;
    private MecanumDriveCmd driveCmd;

    // =========================================
    // TORRETA V7.7 (CONTROL 1)
    // =========================================
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
    // SHOOTER + PIDF + VELOCIDAD
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
    // INTAKE, INDEXER & GATE (CONTROL 2)
    // =========================================
    private DcMotor intake;
    private DcMotor indexer;
    private Servo gate;

    private static final double GATE_CLOSED = 40.0 / 180.0;
    private static final double GATE_OPEN = 90.0 / 180.0;
    private boolean gateOpen = false;

    private boolean autoIntakeEnabled = false;
    private boolean wasShooting = false;

    // =========================================
    // HOOD (CONTROL 2)
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
    // SENSORES DE COLOR / DISTANCIA REV
    // =========================================
    private ColorSensor colorBottom;       // REV V2 (Entrada Intake)
    private ColorSensor colorTop;          // REV V3 (Tope Shooter)
    private DistanceSensor distBottom;
    private DistanceSensor distTop;

    private static final double BOTTOM_DIST_CM = 2.0;
    private static final double TOP_DIST_CM = 2.0;
    private static final int BOTTOM_ALPHA_MIN = 350;
    private static final int TOP_ALPHA_MIN = 350;

    private int ballCount = 0;
    private boolean bottomDetected = false;
    private boolean topDetected = false;
    private boolean previousBottomDetected = false;
    private long lastBottomCountTime = 0;

    // =========================================
    // MEMORIA DE BOTONES (DEBOUNCE)
    // =========================================
    // Gamepad 1
    private boolean lastG1B = false;
    private boolean lastG1A = false;
    private boolean lastG1L1 = false;
    private boolean lastG1R1 = false;
    private boolean lastG1Left = false;
    private boolean lastG1Right = false;
    private boolean lastG1Up = false;
    private boolean lastG1Down = false;

    // Gamepad 2
    private boolean lastG2LB = false;
    private boolean lastG2RB = false;
    private boolean lastG2A = false;
    private boolean lastG2B = false;
    private boolean lastPidUp = false;
    private boolean lastPidDown = false;
    private boolean lastPidRight = false;
    private boolean lastPidLeft = false;

    @Override
    public void init() {
        // 1. DRIVE
        drive = new DriveSubsystem(hardwareMap);
        drive.setInitialPose(48, 6, 90);

        driveCmd = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );

        // 2. TORRETA V7.7
        turret = new TurretSubsystemV4(hardwareMap);
        turret.maxPower = TEST_POWER;

        vision = new TurretVisionV5(hardwareMap);

        aim = new TurretAimControllerV77(
                drive,
                turret,
                vision
        );
        aim.setMode(TurretAimControllerV77.Mode.COMBINED);

        // 3. SHOOTER (CON PIDF Y ENCODER)
        shooter = hardwareMap.get(DcMotorEx.class, "shoot");
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPower(0);
        applyPIDF();

        // 4. INTAKE E INDEXER
        intake = hardwareMap.get(DcMotor.class, "intake1");
        indexer = hardwareMap.get(DcMotor.class, "intake2");

        intake.setDirection(DcMotor.Direction.REVERSE);
        indexer.setDirection(DcMotor.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setPower(0);
        indexer.setPower(0);

        // 5. GATE
        gate = hardwareMap.tryGet(Servo.class, "gate");
        if (gate != null) {
            gate.setDirection(Servo.Direction.REVERSE);
        }
        setGate(false);

        // 6. HOOD
        hood = hardwareMap.tryGet(Servo.class, "hood");
        if (hood != null) {
            hood.setDirection(Servo.Direction.FORWARD);
            hood.setPosition(HOOD_POSITIONS[hoodIndex]);
        }

        // 7. SENSORES REV
        colorBottom = hardwareMap.tryGet(ColorSensor.class, "colorV2");
        colorTop = hardwareMap.tryGet(ColorSensor.class, "colorV3");

        if (colorBottom instanceof DistanceSensor) {
            distBottom = (DistanceSensor) colorBottom;
        }
        if (colorTop instanceof DistanceSensor) {
            distTop = (DistanceSensor) colorTop;
        }

        telemetry.addLine("THUNDERROAR UNIFIED V7.7 LISTO");
        telemetry.addLine("G1: Sticks Drive | B: Track | A: Shooter | RT/LT: Vel +/-5");
        telemetry.addLine("G2: X: Shoot+Gate | Y: Intake Man | LB: AutoIn | RB: ResetCnt");
        telemetry.addLine("G2: A/B: Hood +/- | D-Pad: Ajuste PIDF");
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
        // 1. DRIVE (GAMEPAD 1)
        // ==================================
        driveCmd.execute();
        drive.update();

        // ==================================
        // 2. TORRETA V7.7 (GAMEPAD 1)
        // ==================================
        if (gamepad1.dpad_down && !lastG1Down) {
            autoZoneSelection = !autoZoneSelection;
        }
        lastG1Down = gamepad1.dpad_down;

        double robotX = drive.getX();
        if (autoZoneSelection && Double.isFinite(robotX)) {
            if (robotX < (FIELD_MID_X - SPLIT_HYSTERESIS_IN)) {
                activeTagId = LEFT_SIDE_TAG_ID;
            } else if (robotX > (FIELD_MID_X + SPLIT_HYSTERESIS_IN)) {
                activeTagId = RIGHT_SIDE_TAG_ID;
            }
        }

        vision.setSelectedTag(activeTagId);
        vision.update();

        if (gamepad1.dpad_left && !lastG1Left) {
            aim.setMode(TurretAimControllerV77.Mode.VISION_ONLY);
        }
        if (gamepad1.dpad_right && !lastG1Right) {
            aim.setMode(TurretAimControllerV77.Mode.PEDRO_ONLY);
        }
        if (gamepad1.dpad_up && !lastG1Up) {
            aim.setMode(TurretAimControllerV77.Mode.COMBINED);
        }
        lastG1Left = gamepad1.dpad_left;
        lastG1Right = gamepad1.dpad_right;
        lastG1Up = gamepad1.dpad_up;

        // B (G1): TRACKING ON/OFF
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

        // L1 (G1): HOME
        if (gamepad1.left_bumper && !lastG1L1) {
            aim.disable();
            tracking = false;
            turret.enableHold();
            turret.setTarget(0);
            home = turret.isEnabled();
        }
        lastG1L1 = gamepad1.left_bumper;

        // R1 (G1): RESET FAULT
        if (gamepad1.right_bumper && !lastG1R1) {
            aim.disable();
            tracking = false;
            home = false;
            turret.clearFault();
        }
        lastG1R1 = gamepad1.right_bumper;

        // UPDATE TORRETA
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
        // 3. SHOOTER VELOCIDAD & ON/OFF (GAMEPAD 1) + PIDF (GAMEPAD 2)
        // ==================================
        if (gamepad1.a && !lastG1A) {
            shooterEnabled = !shooterEnabled;
        }
        lastG1A = gamepad1.a;

        // Subir / Bajar velocidad +/- 5 con gatillos de Gamepad 1
        if (now - lastVelocityChange > 200_000_000L) {
            if (gamepad1.right_trigger > 0.5) {
                targetVelocity += 5;
                lastVelocityChange = now;
            } else if (gamepad1.left_trigger > 0.5) {
                targetVelocity -= 5;
                lastVelocityChange = now;
            }
        }
        targetVelocity = Range.clip(targetVelocity, 0, 3000);

        // Ajuste PIDF en Gamepad 2 (D-Pad)
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

        // ==================================
        // 4. HOOD (GAMEPAD 2: A / B)
        // ==================================
        if (gamepad2.a && !lastG2A) {
            setHoodIndex(hoodIndex + 1);
        }
        if (gamepad2.b && !lastG2B) {
            setHoodIndex(hoodIndex - 1);
        }
        lastG2A = gamepad2.a;
        lastG2B = gamepad2.b;

        // ==================================
        // 5. SENSORES & INTAKE / GATE (GAMEPAD 2: X / Y)
        // ==================================
        double bottomCm = (distBottom != null) ? distBottom.getDistance(DistanceUnit.CM) : 999.0;
        double topCm = (distTop != null) ? distTop.getDistance(DistanceUnit.CM) : 999.0;

        int bottomAlpha = (colorBottom != null) ? colorBottom.alpha() : 0;
        int topAlpha = (colorTop != null) ? colorTop.alpha() : 0;

        bottomDetected = (colorBottom != null) && (bottomCm < BOTTOM_DIST_CM || bottomAlpha >= BOTTOM_ALPHA_MIN);
        topDetected = (colorTop != null) && (topCm < TOP_DIST_CM || topAlpha >= TOP_ALPHA_MIN);

        // Contar piezas en entrada (mientras no se esté disparando con G2.X)
        if (!gamepad2.x) {
            if (bottomDetected && !previousBottomDetected && (now - lastBottomCountTime > 180_000_000L)) {
                ballCount++;
                lastBottomCountTime = now;
            }
        }
        previousBottomDetected = bottomDetected;

        // G2 LB: Auto Intake ON/OFF
        if (gamepad2.left_bumper && !lastG2LB) {
            if (!autoIntakeEnabled && ballCount >= 4) {
                ballCount = 0;
            }
            autoIntakeEnabled = !autoIntakeEnabled;
        }
        lastG2LB = gamepad2.left_bumper;

        // G2 RB: Reiniciar contador a 0
        if (gamepad2.right_bumper && !lastG2RB) {
            ballCount = 0;
        }
        lastG2RB = gamepad2.right_bumper;

        // Máquina de estados Intake / Indexer / Gate / Shooter
        double intakePower = 0;
        double indexerPower = 0;

        boolean shootBtn = gamepad2.x;         // X en Control 2: Disparo + Gate
        boolean manualIntakeBtn = gamepad2.y;  // Y en Control 2: Meter manual

        if (shootBtn) {
            // PRIORIDAD 1: DISPARO (Gate abierta + alimenta al 100%)
            autoIntakeEnabled = false;
            wasShooting = true;

            setGate(true);
            intakePower = 1.0;
            indexerPower = 1.0;

            // Si el shooter está encendido con PIDF usa targetVelocity, si no, lo impulsa al 100%
            if (shooterEnabled) {
                shooter.setVelocity(targetVelocity);
            } else {
                shooter.setPower(1.0);
            }

        } else {
            if (wasShooting) {
                ballCount = 0;
                wasShooting = false;
            }

            // Control normal del shooter cuando no se presiona X
            if (shooterEnabled) {
                shooter.setVelocity(targetVelocity);
            } else {
                shooter.setPower(0);
            }

            if (manualIntakeBtn) {
                // PRIORIDAD 2: METER CON Y (Intake 100%, Indexer 50% hasta sensor arriba)
                setGate(false);
                intakePower = 1.0;
                indexerPower = topDetected ? 0.0 : 0.5;

            } else if (autoIntakeEnabled) {
                // PRIORIDAD 3: CARGA AUTOMÁTICA CON LB
                setGate(false);
                if (ballCount >= 4) {
                    intakePower = 0.0;
                    indexerPower = 0.0;
                    autoIntakeEnabled = false;
                } else {
                    intakePower = 1.0;
                    indexerPower = topDetected ? 0.0 : 0.5;
                }

            } else {
                // REPOSO
                setGate(false);
                intakePower = 0.0;
                indexerPower = 0.0;
            }
        }

        intake.setPower(intakePower);
        indexer.setPower(indexerPower);

        // ==================================
        // 6. TELEMETRÍA UNIFICADA
        // ==================================
        String fieldSide = (robotX < FIELD_MID_X) ? "IZQUIERDA (X < 72)" : "DERECHA (X >= 72)";

        telemetry.addLine("===== TORRETA V7.7 (G1) =====");
        telemetry.addData("Zona | Tag", "%s | Tag %d (Auto: %s)", fieldSide, activeTagId, autoZoneSelection ? "ON" : "OFF");
        telemetry.addData("Mode | Status", "%s | %s", aim.getMode(), aim.getStatus());
        telemetry.addData("Tracking | Ready", "%s | %s", tracking ? "ON" : "OFF", turret.isReadyToShoot(2.5) ? "SI" : "NO");
        telemetry.addData("Angulo / Target / Err", "%.1f / %.1f / %.2f", turret.getAngle(), aim.getBaseTarget(), turret.getError());
        telemetry.addData("Limelight", "Vis: %s | Tx: %.2f", vision.hasTarget(), vision.getTx());

        telemetry.addLine("===== SHOOTER & HOOD =====");
        telemetry.addData("Shooter (G1 A)", shooterEnabled ? "ON (PIDF)" : "OFF");
        telemetry.addData("Vel Target / Actual", "%.0f / %.0f ticks/s", targetVelocity, shooter.getVelocity());
        telemetry.addData("PIDF (G2 DPad)", "P:%.2f I:%.2f D:%.2f F:%.2f", p, i, d, f);
        telemetry.addData("Hood (G2 A/B)", "Idx %d (Pos: %.4f)", hoodIndex, HOOD_POSITIONS[hoodIndex]);

        telemetry.addLine("===== INTAKE & SENSORES (G2) =====");
        telemetry.addData("Piezas | AutoIntake", "%d / 4 | %s", ballCount, autoIntakeEnabled ? "ON" : "OFF");
        telemetry.addData("Gate | In / Idx", "%s | %.2f / %.2f", gateOpen ? "ABIERTA" : "CERRADA", intakePower, indexerPower);
        telemetry.addData("V2 Abajo | V3 Arriba", "%s (%.1fcm) | %s (%.1fcm)",
                bottomDetected ? "SI" : "NO", bottomCm,
                topDetected ? "SI" : "NO", topCm);
        telemetry.update();
    }

    // =========================================
    // MÉTODOS AUXILIARES
    // =========================================
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
        setGate(false);
    }
}

package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.turret.ZoneTargetSelector;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV5;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV6;

@TeleOp(name = "BIOBUZZ FULL TELEOP V6")
public class BioBuzzFullTeleOp extends OpMode {

    // =========================================
    // SUBSYSTEMS
    // =========================================

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCmd;

    private TurretSubsystemV4 turret;
    private TurretVisionV5 vision;
    private TurretAimControllerV6 aim;
    private ZoneTargetSelector zones;

    // =========================================
    // HARDWARE
    // =========================================

    private DcMotorEx shooter;
    private DcMotorEx intake;
    private DcMotorEx indexer;

    private ColorSensor colorV2;
    private ColorSensor colorV3;

    private Servo gate;

    private CRServo hood1;
    private CRServo hood2;

    // Los servos hood son opcionales.
    // Cambia estos nombres si son distintos.
    private static final String HOOD_1_NAME = "hood1";
    private static final String HOOD_2_NAME = "hood2";

    // =========================================
    // CONFIGURACION
    // =========================================

    public static double SHOOTER_START_RPM = 1100;
    public static double SHOOTER_STEP = 5;
    public static double SHOOTER_MAX = 3000;

    public static double INTAKE_POWER = 1.0;
    public static double INDEXER_POWER = 1.0;

    // REV Smart Robot Servo.
    // Se asume recorrido configurado de 270 grados.
    // Verificar antes de operar.
    public static double GATE_CLOSED = 90.0 / 270.0;
    public static double GATE_OPEN = 20.0 / 270.0;

    // Umbrales provisionales: requieren calibracion.
    public static int COLOR_V2_THRESHOLD = 300;
    public static int COLOR_V3_THRESHOLD = 300;

    public static int MAX_BALLS = 4;

    public static double HOOD_POWER = 0.45;
    public static double HOOD_MOVE_SECONDS = 0.30;

    // =========================================
    // ESTADOS
    // =========================================

    private boolean shooterOn = false;
    private boolean trackingOn = false;
    private boolean homeOn = false;

    private boolean autoIntake = false;
    private boolean manualIntake = false;
    private boolean manualIndexer = false;

    private boolean hoodOpen = false;
    private boolean hoodMoving = false;
    private long hoodStopNs = 0;

    private boolean hoodAvailable = false;

    private int ballCount = 0;

    private boolean v2Detected = false;
    private boolean v3Detected = false;

    private boolean lastV2 = false;
    private boolean lastV3 = false;

    private boolean feeding = false;
    private boolean reversing = false;

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;

    private boolean lastL1 = false;
    private boolean lastR1 = false;

    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastLeft = false;
    private boolean lastRight = false;

    private boolean lastL3 = false;
    private boolean lastR3 = false;
    private boolean lastStart = false;

    private double shooterVelocity = SHOOTER_START_RPM;
    private long lastSpeedNs = 0;

    // =========================================
    // INIT
    // =========================================

    @Override
    public void init() {

        drive = new DriveSubsystem(hardwareMap);

        // Cambiar a la pose real de inicio.
        drive.setInitialPose(72, 72, 0);

        driveCmd = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );

        turret = new TurretSubsystemV4(hardwareMap);

        // Mantener tu configuracion actual.
        // Probar con potencia reducida primero.
        turret.maxPower = 0.85;

        vision = new TurretVisionV5(hardwareMap);

        aim = new TurretAimControllerV6(
                drive,
                turret,
                vision
        );

        zones = new ZoneTargetSelector();

        // =====================================
        // SHOOTER
        // =====================================

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

        shooter.setVelocityPIDFCoefficients(
                8.0,
                0.0,
                0.5,
                18.0
        );

        shooter.setPower(0);

        // =====================================
        // INTAKE
        // =====================================

        intake = hardwareMap.get(
                DcMotorEx.class,
                "intake2"
        );

        intake.setDirection(
                DcMotor.Direction.FORWARD
        );

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        intake.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        intake.setPower(0);

        // =====================================
        // INDEXER
        // =====================================

        indexer = hardwareMap.get(
                DcMotorEx.class,
                "intake1"
        );

        indexer.setDirection(
                DcMotor.Direction.REVERSE
        );

        indexer.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        indexer.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        indexer.setPower(0);

        // =====================================
        // SENSORES
        // =====================================

        colorV2 = hardwareMap.get(
                ColorSensor.class,
                "colorV2"
        );

        colorV3 = hardwareMap.get(
                ColorSensor.class,
                "colorV3"
        );

        // =====================================
        // GATE
        // =====================================

        gate = hardwareMap.get(
                Servo.class,
                "gate"
        );

        gate.setDirection(
                Servo.Direction.REVERSE
        );

        gate.setPosition(GATE_CLOSED);

        // =====================================
        // HOOD (OPCIONAL)
        // =====================================

        try {

            hood1 = hardwareMap.get(
                    CRServo.class,
                    HOOD_1_NAME
            );

            hood2 = hardwareMap.get(
                    CRServo.class,
                    HOOD_2_NAME
            );

            hoodAvailable = true;

            hood1.setPower(0);
            hood2.setPower(0);

        } catch (IllegalArgumentException e) {

            hoodAvailable = false;
            hood1 = null;
            hood2 = null;
        }

        shooterVelocity = SHOOTER_START_RPM;

        telemetry.addLine("BIOBUZZ FULL TELEOP READY");
        telemetry.addLine("1 GAMEPAD");
        telemetry.addLine("PEDRO + LIMELIGHT + INTAKE");
        telemetry.addData("Hood Available", hoodAvailable);
        telemetry.addLine("CHECK GATE AND HOOD DIRECTIONS");
        telemetry.update();
    }

    // =========================================
    // START
    // =========================================

    @Override
    public void start() {

        vision.start();

        lastV2 = false;
        lastV3 = false;

        gate.setPosition(GATE_CLOSED);
    }

    // =========================================
    // LOOP
    // =========================================

    @Override
    public void loop() {

        long now = System.nanoTime();

        // =====================================
        // DRIVE
        // =====================================

        driveCmd.execute();
        drive.update();

        // =====================================
        // ZONES
        // =====================================

        zones.update(
                drive.getX(),
                drive.getY()
        );

        // D-PAD DOWN: cambiar alianza
        if (gamepad1.dpad_down && !lastDown) {

            if (zones.getAlliance() ==
                    ZoneTargetSelector.Alliance.RED) {

                zones.setAlliance(
                        ZoneTargetSelector.Alliance.BLUE
                );

            } else {

                zones.setAlliance(
                        ZoneTargetSelector.Alliance.RED
                );
            }

            zones.setAutomatic(true);
        }

        // D-PAD UP: automatico
        if (gamepad1.dpad_up && !lastUp) {
            zones.setAutomatic(true);
        }

        // D-PAD LEFT
        if (gamepad1.dpad_left && !lastLeft) {
            zones.selectLeft();
        }

        // D-PAD RIGHT
        if (gamepad1.dpad_right && !lastRight) {
            zones.selectRight();
        }

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;

        int tagId = zones.getTagId();

        vision.setSelectedTag(tagId);
        vision.update();

        aim.setSelectedTag(tagId);

        // =====================================
        // SHOOTER - A
        // =====================================

        if (gamepad1.a && !lastA) {
            shooterOn = !shooterOn;
        }

        lastA = gamepad1.a;

        // R2 / L2: velocidad
        if (now - lastSpeedNs > 200_000_000L) {

            if (gamepad1.right_trigger > 0.5) {

                shooterVelocity += SHOOTER_STEP;
                lastSpeedNs = now;

            } else if (gamepad1.left_trigger > 0.5) {

                shooterVelocity -= SHOOTER_STEP;
                lastSpeedNs = now;
            }
        }

        shooterVelocity = Range.clip(
                shooterVelocity,
                0,
                SHOOTER_MAX
        );

        if (shooterOn) {
            shooter.setVelocity(shooterVelocity);
        } else {
            shooter.setPower(0);
        }

        // =====================================
        // TRACKING - B
        // =====================================

        if (gamepad1.b && !lastB) {

            if (trackingOn) {

                aim.disable();
                trackingOn = false;

            } else {

                homeOn = false;

                aim.enable();
                trackingOn = aim.isEnabled();
            }
        }

        lastB = gamepad1.b;

        // =====================================
        // HOME - L1
        // =====================================

        if (gamepad1.left_bumper && !lastL1) {

            aim.disable();
            trackingOn = false;

            turret.enableHold();
            turret.setTarget(0);

            homeOn = turret.isEnabled();
        }

        lastL1 = gamepad1.left_bumper;

        // =====================================
        // RESET FAULT - R1
        // =====================================

        if (gamepad1.right_bumper && !lastR1) {

            aim.disable();

            trackingOn = false;
            homeOn = false;

            turret.clearFault();
        }

        lastR1 = gamepad1.right_bumper;

        // =====================================
        // AUTO INTAKE - Y
        // =====================================

        if (gamepad1.y && !lastY) {

            autoIntake = !autoIntake;

            if (autoIntake) {
                manualIntake = false;
                manualIndexer = false;
            }
        }

        lastY = gamepad1.y;

        // =====================================
        // MANUAL INTAKE - L3
        // =====================================

        if (gamepad1.left_stick_button && !lastL3) {

            manualIntake = !manualIntake;

            if (manualIntake) {
                autoIntake = false;
            }
        }

        lastL3 = gamepad1.left_stick_button;

        // =====================================
        // MANUAL INDEXER - R3
        // =====================================

        if (gamepad1.right_stick_button && !lastR3) {

            manualIndexer = !manualIndexer;

            if (manualIndexer) {
                autoIntake = false;
            }
        }

        lastR3 = gamepad1.right_stick_button;

        // =====================================
        // SENSORES DE COLOR
        // =====================================

        int v2Alpha = colorV2.alpha();
        int v3Alpha = colorV3.alpha();

        v2Detected =
                v2Alpha >= COLOR_V2_THRESHOLD;

        v3Detected =
                v3Alpha >= COLOR_V3_THRESHOLD;

        // =====================================
        // FEED / REVERSE
        // =====================================

        // X: mantener para alimentar disparo.
        feeding = gamepad1.x;

        // BACK/SHARE: reversa mientras
        // este presionado.
        reversing = gamepad1.back;

        // =====================================
        // BALL COUNT
        // =====================================

        // Contar solo si auto intake activo,
        // no se esta disparando ni revirtiendo.
        if (autoIntake && !feeding && !reversing) {

            if (v2Detected && !lastV2) {

                ballCount = Math.min(
                        MAX_BALLS,
                        ballCount + 1
                );
            }
        }

        // Descontar cuando la pelota deja
        // el sensor superior durante disparo.
        if (feeding && lastV3 && !v3Detected) {

            ballCount = Math.max(
                    0,
                    ballCount - 1
            );
        }

        lastV2 = v2Detected;
        lastV3 = v3Detected;

        // =====================================
        // INTAKE / INDEXER / GATE
        // =====================================

        double intakePower = 0;
        double indexerPower = 0;

        if (reversing) {

            // Reversa tiene prioridad.
            intakePower = -INTAKE_POWER;
            indexerPower = -INDEXER_POWER;

            gate.setPosition(GATE_CLOSED);

        } else if (feeding) {

            // Disparo manual mientras X presionado.
            intakePower = INTAKE_POWER;
            indexerPower = INDEXER_POWER;

            gate.setPosition(GATE_OPEN);

        } else if (autoIntake) {

            gate.setPosition(GATE_CLOSED);

            // Intake se detiene al llegar a 4.
            if (ballCount < MAX_BALLS) {
                intakePower = INTAKE_POWER;
            }

            // Indexer solo avanza cuando
            // el sensor superior esta libre.
            if (!v3Detected && ballCount > 0) {
                indexerPower = INDEXER_POWER;
            }

        } else {

            gate.setPosition(GATE_CLOSED);

            if (manualIntake) {
                intakePower = INTAKE_POWER;
            }

            if (manualIndexer) {
                indexerPower = INDEXER_POWER;
            }
        }

        intake.setPower(
                Range.clip(intakePower, -1, 1)
        );

        indexer.setPower(
                Range.clip(indexerPower, -1, 1)
        );

        // =====================================
        // HOOD - START/OPTIONS
        // =====================================

        if (gamepad1.start && !lastStart) {

            hoodOpen = !hoodOpen;

            if (hoodAvailable) {

                double direction =
                        hoodOpen ? 1.0 : -1.0;

                hood1.setPower(
                        HOOD_POWER * direction
                );

                hood2.setPower(
                        -HOOD_POWER * direction
                );

                hoodMoving = true;

                hoodStopNs = now +
                        (long)(
                                HOOD_MOVE_SECONDS * 1e9
                        );
            }
        }

        lastStart = gamepad1.start;

        if (hoodMoving && now >= hoodStopNs) {

            if (hoodAvailable) {

                hood1.setPower(0);
                hood2.setPower(0);
            }

            hoodMoving = false;
        }

        // =====================================
        // TURRET UPDATE
        // =====================================

        if (trackingOn) {

            aim.update();

            if (turret.isFault() ||
                    !aim.isEnabled()) {

                trackingOn = false;
            }

        } else if (homeOn) {

            turret.update();

            if (turret.isFault() ||
                    Math.abs(turret.getAngle()) <= 1.0) {

                turret.disable();
                homeOn = false;
            }

        } else {

            turret.update();
        }

        // =====================================
        // TELEMETRY
        // =====================================

        telemetry.addLine("===== BIOBUZZ FULL V6 =====");

        telemetry.addData(
                "Alliance",
                zones.getAlliance()
        );

        telemetry.addData(
                "Zone",
                zones.getZone()
        );

        telemetry.addData(
                "Tag",
                tagId
        );

        telemetry.addLine("===== DRIVE =====");

        telemetry.addData(
                "X",
                "%.2f",
                drive.getX()
        );

        telemetry.addData(
                "Y",
                "%.2f",
                drive.getY()
        );

        telemetry.addData(
                "Heading",
                "%.2f",
                drive.getHeadingDegrees()
        );

        telemetry.addLine("===== TURRET =====");

        telemetry.addData(
                "Tracking",
                trackingOn
        );

        telemetry.addData(
                "Aim",
                aim.getStatus()
        );

        telemetry.addData(
                "Actual",
                "%.2f",
                turret.getAngle()
        );

        telemetry.addData(
                "Target",
                "%.2f",
                turret.getTarget()
        );

        telemetry.addData(
                "Error",
                "%.2f",
                turret.getError()
        );

        telemetry.addData(
                "Power",
                "%.2f",
                turret.getPower()
        );

        telemetry.addData(
                "Max Power",
                "%.2f",
                turret.maxPower
        );

        telemetry.addData(
                "Turret Velocity",
                "%.1f",
                turret.getVelocity()
        );

        telemetry.addData(
                "Chassis Angular Velocity",
                "%.1f",
                aim.getChassisAngularVelocity()
        );

        telemetry.addData(
                "Fault",
                turret.getFaultReason()
        );

        telemetry.addLine("===== LIMELIGHT =====");

//        telemetry.addData(
//                "Vision",
//                vision.getStatus()
//        );

        telemetry.addData(
                "Tag Visible",
                vision.hasTarget()
        );

//        telemetry.addData(
//                "Detected Tag",
//                vision.getDetectedTag()
//        );

        telemetry.addData(
                "Tx",
                "%.2f",
                vision.getTx()
        );

        telemetry.addLine("===== SHOOTER =====");

        telemetry.addData(
                "Shooter ON",
                shooterOn
        );

        telemetry.addData(
                "Target Velocity",
                "%.0f",
                shooterVelocity
        );

        telemetry.addData(
                "Actual Velocity",
                "%.0f",
                shooter.getVelocity()
        );

        telemetry.addLine("===== INTAKE =====");

        telemetry.addData(
                "Auto Intake",
                autoIntake
        );

        telemetry.addData(
                "Manual Intake",
                manualIntake
        );

        telemetry.addData(
                "Manual Indexer",
                manualIndexer
        );

        telemetry.addData(
                "Feeding",
                feeding
        );

        telemetry.addData(
                "Reversing",
                reversing
        );

        telemetry.addData(
                "Ball Count",
                ballCount
        );

        telemetry.addData(
                "V2 Alpha",
                v2Alpha
        );

        telemetry.addData(
                "V3 Alpha",
                v3Alpha
        );

        telemetry.addData(
                "V2 Detected",
                v2Detected
        );

        telemetry.addData(
                "V3 Detected",
                v3Detected
        );

        telemetry.addData(
                "Intake Power",
                "%.2f",
                intake.getPower()
        );

        telemetry.addData(
                "Indexer Power",
                "%.2f",
                indexer.getPower()
        );

        telemetry.addLine("===== GATE / HOOD =====");

        telemetry.addData(
                "Gate",
                feeding && !reversing
                        ? "OPEN" : "CLOSED"
        );

        telemetry.addData(
                "Hood Open",
                hoodOpen
        );

        telemetry.addData(
                "Hood Available",
                hoodAvailable
        );

        telemetry.addData(
                "Hood Moving",
                hoodMoving
        );

        telemetry.update();
    }

    // =========================================
    // STOP
    // =========================================

    @Override
    public void stop() {

        if (aim != null) {
            aim.disable();
        }

        if (turret != null) {
            turret.stop();
        }

        if (vision != null) {
            vision.stop();
        }

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
            gate.setPosition(GATE_CLOSED);
        }

        if (hood1 != null) {
            hood1.setPower(0);
        }

        if (hood2 != null) {
            hood2.setPower(0);
        }

        if (drive != null) {
            drive.stop();
        }
    }
}

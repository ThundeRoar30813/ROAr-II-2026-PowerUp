package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;
import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV5;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV77;

@TeleOp(name = "DIAGNOSTICO TURRET V7.7")
public class BioBuzzTurretDiagnostic1 extends OpMode {

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCmd;

    private TurretSubsystemV4 turret;
    private TurretVisionV5 vision;
    private TurretAimControllerV77 aim;

    private DcMotorEx shooter;

    // División del campo en pulgadas (144" ancho total -> mitad = 72.0")
    public static double FIELD_MID_X = 72.0;
    public static double SPLIT_HYSTERESIS_IN = 1.5; // Evita saltos justo en la línea 72.0

    public static int LEFT_SIDE_TAG_ID = 37;
    public static int RIGHT_SIDE_TAG_ID = 35;
    public static double TEST_POWER = 1.0;

    private int activeTagId = LEFT_SIDE_TAG_ID;
    private boolean autoZoneSelection = true;

    private boolean tracking = false;
    private boolean home = false;

    private boolean lastB = false;
    private boolean lastL1 = false;
    private boolean lastR1 = false;
    private boolean lastLeft = false;
    private boolean lastRight = false;
    private boolean lastUp = false;
    private boolean lastDown = false;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap);
        // Pose inicial en lado izquierdo de Alianza Roja (X=48 < 72 -> Tag 37)
        drive.setInitialPose(48, 6, 90);

        driveCmd = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );

        turret = new TurretSubsystemV4(hardwareMap);
        turret.maxPower = TEST_POWER;

        vision = new TurretVisionV5(hardwareMap);

        aim = new TurretAimControllerV77(
                drive,
                turret,
                vision
        );
        aim.setMode(TurretAimControllerV77.Mode.COMBINED);

        shooter = hardwareMap.get(DcMotorEx.class, "shoot");
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("V7.7 FAST FUSION - RED ALLIANCE SPLIT");
        telemetry.addLine("GATILLO DER (RT): ACELERAR SHOOTER");
        telemetry.addLine("IZQUIERDA: SOLO LIMELIGHT");
        telemetry.addLine("DERECHA: SOLO PEDRO");
        telemetry.addLine("ARRIBA: COMBINED (PEDRO + LIMELIGHT)");
        telemetry.addLine("ABAJO: AUTO-TAG (IZQ=37 / DER=35) ON/OFF");
        telemetry.addLine("B: TRACKING ON/OFF | L1: HOME | R1: RESET FAULT");
        telemetry.update();
    }

    @Override
    public void start() {
        vision.start();
    }

    @Override
    public void loop() {
        // ==================================
        // DRIVE
        // ==================================
        driveCmd.execute();
        drive.update();

        // ==================================
        // SHOOTER (GATILLO DERECHO)
        // ==================================
        double shooterPower = gamepad1.right_trigger;
        shooter.setPower(shooterPower);

        // ==================================
        // SELECCIÓN AUTOMÁTICA DE TAG POR MITAD DE CAMPO (ALIANZA ROJA)
        // ==================================
        if (gamepad1.dpad_down && !lastDown) {
            autoZoneSelection = !autoZoneSelection;
        }
        lastDown = gamepad1.dpad_down;

        double robotX = drive.getX();
        if (autoZoneSelection && Double.isFinite(robotX)) {
            if (robotX < (FIELD_MID_X - SPLIT_HYSTERESIS_IN)) {
                activeTagId = LEFT_SIDE_TAG_ID;  // Lado Izquierdo -> Tag 37
            } else if (robotX > (FIELD_MID_X + SPLIT_HYSTERESIS_IN)) {
                activeTagId = RIGHT_SIDE_TAG_ID; // Lado Derecho -> Tag 35
            }
        }

        // ==================================
        // LIMELIGHT
        // ==================================
        vision.setSelectedTag(activeTagId);
        vision.update();

        // ==================================
        // SELECCIÓN DE MODO DE APUNTADO
        // ==================================
        if (gamepad1.dpad_left && !lastLeft) {
            aim.setMode(TurretAimControllerV77.Mode.VISION_ONLY);
        }
        if (gamepad1.dpad_right && !lastRight) {
            aim.setMode(TurretAimControllerV77.Mode.PEDRO_ONLY);
        }
        if (gamepad1.dpad_up && !lastUp) {
            aim.setMode(TurretAimControllerV77.Mode.COMBINED);
        }

        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        lastUp = gamepad1.dpad_up;

        // ==================================
        // TRACKING ON/OFF
        // ==================================
        if (gamepad1.b && !lastB) {
            if (tracking) {
                aim.disable();
                tracking = false;
            } else {
                home = false;
                aim.enable();
                tracking = aim.isEnabled();
            }
        }
        lastB = gamepad1.b;

        // ==================================
        // HOME
        // ==================================
        if (gamepad1.left_bumper && !lastL1) {
            aim.disable();
            tracking = false;
            turret.enableHold();
            turret.setTarget(0);
            home = turret.isEnabled();
        }
        lastL1 = gamepad1.left_bumper;

        // ==================================
        // RESET FAULT
        // ==================================
        if (gamepad1.right_bumper && !lastR1) {
            aim.disable();
            tracking = false;
            home = false;
            turret.clearFault();
        }
        lastR1 = gamepad1.right_bumper;

        // ==================================
        // UPDATE TURRET
        // ==================================
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
        // TELEMETRÍA
        // ==================================
        String fieldSide = (robotX < FIELD_MID_X) ? "IZQUIERDA (X < 72)" : "DERECHA (X >= 72)";

        telemetry.addLine("===== V7.7 RED ALLIANCE =====");
        telemetry.addData("Zona Campo", "%s | AutoTag: %s", fieldSide, autoZoneSelection ? "ON" : "OFF");
        telemetry.addData("Target Tag ID", activeTagId);
        telemetry.addData("Mode", aim.getMode());
        telemetry.addData("Status", aim.getStatus());
        telemetry.addData("Tracking", tracking);
        telemetry.addData("Ready To Shoot", turret.isReadyToShoot(2.5) ? "SI" : "NO");
        telemetry.addData("Shooter Power", "%.2f", shooterPower);

        telemetry.addLine("===== TURRET =====");
        telemetry.addData("Actual Angle", "%.2f", turret.getAngle());
        telemetry.addData("Base Target", "%.2f", aim.getBaseTarget());
        telemetry.addData("Error", "%.2f", turret.getError());
        telemetry.addData("At Cable Limit", turret.isAtLimit());
        telemetry.addData("Power", "%.3f", turret.getPower());
        telemetry.addData("Fault", turret.getFaultReason());

        telemetry.addLine("===== PEDRO & VISION =====");
        telemetry.addData("Robot X / Y", "%.1f, %.1f", drive.getX(), drive.getY());
        telemetry.addData("Robot Heading", "%.2f", drive.getHeadingDegrees());
        telemetry.addData("Vision Offset", "%.2f", aim.getVisionOffset());

        telemetry.addLine("===== LIMELIGHT =====");
        telemetry.addData("Visible", vision.hasTarget());
        telemetry.addData("Raw Tx", "%.2f", vision.getTx());
        telemetry.addData("Age (ms)", "%.0f", vision.getAgeMs());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (aim != null) aim.disable();
        if (turret != null) turret.stop();
        if (vision != null) vision.stop();
        if (drive != null) drive.stop();
        if (shooter != null) shooter.setPower(0);
    }
}
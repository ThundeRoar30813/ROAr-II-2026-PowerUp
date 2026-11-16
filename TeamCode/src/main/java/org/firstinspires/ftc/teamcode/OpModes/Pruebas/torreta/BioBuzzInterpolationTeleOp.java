package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterInterpolation;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV5;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV77;

@TeleOp(name = "BIOBUZZ INTERPOLACION V6")
public class BioBuzzInterpolationTeleOp extends OpMode {

    // ==========================================
    // CONFIGURACION
    // ==========================================

    public static double START_X = 48.0;
    public static double START_Y = 6.0;
    public static double START_HEADING = 90.0;

    public static double FIELD_SIZE = 144.0;
    public static double FIELD_MID_Y = 72.0;
    public static double ZONE_HYSTERESIS = 1.5;

    public static int INITIAL_TAG = 37;
    public static int OPPOSITE_TAG = 30;

    public static double IDLE_TPS = 600.0;
    public static double RT_THRESHOLD = 0.20;

    public static double HOOD_HYSTERESIS = 0.15;

    // Provisional hasta calibrar el otro lado.
    public static boolean MIRROR_UPPER_TABLE = true;

    public static double MAX_TABLE_DISTANCE_IN = 24.0;

    // ==========================================
    // SUBSYSTEMS
    // ==========================================

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCmd;

    private TurretSubsystemV4 turret;
    private TurretVisionV5 vision;
    private TurretAimControllerV77 aim;

    private IntakeSubsystem intake;
    private IntakeCommand intakeCmd;

    private ShooterSubsystem shooter;
    private HoodSubsystem hood;

    // ==========================================
    // ESTADOS
    // ==========================================

    private boolean oppositeSide = false;
    private boolean tracking = false;

    private boolean previousB = false;
    private boolean previousStart = false;
    private boolean previousBack = false;

    private boolean previousUp = false;
    private boolean previousDown = false;

    private int selectedTag = INITIAL_TAG;
    private int hoodLevel = 3;

    private double globalTPSOffset = 0.0;
    private double requestedTPS = IDLE_TPS;

    private double tableX = Double.NaN;
    private double tableY = Double.NaN;

    private boolean validPose = false;
    private boolean validTable = false;

    private ShooterInterpolation.Result interpolation;

    // ==========================================
    // INIT
    // ==========================================

    @Override
    public void init() {

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

        turret = new TurretSubsystemV4(hardwareMap);
        vision = new TurretVisionV5(hardwareMap);

        aim = new TurretAimControllerV77(
                drive,
                turret,
                vision
        );

        aim.setMode(
                TurretAimControllerV77.Mode.COMBINED
        );

        // Siempre iniciar con Tag 37.
        oppositeSide = false;
        selectedTag = INITIAL_TAG;

        vision.setSelectedTag(selectedTag);

        intake = new IntakeSubsystem(hardwareMap);
        intakeCmd = new IntakeCommand(intake);

        shooter = new ShooterSubsystem(hardwareMap);
        shooter.setTargetTPS(IDLE_TPS);
        shooter.setEnabled(false);

        hood = new HoodSubsystem(hardwareMap);

        hoodLevel = 3;
        hood.setIndex(hoodLevel - 1);

        telemetry.addLine("BIOBUZZ V6 LISTO");
        telemetry.addLine("INICIO: TAG 37");
        telemetry.addLine("OTRO LADO: TAG 30");
        telemetry.addLine("TRACKING ORIGINAL COMBINED");
        telemetry.update();
    }

    // ==========================================
    // START
    // ==========================================

    @Override
    public void start() {

        oppositeSide = false;
        selectedTag = INITIAL_TAG;

        vision.setSelectedTag(selectedTag);
        vision.start();

        shooter.setTargetTPS(IDLE_TPS);
        shooter.setEnabled(true);

        tracking = false;
    }

    // ==========================================
    // SELECCION AUTOMATICA 37 / 30
    // ==========================================

    private void updateZoneAndTag() {

        double y = drive.getY();

        if (!Double.isFinite(y)) {
            return;
        }

        if (oppositeSide) {

            // Regreso al lado inicial.
            if (y < FIELD_MID_Y - ZONE_HYSTERESIS) {
                oppositeSide = false;
            }

        } else {

            // Cruce al lado contrario.
            if (y > FIELD_MID_Y + ZONE_HYSTERESIS) {
                oppositeSide = true;
            }
        }

        selectedTag = oppositeSide
                ? OPPOSITE_TAG
                : INITIAL_TAG;

        vision.setSelectedTag(selectedTag);
    }

    // ==========================================
    // AJUSTE GLOBAL TPS
    // ==========================================

    private void updateTPSAdjustment() {

        boolean up = gamepad2.dpad_up;
        boolean down = gamepad2.dpad_down;

        if (up && !previousUp) {
            globalTPSOffset += 5.0;
        }

        if (down && !previousDown) {
            globalTPSOffset -= 2.0;
        }

        previousUp = up;
        previousDown = down;
    }

    // ==========================================
    // INTERPOLACION
    // ==========================================

    private void updateInterpolation() {

        tableX = drive.getX();
        tableY = drive.getY();

        validPose =
                Double.isFinite(tableX)
                        && Double.isFinite(tableY);

        validTable = false;
        interpolation = null;

        if (!validPose) {
            return;
        }

        if (oppositeSide && MIRROR_UPPER_TABLE) {
            tableY = FIELD_SIZE - tableY;
        }

        interpolation = ShooterInterpolation.calculate(
                tableX,
                tableY
        );

        if (interpolation == null) {
            return;
        }

        validTable =
                Double.isFinite(interpolation.tps)
                        && Double.isFinite(
                        interpolation.nearestDistance
                )
                        && interpolation.nearestDistance
                        <= MAX_TABLE_DISTANCE_IN;

        if (validTable) {
            updateHood(
                    interpolation.hoodInterpolated
            );
        }
    }

    // ==========================================
    // HOOD
    // ==========================================

    private void updateHood(double level) {

        if (!Double.isFinite(level)) {
            return;
        }

        int nextLevel = hoodLevel;

        while (nextLevel < 5
                && level > nextLevel + 0.5
                + HOOD_HYSTERESIS) {

            nextLevel++;
        }

        while (nextLevel > 1
                && level < nextLevel - 0.5
                - HOOD_HYSTERESIS) {

            nextLevel--;
        }

        if (nextLevel != hoodLevel) {

            hoodLevel = nextLevel;
            hood.setIndex(hoodLevel - 1);
        }
    }

    // ==========================================
    // CONTROLES TURRET
    // ==========================================

    private void updateTurretControls() {

        boolean b = gamepad2.b;
        boolean home = gamepad2.start;
        boolean reset = gamepad2.back;

        // RESET FAULT
        if (reset && !previousBack) {
            turret.clearFault();
        }

        // HOME
        if (home && !previousStart) {

            if (aim.isEnabled()) {
                aim.disable();
            }

            tracking = false;

            turret.enableHold();
            turret.setTarget(0.0);
        }

        // TRACKING ON/OFF
        if (b && !previousB) {

            if (tracking) {

                aim.disable();
                tracking = false;

                turret.enableHold();
                turret.setTarget(
                        turret.getAngle()
                );

            } else {

                aim.enable();
                tracking = aim.isEnabled();
            }
        }

        previousB = b;
        previousStart = home;
        previousBack = reset;

        if (tracking && !aim.isEnabled()) {
            tracking = false;
        }

        if (tracking) {

            aim.update();

        } else {

            turret.setVelocityFeedforward(0);
            turret.update();
        }
    }

    // ==========================================
    // LOOP
    // ==========================================

    @Override
    public void loop() {

        // 1. CHASIS Y PEDRO
        driveCmd.execute();
        drive.update();

        // 2. AJUSTE TPS
        updateTPSAdjustment();

        // 3. SELECCION 37 / 30
        updateZoneAndTag();

        // 4. LIMELIGHT
        vision.update();

        // 5. TURRET
        updateTurretControls();

        // 6. INTAKE
        intakeCmd.execute(
                gamepad2.y,
                gamepad2.right_bumper,
                gamepad2.a,
                gamepad2.left_bumper
        );

        // 7. INTERPOLACION
        updateInterpolation();

        // 8. SHOOTER
        boolean shooting =
                gamepad2.right_trigger > RT_THRESHOLD;

        if (shooting
                && validTable
                && interpolation != null) {

            requestedTPS =
                    interpolation.tps
                            + globalTPSOffset;

        } else {

            requestedTPS = IDLE_TPS;
        }

        requestedTPS = Math.max(
                0.0,
                requestedTPS
        );

        shooter.setTargetTPS(requestedTPS);
        shooter.setEnabled(true);
        shooter.update();

        // ======================================
        // TELEMETRIA
        // ======================================

        telemetry.addLine(
                "===== BIOBUZZ V6 ====="
        );

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

        telemetry.addData(
                "Zona",
                oppositeSide
                        ? "LADO CONTRARIO"
                        : "LADO INICIAL"
        );

        telemetry.addLine(
                "===== APRILTAG ====="
        );

        telemetry.addData(
                "Tag seleccionado",
                selectedTag
        );

        telemetry.addData(
                "Tag detectado",
                vision.getDetectedTag()
        );

        telemetry.addData(
                "Tag visible",
                vision.hasTarget()
        );

        telemetry.addData(
                "Tx",
                "%.2f",
                vision.getTx()
        );

        telemetry.addData(
                "Vision Status",
                vision.getStatus()
        );

        telemetry.addData(
                "Frame Age",
                "%.1f",
                vision.getAgeMs()
        );

        telemetry.addLine(
                "===== TURRET ====="
        );

        telemetry.addData(
                "Tracking",
                tracking
        );

        telemetry.addData(
                "Aim Mode",
                aim.getMode()
        );

        telemetry.addData(
                "Aim Status",
                aim.getStatus()
        );

        telemetry.addData(
                "Turret Actual",
                "%.2f",
                turret.getAngle()
        );

        telemetry.addData(
                "Turret Target",
                "%.2f",
                turret.getTarget()
        );

        telemetry.addData(
                "Turret Error",
                "%.2f",
                turret.getError()
        );

        telemetry.addData(
                "Vision Offset",
                "%.2f",
                aim.getVisionOffset()
        );

        telemetry.addData(
                "Ready",
                turret.isReadyToShoot(2.5)
        );

        telemetry.addData(
                "Fault",
                turret.getFaultReason()
        );

        telemetry.addLine(
                "===== SHOOTER ====="
        );

        telemetry.addData(
                "Shooter Mode",
                shooting ? "SHOOTING" : "IDLE"
        );

        telemetry.addData(
                "TPS objetivo",
                "%.1f",
                shooter.getTargetTPS()
        );

        telemetry.addData(
                "TPS real",
                "%.1f",
                shooter.getActualTPS()
        );

        telemetry.addData(
                "TPS error",
                "%.1f",
                shooter.getErrorTPS()
        );

        telemetry.addData(
                "Offset global",
                "%+.0f",
                globalTPSOffset
        );

        telemetry.addLine(
                "===== INTERPOLACION ====="
        );

        telemetry.addData(
                "Tabla valida",
                validTable
        );

        telemetry.addData(
                "Tabla X",
                "%.2f",
                tableX
        );

        telemetry.addData(
                "Tabla Y",
                "%.2f",
                tableY
        );

        if (interpolation != null) {

            telemetry.addData(
                    "TPS interpoladas",
                    "%.1f",
                    interpolation.tps
            );

            telemetry.addData(
                    "Hood interpolado",
                    "%.2f",
                    interpolation.hoodInterpolated
            );

            telemetry.addData(
                    "Distancia calibracion",
                    "%.2f in",
                    interpolation.nearestDistance
            );
        }

        telemetry.addLine(
                "===== HOOD ====="
        );

        telemetry.addData(
                "Hood Level",
                hoodLevel
        );

        telemetry.addData(
                "Hood Servo",
                "%.4f",
                hood.getPosition()
        );

        telemetry.addLine(
                "===== INTAKE ====="
        );

        telemetry.addData(
                "Auto Intake",
                intake.isAutoIntakeEnabled()
        );

        telemetry.addData(
                "Ball Count",
                intake.getBallCount()
        );

        telemetry.addData(
                "Sensor V2",
                intake.isBottomDetected()
        );

        telemetry.addData(
                "Sensor V3",
                intake.isTopDetected()
        );

        telemetry.update();
    }

    // ==========================================
    // STOP
    // ==========================================

    @Override
    public void stop() {

        if (shooter != null) {
            shooter.stop();
        }

        if (intakeCmd != null) {
            intakeCmd.stop();
        }

        if (aim != null) {
            aim.disable();
        }

        if (vision != null) {
            vision.stop();
        }

        if (turret != null) {
            turret.setVelocityFeedforward(0);
            turret.disable();
        }

        if (driveCmd != null) {
            driveCmd.stop();
        }
    }
}
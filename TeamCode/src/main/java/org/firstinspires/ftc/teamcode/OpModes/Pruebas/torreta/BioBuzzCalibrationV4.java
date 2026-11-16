package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterInterpolation;
import org.firstinspires.ftc.teamcode.Subsystems.HoodSubsystem;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV5;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV77;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretControlCommand;
import org.firstinspires.ftc.teamcode.Commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.shooter.ShooterTuningCommand;
import org.firstinspires.ftc.teamcode.Commands.shooter.HoodCommand;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@TeleOp(name = "BIOBUZZ - CALIBRACION V4")
public class BioBuzzCalibrationV4 extends OpMode {

    // =========================================
    // SUBSYSTEMS Y COMMANDS
    // =========================================

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCmd;

    private TurretSubsystemV4 turret;
    private TurretVisionV5 vision;
    private TurretAimControllerV77 aim;
    private TurretControlCommand turretCmd;

    private IntakeSubsystem intake;
    private IntakeCommand intakeCmd;

    private ShooterSubsystem shooter;
    private ShooterTuningCommand shooterCmd;

    private HoodSubsystem hood;
    private HoodCommand hoodCmd;

    private FtcDashboard dashboard;

    // =========================================
    // POSICION INICIAL
    // =========================================

    public static double START_X = 48.0;
    public static double START_Y = 6.0;
    public static double START_HEADING = 90.0;

    public static double GOAL_X = 62.0;
    public static double GOAL_Y = 57.0;

    private static final double FIELD_HALF = 72.0;

    // =========================================
    // MODO INTERPOLACION AUTOMATICA (PRUEBA)
    // =========================================

    private boolean autoInterpEnabled = false;
    private boolean previousAutoInterpBtn = false;

    // =========================================
    // GUARDAR MUESTRAS
    // =========================================

    private int sampleCount = 0;
    private String lastSaveStatus = "Sin muestras";
    private File csvFile;

    private boolean previousSave = false;

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

        // TURRET
        turret = new TurretSubsystemV4(hardwareMap);
        vision = new TurretVisionV5(hardwareMap);

        aim = new TurretAimControllerV77(
                drive,
                turret,
                vision
        );

        turretCmd = new TurretControlCommand(
                drive,
                turret,
                vision,
                aim
        );

        // INTAKE
        intake = new IntakeSubsystem(hardwareMap);
        intakeCmd = new IntakeCommand(intake);

        // SHOOTER
        shooter = new ShooterSubsystem(hardwareMap);
        shooterCmd = new ShooterTuningCommand(shooter);

        // HOOD
        hood = new HoodSubsystem(hardwareMap);
        hoodCmd = new HoodCommand(hood);

        dashboard = FtcDashboard.getInstance();

        // =====================================
        // ARCHIVO CSV
        // =====================================

        File folder =
                hardwareMap.appContext.getExternalFilesDir(null);

        if (folder != null) {

            csvFile = new File(
                    folder,
                    "BioBuzzCalibrationV4.csv"
            );

            if (!csvFile.exists()) {

                try (FileWriter writer =
                             new FileWriter(csvFile, true)) {

                    writer.write(
                            "sample,x_in,y_in,heading_deg,"
                                    + "goal_x_in,goal_y_in,"
                                    + "distance_cm,target_tps,"
                                    + "actual_tps,hood_position,"
                                    + "tag_id,tx_deg,turret_error_deg\n"
                    );

                } catch (IOException e) {

                    lastSaveStatus =
                            "Error creando CSV: " + e.getMessage();
                }
            }

        } else {

            lastSaveStatus =
                    "Almacenamiento no disponible";
        }

        telemetry.addLine("BIOBUZZ CALIBRACION V4");
        telemetry.addLine("G2 A: Guardar CSV | G2 X: Auto-Interpolacion ON/OFF");
        telemetry.addLine("Verificar posicion inicial de Pedro");
        telemetry.update();
    }

    // =========================================
    // START
    // =========================================

    @Override
    public void start() {
        turretCmd.start();
    }

    // =========================================
    // LOOP
    // =========================================

    @Override
    public void loop() {

        // =====================================
        // DRIVE
        // =====================================

        driveCmd.execute();
        drive.update();

        double x = drive.getX();
        double y = drive.getY();
        double heading = drive.getHeadingDegrees();

        // =====================================
        // TURRET
        // =====================================

        turretCmd.execute(
                gamepad1.b,
                gamepad1.start,
                gamepad1.back
        );

        // =====================================
        // INTAKE
        // =====================================

        intakeCmd.execute(
                gamepad1.y,
                gamepad1.right_bumper,
                gamepad1.a,
                gamepad2.y
        );

        // =====================================
        // TOGGLE MODO INTERPOLACION (G2 X)
        // =====================================

        boolean autoInterpBtn = gamepad2.x;
        if (autoInterpBtn && !previousAutoInterpBtn) {
            autoInterpEnabled = !autoInterpEnabled;
        }
        previousAutoInterpBtn = autoInterpBtn;

        // =====================================
        // SHOOTER Y HOOD (MANUAL VS AUTO-INTERP)
        // =====================================

        ShooterInterpolation.Result interpResult =
                ShooterInterpolation.calculate(x, y);

        if (autoInterpEnabled) {
            // Permite encender/apagar el shooter con G1 X, pero TPS y Hood vienen de (X, Y)
            if (gamepad1.x) {
                // Usa shooterCmd solo para toggle/update sin pisar TPS
                shooterCmd.execute(
                        true, 0, 0,
                        false, false, false, false,
                        false, false
                );
            } else {
                shooterCmd.execute(
                        false, 0, 0,
                        false, false, false, false,
                        false, false
                );
            }

            shooter.setTargetTPS(interpResult.tps);
            shooter.update();
            hood.setIndex(interpResult.hoodLevel - 1);

        } else {
            // Modo calibración manual completo
            shooterCmd.execute(
                    gamepad1.x,

                    gamepad2.right_trigger,
                    gamepad2.left_trigger,

                    gamepad1.dpad_up,
                    gamepad1.dpad_right,
                    gamepad1.dpad_down,
                    gamepad1.dpad_left,

                    gamepad2.right_bumper,
                    gamepad2.left_bumper
            );

            hoodCmd.execute(
                    gamepad2.dpad_up,
                    gamepad2.dpad_down,
                    gamepad2.dpad_left,
                    gamepad2.dpad_right
            );
        }

        // =====================================
        // DISTANCIA CALCULADA POR PEDRO
        // =====================================

        double dx = GOAL_X - x;
        double dy = GOAL_Y - y;

        double distanceCm =
                Math.hypot(dx, dy) * 2.54;

        // =====================================
        // GUARDAR MUESTRA
        // =====================================

        boolean saveButton = gamepad2.a;

        if (saveButton && !previousSave) {

            saveSample(
                    x,
                    y,
                    heading,
                    distanceCm,
                    shooter.getTargetTPS(),
                    shooter.getActualTPS(),
                    hood.getPositionNumber(),
                    turretCmd.getActiveTagId(),
                    vision.getTx(),
                    turret.getError()
            );
        }

        previousSave = saveButton;

        // =====================================
        // DRIVER STATION
        // =====================================

        telemetry.addLine("=== BIOBUZZ CALIBRACION V4 ===");

        telemetry.addData("Modo Shooter",
                autoInterpEnabled ? "AUTO INTERPOLACION (X,Y)" : "MANUAL TUNING");

        telemetry.addData("Pedro X / Y", "%.2f / %.2f", x, y);
        telemetry.addData("Heading", "%.2f", heading);

        telemetry.addData("Distancia Goal", "%.1f cm",
                distanceCm);

        telemetry.addData("Interp Sugerida",
                "TPS: %.0f | Hood: %d (Dist vecino: %.1f in)",
                interpResult.tps,
                interpResult.hoodLevel,
                interpResult.nearestDistance);

        telemetry.addLine("===== TURRET =====");

        telemetry.addData("Zona",
                turretCmd.getFieldSide());

        telemetry.addData("AprilTag",
                turretCmd.getActiveTagId());

        telemetry.addData("Tracking",
                turretCmd.isTracking());

        telemetry.addData("Modo",
                aim.getMode());

        telemetry.addData("Aim Status",
                aim.getStatus());

        telemetry.addData("Turret Angle",
                "%.2f", turret.getAngle());

        telemetry.addData("Turret Target",
                "%.2f", turret.getTarget());

        telemetry.addData("Turret Error",
                "%.2f", turret.getError());

        telemetry.addData("Vision Offset",
                "%.2f deg", aim.getVisionOffset());

        telemetry.addData("Turret Ready",
                turret.isReadyToShoot(2.5));

        telemetry.addData("Turret Fault",
                turret.getFaultReason());

        telemetry.addLine("===== LIMELIGHT =====");

        telemetry.addData("Tag Visible",
                vision.hasTarget());

        telemetry.addData("Tx",
                "%.2f", vision.getTx());

        telemetry.addData("Age",
                "%.0f ms", vision.getAgeMs());

        double distanceM =
                vision.getDistanceToTagMeters();

        if (vision.hasTarget() &&
                Double.isFinite(distanceM)) {

            telemetry.addData(
                    "Distancia Limelight",
                    "%.1f cm",
                    distanceM * 100.0
            );
        }

        telemetry.addLine("===== SHOOTER =====");

        telemetry.addData("Shooter",
                shooter.isEnabled() ? "ON" : "OFF");

        telemetry.addData("TPS Target",
                "%.1f", shooter.getTargetTPS());

        telemetry.addData("TPS Actual",
                "%.1f", shooter.getActualTPS());

        telemetry.addData("TPS Error",
                "%.1f", shooter.getErrorTPS());

        telemetry.addData("Shooter Power",
                "%.3f", shooter.getPower());

        telemetry.addData("PIDF Parametro",
                shooterCmd.getSelectedParameter());

        telemetry.addData("P", "%.7f", shooter.kP);
        telemetry.addData("I", "%.7f", shooter.kI);
        telemetry.addData("D", "%.7f", shooter.kD);
        telemetry.addData("F", "%.7f", shooter.kF);

        telemetry.addLine("===== INTAKE =====");

        telemetry.addData("Auto Intake",
                intake.isAutoIntakeEnabled());

        telemetry.addData("Ball Count",
                intake.getBallCount());

        telemetry.addData("Sensor V2",
                intake.isBottomDetected());

        telemetry.addData("Sensor V3",
                intake.isTopDetected());

        telemetry.addLine("===== HOOD =====");

        telemetry.addData("Hood",
                "%d / 5", hood.getPositionNumber());

        telemetry.addData("Hood Servo",
                "%.4f", hood.getPosition());

        telemetry.addLine("===== CALIBRACION =====");

        telemetry.addData("Muestras guardadas",
                sampleCount);

        telemetry.addData("Estado CSV",
                lastSaveStatus);

        telemetry.update();

        // =====================================
        // FTC DASHBOARD
        // =====================================

        TelemetryPacket packet =
                new TelemetryPacket(false);

        drawBioBuzzField(packet);

        packet.put("Modo Interp", autoInterpEnabled);
        packet.put("Pedro X", x);
        packet.put("Pedro Y", y);
        packet.put("Heading", heading);

        packet.put("Distancia calculada cm",
                distanceCm);

        packet.put("AprilTag",
                turretCmd.getActiveTagId());

        packet.put("Tracking",
                turretCmd.isTracking());

        packet.put("Turret Error",
                turret.getError());

        packet.put("Vision Offset",
                aim.getVisionOffset());

        packet.put("Limelight Tx",
                vision.getTx());

        packet.put("Shooter Target TPS",
                shooter.getTargetTPS());

        packet.put("Shooter Actual TPS",
                shooter.getActualTPS());

        packet.put("Hood Position",
                hood.getPositionNumber());

        packet.put("Ball Count",
                intake.getBallCount());

        packet.put("Muestras",
                sampleCount);

        packet.put("Ultimo guardado",
                lastSaveStatus);

        // =====================================
        // DIBUJAR ROBOT Y OBJETIVO
        // =====================================

        double dashX = x - FIELD_HALF;
        double dashY = y - FIELD_HALF;

        double goalDashX = GOAL_X - FIELD_HALF;
        double goalDashY = GOAL_Y - FIELD_HALF;

        double headingRad =
                Math.toRadians(heading);

        Canvas c = packet.fieldOverlay();

        // LINEA AL OBJETIVO
        c.setStroke("#EAB308")
                .setStrokeWidth(2)
                .strokeLine(
                        dashX,
                        dashY,
                        goalDashX,
                        goalDashY
                );

        // OBJETIVO
        c.setFill("#16A34A")
                .fillCircle(
                        goalDashX,
                        goalDashY,
                        4
                );

        // ROBOT
        c.setStroke("#2563EB")
                .setStrokeWidth(3)
                .strokeCircle(
                        dashX,
                        dashY,
                        9
                );

        // HEADING
        c.setStroke("#2563EB")
                .setStrokeWidth(3)
                .strokeLine(
                        dashX,
                        dashY,
                        dashX + 14 * Math.cos(headingRad),
                        dashY + 14 * Math.sin(headingRad)
                );

        dashboard.sendTelemetryPacket(packet);
    }

    // =========================================
    // CAMPO BIOBUZZ
    // =========================================

    private void drawBioBuzzField(TelemetryPacket packet) {

        Canvas c = packet.fieldOverlay();

        c.setFill("#292929")
                .fillRect(-72, -72, 144, 144);

        c.setStroke("#555555")
                .setStrokeWidth(1);

        for (int i = -72; i <= 72; i += 24) {

            c.strokeLine(i, -72, i, 72);
            c.strokeLine(-72, i, 72, i);
        }

        c.setStroke("#B0B0B0")
                .setStrokeWidth(1)
                .strokeRect(-72, -72, 144, 144);

        // ESTACION ROJA
        c.setStroke("#EF3340")
                .setStrokeWidth(2)
                .strokeRect(-72, 24, 10, 24);

        // ESTACION AZUL
        c.setStroke("#315BEA")
                .setStrokeWidth(2)
                .strokeRect(62, -48, 10, 24);

        // ESTRUCTURA CENTRAL
        c.setFill("#474747")
                .fillRect(-25, -22, 50, 44);

        c.setStroke("#171717")
                .setStrokeWidth(2)
                .strokeRect(-25, -22, 50, 44);

        // PUENTE
        c.setFill("#F5C76E")
                .fillRect(-12, -4, 24, 8);

        // RAMPAS ROJAS
        c.setStroke("#F02C3B")
                .setStrokeWidth(2);

        c.strokeRect(-24, 8, 20, 10);
        c.strokeLine(-24, 18, -14, 23);
        c.strokeLine(-14, 23, -4, 18);

        c.strokeRect(-24, -18, 20, 10);
        c.strokeLine(-24, -8, -14, -3);
        c.strokeLine(-14, -3, -4, -8);

        // RAMPAS AZULES
        c.setStroke("#3457E5")
                .setStrokeWidth(2);

        c.strokeRect(4, 8, 20, 10);
        c.strokeLine(4, 8, 14, 3);
        c.strokeLine(14, 3, 24, 8);

        c.strokeRect(4, -18, 20, 10);
        c.strokeLine(4, -18, 14, -23);
        c.strokeLine(14, -23, 24, -18);

        // MARCADORES VISUALES
        c.setFill("#F6BC4B");

        c.fillCircle(-72, -24, 2.3);
        c.fillCircle(72, 24, 2.3);
        c.fillCircle(-24, 72, 2.3);
        c.fillCircle(24, -72, 2.3);

        // PUNTOS YA CALIBRADOS EN SHOOTER INTERPOLATION (VERDE CLARO)
        c.setFill("#4ADE80");
        double[][] calibratedPoints = {
                {12, 12}, {24, 12}, {36, 12},
                {12, 24}, {24, 24}, {36, 24},
                {12, 36}, {24, 36}, {36, 36},
                {60, 28}, {60, 25}, {60, 15},
                {57, 10}, {54, 4}
        };
        for (double[] pt : calibratedPoints) {
            c.fillCircle(pt[0] - FIELD_HALF, pt[1] - FIELD_HALF, 1.5);
        }
    }

    // =========================================
    // GUARDAR MUESTRAS CSV
    // =========================================

    private void saveSample(
            double x,
            double y,
            double heading,
            double distanceCm,
            double targetTPS,
            double actualTPS,
            int hoodPosition,
            int tagId,
            double tx,
            double turretError) {

        if (csvFile == null) {
            lastSaveStatus = "CSV no disponible";
            return;
        }

        String line = String.format(
                Locale.US,
                "%d,%.3f,%.3f,%.3f,"
                        + "%.3f,%.3f,%.2f,"
                        + "%.1f,%.1f,%d,"
                        + "%d,%.3f,%.3f\n",

                sampleCount + 1,
                x,
                y,
                heading,
                GOAL_X,
                GOAL_Y,
                distanceCm,
                targetTPS,
                actualTPS,
                hoodPosition,
                tagId,
                tx,
                turretError
        );

        try (FileWriter writer =
                     new FileWriter(csvFile, true)) {

            writer.write(line);

            sampleCount++;

            lastSaveStatus =
                    "Muestra " + sampleCount + " guardada";

        } catch (IOException e) {

            lastSaveStatus =
                    "Error CSV: " + e.getMessage();
        }
    }

    // =========================================
    // STOP
    // =========================================

    @Override
    public void stop() {

        if (shooterCmd != null) {
            shooterCmd.stop();
        }

        if (intakeCmd != null) {
            intakeCmd.stop();
        }

        if (turretCmd != null) {
            turretCmd.stop();
        }

        if (driveCmd != null) {
            driveCmd.stop();
        }
    }
}
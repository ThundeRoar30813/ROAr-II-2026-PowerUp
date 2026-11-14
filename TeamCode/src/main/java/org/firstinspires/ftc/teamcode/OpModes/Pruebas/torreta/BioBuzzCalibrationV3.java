
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@TeleOp(name = "BIOBUZZ - CALIBRACION V3")
public class BioBuzzCalibrationV3 extends OpMode {

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCmd;
    private ShooterSubsystem shooter;
    private HoodSubsystem hood;
    private FtcDashboard dashboard;

    // POSICION INICIAL
    private static final double START_X = 48.0;
    private static final double START_Y = 6.0;
    private static final double START_HEADING = 90.0;

    private static final double FIELD_HALF = 72.0;

    // OBJETIVO PROVISIONAL
    // IMPORTANTE: Estas NO son coordenadas verificadas
    // del High Goal real.
    private static final double GOAL_X = 62.0;
    private static final double GOAL_Y = 57.0;

    // SHOOTER
    private static final double TPS_STEP = 25.0;
    private static final double MAX_TPS = 6000.0;
    private static final double MIN_TPS = 0.0;

    // ESTADOS DE BOTONES
    private boolean previousX = false;
    private boolean previousUp = false;
    private boolean previousDown = false;
    private boolean previousLeft = false;
    private boolean previousRight = false;
    private boolean previousSave = false;
    private boolean previousRT = false;
    private boolean previousLT = false;

    // CSV
    private int sampleCount = 0;
    private String lastSaveStatus = "Sin muestras";
    private File csvFile;

    @Override
    public void init() {

        // PEDRO PATHING
        drive = new DriveSubsystem(hardwareMap);

        drive.setInitialPose(
                START_X,
                START_Y,
                START_HEADING
        );

        driveCmd = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> gamepad1.left_stick_x,
                () -> gamepad1.right_stick_x
        );

        // SHOOTER Y HOOD
        shooter = new ShooterSubsystem(hardwareMap);
        hood = new HoodSubsystem(hardwareMap);

        shooter.setEnabled(false);

        dashboard = FtcDashboard.getInstance();

        // ARCHIVO CSV
        File folder =
                hardwareMap.appContext.getExternalFilesDir(null);

        if (folder != null) {

            csvFile = new File(
                    folder,
                    "BioBuzzCalibration.csv"
            );

            if (!csvFile.exists()) {

                try (FileWriter writer =
                             new FileWriter(csvFile, true)) {

                    writer.write(
                            "sample,x_in,y_in,heading_deg,"
                                    + "goal_x_in,goal_y_in,"
                                    + "distance_cm,target_tps,"
                                    + "actual_tps,hood_position\n"
                    );

                } catch (IOException e) {
                    lastSaveStatus = "Error creando CSV";
                }
            }

        } else {
            lastSaveStatus = "Almacenamiento no disponible";
        }

        telemetry.addLine("BIOBUZZ CALIBRACION V3");
        telemetry.addLine("Campo BioBuzz vectorial");
        telemetry.addLine("Verifica la pose inicial");
        telemetry.addLine("High Goal NO calibrado");
        telemetry.update();
    }

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
        // SHOOTER ON/OFF
        // =====================================

        if (gamepad1.x && !previousX) {
            shooter.toggle();
        }

        previousX = gamepad1.x;

        // =====================================
        // AJUSTE TPS
        // =====================================

        double targetTPS = shooter.getTargetTPS();

        boolean rt = gamepad2.right_trigger > 0.5;
        boolean lt = gamepad2.left_trigger > 0.5;

        if (rt && !previousRT) {
            targetTPS += TPS_STEP;
        }

        if (lt && !previousLT) {
            targetTPS -= TPS_STEP;
        }

        previousRT = rt;
        previousLT = lt;

        targetTPS = Math.max(
                MIN_TPS,
                Math.min(MAX_TPS, targetTPS)
        );

        shooter.setTargetTPS(targetTPS);
        shooter.update();

        // =====================================
        // HOOD
        // =====================================

        if (gamepad2.dpad_up && !previousUp) {
            hood.nextPosition();
        }

        if (gamepad2.dpad_down && !previousDown) {
            hood.previousPosition();
        }

        if (gamepad2.dpad_left && !previousLeft) {
            hood.setIndex(0);
        }

        if (gamepad2.dpad_right && !previousRight) {
            hood.setIndex(4);
        }

        previousUp = gamepad2.dpad_up;
        previousDown = gamepad2.dpad_down;
        previousLeft = gamepad2.dpad_left;
        previousRight = gamepad2.dpad_right;

        // =====================================
        // DISTANCIA PROVISIONAL
        // =====================================

        double dx = GOAL_X - x;
        double dy = GOAL_Y - y;

        double distanceCm = Math.hypot(dx, dy) * 2.54;

        // =====================================
        // GUARDAR MUESTRA
        // =====================================

        if (gamepad2.a && !previousSave) {

            saveSample(
                    x,
                    y,
                    heading,
                    distanceCm,
                    shooter.getTargetTPS(),
                    shooter.getActualTPS(),
                    hood.getPositionNumber()
            );
        }

        previousSave = gamepad2.a;

        // =====================================
        // DRIVER STATION
        // =====================================

        telemetry.addLine("=== BIOBUZZ CALIBRACION V3 ===");

        telemetry.addData("Pedro X", "%.2f in", x);
        telemetry.addData("Pedro Y", "%.2f in", y);
        telemetry.addData("Heading", "%.2f deg", heading);

        telemetry.addData(
                "Distancia PROVISIONAL",
                "%.1f cm",
                distanceCm
        );

        telemetry.addData(
                "Shooter ON",
                shooter.isEnabled()
        );

        telemetry.addData(
                "TPS objetivo",
                "%.0f",
                shooter.getTargetTPS()
        );

        telemetry.addData(
                "TPS reales",
                "%.0f",
                shooter.getActualTPS()
        );

        telemetry.addData(
                "Hood",
                hood.getPositionNumber()
        );

        telemetry.addData("Muestras", sampleCount);
        telemetry.addData("Ultimo guardado", lastSaveStatus);

        telemetry.addLine("G1 X: Shooter ON/OFF");
        telemetry.addLine("G2 RT/LT: TPS +/-25");
        telemetry.addLine("G2 DPad: Hood");
        telemetry.addLine("G2 A: Guardar muestra");

        telemetry.update();

        // =====================================
        // FTC DASHBOARD
        // =====================================

        TelemetryPacket packet = new TelemetryPacket(false);

        // CAMPO BIOBUZZ
        drawBioBuzzField(packet);

        // TELEMETRIA
        packet.put("Pedro X (in)", x);
        packet.put("Pedro Y (in)", y);
        packet.put("Heading (deg)", heading);

        packet.put(
                "Distancia provisional (cm)",
                distanceCm
        );

        packet.put(
                "Shooter Target TPS",
                shooter.getTargetTPS()
        );

        packet.put(
                "Shooter Actual TPS",
                shooter.getActualTPS()
        );

        packet.put(
                "Hood Position",
                hood.getPositionNumber()
        );

        packet.put("Muestras", sampleCount);
        packet.put("Ultimo guardado", lastSaveStatus);

        // =====================================
        // COORDENADAS DASHBOARD
        // =====================================

        double dashX = x - FIELD_HALF;
        double dashY = y - FIELD_HALF;

        double goalDashX = GOAL_X - FIELD_HALF;
        double goalDashY = GOAL_Y - FIELD_HALF;

        double headingRad = Math.toRadians(heading);

        Canvas c = packet.fieldOverlay();

        // LINEA AMARILLA AL OBJETIVO
        c.setStroke("#EAB308")
                .setStrokeWidth(2)
                .strokeLine(
                        dashX,
                        dashY,
                        goalDashX,
                        goalDashY
                );

        // OBJETIVO VERDE PROVISIONAL
        c.setFill("#16A34A")
                .fillCircle(
                        goalDashX,
                        goalDashY,
                        4
                );

        // ROBOT AZUL
        c.setStroke("#2563EB")
                .setStrokeWidth(3)
                .strokeCircle(
                        dashX,
                        dashY,
                        9
                );

        // ORIENTACION DEL ROBOT
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

    // ==========================================
    // CAMPO BIOBUZZ
    // REPRESENTACION ESQUEMATICA
    // ==========================================

    private void drawBioBuzzField(TelemetryPacket packet) {

        Canvas c = packet.fieldOverlay();

        // FONDO
        c.setFill("#292929")
                .fillRect(-72, -72, 144, 144);

        // CUADRICULA
        c.setStroke("#555555")
                .setStrokeWidth(1);

        for (int i = -72; i <= 72; i += 24) {

            c.strokeLine(i, -72, i, 72);
            c.strokeLine(-72, i, 72, i);
        }

        // BORDE EXTERIOR
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

        // PUENTE CENTRAL
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
        // NO SON COORDENADAS REALES DE APRILTAGS
        c.setFill("#F6BC4B");

        c.fillCircle(-72, -24, 2.3);
        c.fillCircle(72, 24, 2.3);
        c.fillCircle(-24, 72, 2.3);
        c.fillCircle(24, -72, 2.3);
    }

    // ==========================================
    // GUARDAR MUESTRAS CSV
    // ==========================================

    private void saveSample(
            double x,
            double y,
            double heading,
            double distanceCm,
            double targetTPS,
            double actualTPS,
            int hoodPosition
    ) {

        if (csvFile == null) {
            lastSaveStatus = "CSV no disponible";
            return;
        }

        String line = String.format(
                Locale.US,
                "%d,%.3f,%.3f,%.3f,"
                        + "%.3f,%.3f,%.2f,"
                        + "%.1f,%.1f,%d\n",

                sampleCount + 1,
                x,
                y,
                heading,
                GOAL_X,
                GOAL_Y,
                distanceCm,
                targetTPS,
                actualTPS,
                hoodPosition
        );

        try (FileWriter writer =
                     new FileWriter(csvFile, true)) {

            writer.write(line);
            sampleCount++;

            lastSaveStatus =
                    "Muestra " + sampleCount + " guardada";

        } catch (IOException e) {

            lastSaveStatus =
                    "Error al guardar: " + e.getMessage();
        }
    }

    @Override
    public void stop() {

        if (driveCmd != null) {
            driveCmd.stop();
        }

        if (shooter != null) {
            shooter.stop();
        }
    }
}

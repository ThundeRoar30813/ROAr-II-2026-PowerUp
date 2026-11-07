package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayDeque;

@TeleOp(name = "ThunderROAR - Limelight + Shooter Test")
public class ThunderROARLimelightDiagnostic extends OpMode {

    // =========================================
    // HARDWARE
    // =========================================

    private Limelight3A limelight;
    private DcMotorEx shooter;

    // =========================================
    // CONFIGURACION
    // =========================================

    private static final int PIPELINE = 0;

    private static final int TAG_34 = 34;
    private static final int TAG_37 = 37;

    private static final long MAX_AGE_MS = 150;

    // Potencia inicial de prueba
    private double shooterPower = 0.60;

    private static final double POWER_STEP = 0.05;

    // =========================================
    // ESTADO
    // =========================================

    private boolean shooterEnabled = false;

    private boolean previousA = false;
    private boolean previousUp = false;
    private boolean previousDown = false;

    // =========================================
    // ESTADISTICAS DE TX
    // =========================================

    private final ArrayDeque<Double> txSamples =
            new ArrayDeque<>();

    private static final int WINDOW_SIZE = 30;

    private double txMean = 0.0;
    private double txStdDev = 0.0;
    private double txMin = 0.0;
    private double txMax = 0.0;

    // =========================================
    // INIT
    // =========================================

    @Override
    public void init() {

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        shooter = hardwareMap.get(
                DcMotorEx.class,
                "shoot"
        );

        shooter.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        shooter.setPower(0.0);

        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        telemetry.addLine("THUNDERROAR LIMELIGHT TEST");
        telemetry.addLine("A = Shooter ON/OFF");
        telemetry.addLine("DPAD UP/DOWN = Power");
        telemetry.update();
    }

    // =========================================
    // LOOP
    // =========================================

    @Override
    public void loop() {

        // -------------------------------------
        // CONTROLES DEL SHOOTER
        // -------------------------------------

        boolean a = gamepad1.a;

        if (a && !previousA) {
            shooterEnabled = !shooterEnabled;
        }

        previousA = a;

        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        if (up && !previousUp) {
            shooterPower = Math.min(
                    1.0,
                    shooterPower + POWER_STEP
            );
        }

        if (down && !previousDown) {
            shooterPower = Math.max(
                    0.0,
                    shooterPower - POWER_STEP
            );
        }

        previousUp = up;
        previousDown = down;

        shooter.setPower(
                shooterEnabled ? shooterPower : 0.0
        );

        // -------------------------------------
        // LIMELIGHT
        // -------------------------------------

        LLResult result = limelight.getLatestResult();

        boolean found34 = false;
        boolean found37 = false;

        double tx34 = Double.NaN;
        double tx37 = Double.NaN;

        long resultAgeMs = -1;

        boolean valid = false;

        if (result != null) {

            resultAgeMs = result.getStaleness();

            valid = result.isValid()
                    && resultAgeMs >= 0
                    && resultAgeMs <= MAX_AGE_MS;

            if (valid) {

                for (LLResultTypes.FiducialResult tag :
                        result.getFiducialResults()) {

                    int id = tag.getFiducialId();

                    if (id == TAG_34) {
                        found34 = true;
                        tx34 = tag.getTargetXDegrees();
                    }

                    if (id == TAG_37) {
                        found37 = true;
                        tx37 = tag.getTargetXDegrees();
                    }
                }
            }
        }

        // -------------------------------------
        // SELECCIONAR TX PARA ANALISIS
        // -------------------------------------

        double selectedTx = Double.NaN;
        int selectedTag = -1;

        if (found37) {
            selectedTx = tx37;
            selectedTag = TAG_37;
        } else if (found34) {
            selectedTx = tx34;
            selectedTag = TAG_34;
        }

        // Solo agregar muestras validas
        if (Double.isFinite(selectedTx)) {

            txSamples.addLast(selectedTx);

            while (txSamples.size() > WINDOW_SIZE) {
                txSamples.removeFirst();
            }

            calculateStatistics();
        }

        // -------------------------------------
        // TELEMETRIA
        // -------------------------------------

        telemetry.addLine("===== SHOOTER =====");

        telemetry.addData(
                "Shooter",
                shooterEnabled ? "ON" : "OFF"
        );

        telemetry.addData(
                "Power",
                "%.2f",
                shooterPower
        );

        telemetry.addData(
                "Velocity (ticks/s)",
                "%.1f",
                shooter.getVelocity()
        );

        telemetry.addLine("===== LIMELIGHT =====");

        telemetry.addData("Valid", valid);

        telemetry.addData(
                "Result Age (ms)",
                "%d",
                resultAgeMs
        );

        telemetry.addData("Tag 34", found34);
        telemetry.addData("TX 34", "%.2f", tx34);

        telemetry.addData("Tag 37", found37);
        telemetry.addData("TX 37", "%.2f", tx37);

        telemetry.addLine("===== TX ANALYSIS =====");

        telemetry.addData(
                "Selected Tag",
                selectedTag
        );

        telemetry.addData(
                "Selected TX",
                "%.2f",
                selectedTx
        );

        telemetry.addData(
                "TX Mean",
                "%.3f",
                txMean
        );

        telemetry.addData(
                "TX StdDev",
                "%.3f",
                txStdDev
        );

        telemetry.addData(
                "TX Min",
                "%.2f",
                txMin
        );

        telemetry.addData(
                "TX Max",
                "%.2f",
                txMax
        );

        telemetry.addData(
                "Samples",
                txSamples.size()
        );

        telemetry.update();
    }

    // =========================================
    // CALCULAR ESTADISTICAS
    // =========================================

    private void calculateStatistics() {

        if (txSamples.isEmpty()) {
            return;
        }

        double sum = 0.0;

        txMin = Double.POSITIVE_INFINITY;
        txMax = Double.NEGATIVE_INFINITY;

        for (double tx : txSamples) {

            sum += tx;

            txMin = Math.min(txMin, tx);
            txMax = Math.max(txMax, tx);
        }

        txMean = sum / txSamples.size();

        double variance = 0.0;

        for (double tx : txSamples) {

            double error = tx - txMean;
            variance += error * error;
        }

        variance /= txSamples.size();

        txStdDev = Math.sqrt(variance);
    }

    // =========================================
    // STOP
    // =========================================

    @Override
    public void stop() {

        if (shooter != null) {
            shooter.setPower(0.0);
        }

        if (limelight != null) {
            limelight.stop();
        }
    }
}
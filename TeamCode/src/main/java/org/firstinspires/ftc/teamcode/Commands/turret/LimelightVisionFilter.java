
package org.firstinspires.ftc.teamcode.Commands.turret;

import java.util.ArrayDeque;
import java.util.Arrays;

public class LimelightVisionFilter {

    public enum Mode {
        NO_VISION,
        STABLE,
        VIBRATING
    }

    // ==========================================
    // CONFIGURACION
    // ==========================================

    // Lecturas de TX utilizadas para analizar ruido
    private static final int WINDOW_SIZE = 7;

    // Diferencia maxima para aceptar una lectura
    private static final double MAX_JUMP_DEG = 5.0;

    // Desviacion estandar para detectar vibraciones
    private static final double VIBRATION_THRESHOLD = 1.0;

    // Suavizado para TX estable
    private static final double ALPHA_STABLE = 0.40;

    // Suavizado para TX con vibraciones
    private static final double ALPHA_VIBRATING = 0.12;

    // Correccion maxima que puede aportar la vision
    private static final double MAX_CORRECTION_DEG = 5.0;

    // Tiempo maximo sin una lectura valida
    private static final long TIMEOUT_MS = 250;

    // Cantidad de lecturas para habilitar vision
    private static final int MIN_SAMPLES = 4;

    // ==========================================
    // VARIABLES
    // ==========================================

    private final ArrayDeque<Double> samples =
            new ArrayDeque<>();

    private Mode mode = Mode.NO_VISION;

    private double filteredTx = 0.0;
    private double standardDeviation = 0.0;

    private boolean initialized = false;
    private boolean visionReady = false;

    private long lastValidTime = 0;

    private int rejectedSamples = 0;

    // ==========================================
    // RECIBIR UNA MEDICION DE LIMELIGHT
    // ==========================================

    public void addMeasurement(
            double tx,
            boolean valid,
            long nowMs
    ) {

        if (!valid || !Double.isFinite(tx)) {
            checkTimeout(nowMs);
            return;
        }

        // Evitar lecturas repetidas o fuera de orden
        if (nowMs <= lastValidTime) {
            return;
        }

        // Rechazar saltos muy grandes cuando
        // ya tenemos una referencia.
        if (initialized &&
                Math.abs(tx - filteredTx) > MAX_JUMP_DEG) {

            rejectedSamples++;
            checkTimeout(nowMs);
            return;
        }

        lastValidTime = nowMs;

        // Guardar muestra
        samples.addLast(tx);

        while (samples.size() > WINDOW_SIZE) {
            samples.removeFirst();
        }

        // Primera medicion
        if (!initialized) {
            filteredTx = tx;
            initialized = true;
        }

        standardDeviation =
                calculateStandardDeviation();

        // Detectar estabilidad
        if (standardDeviation > VIBRATION_THRESHOLD) {
            mode = Mode.VIBRATING;
        } else {
            mode = Mode.STABLE;
        }

        double alpha =
                mode == Mode.STABLE
                        ? ALPHA_STABLE
                        : ALPHA_VIBRATING;

        // Filtro exponencial
        filteredTx += alpha * (tx - filteredTx);

        visionReady = samples.size() >= MIN_SAMPLES;
    }

    // ==========================================
    // CONTROL DE PERDIDA DE VISION
    // ==========================================

    public void checkTimeout(long nowMs) {

        if (lastValidTime == 0 ||
                nowMs - lastValidTime > TIMEOUT_MS) {

            mode = Mode.NO_VISION;
            visionReady = false;
            initialized = false;

            samples.clear();
            filteredTx = 0.0;
            standardDeviation = 0.0;
        }
    }

    // ==========================================
    // DESVIACION ESTANDAR
    // ==========================================

    private double calculateStandardDeviation() {

        if (samples.size() < 2) {
            return 0.0;
        }

        double mean = 0.0;

        for (double sample : samples) {
            mean += sample;
        }

        mean /= samples.size();

        double variance = 0.0;

        for (double sample : samples) {
            double difference = sample - mean;
            variance += difference * difference;
        }

        variance /= samples.size();

        return Math.sqrt(variance);
    }

    // ==========================================
    // CORRECCION DE PUNTERIA
    // ==========================================

    public double getCorrectionDeg() {

        if (!visionReady ||
                mode == Mode.NO_VISION) {
            return 0.0;
        }

        // Correccion mas conservadora si vibra
        double gain =
                mode == Mode.STABLE ? 1.0 : 0.35;

        return clamp(
                filteredTx * gain,
                -MAX_CORRECTION_DEG,
                MAX_CORRECTION_DEG
        );
    }

    // ==========================================
    // TELEMETRIA
    // ==========================================

    public Mode getMode() {
        return mode;
    }

    public double getFilteredTx() {
        return filteredTx;
    }

    public double getStandardDeviation() {
        return standardDeviation;
    }

    public boolean isVisionReady() {
        return visionReady;
    }

    public int getRejectedSamples() {
        return rejectedSamples;
    }

    public int getSampleCount() {
        return samples.size();
    }

    // ==========================================
    // RESET
    // ==========================================

    public void reset() {

        samples.clear();

        mode = Mode.NO_VISION;

        filteredTx = 0.0;
        standardDeviation = 0.0;

        initialized = false;
        visionReady = false;

        lastValidTime = 0;
        rejectedSamples = 0;
    }

    private static double clamp(
            double value,
            double min,
            double max
    ) {
        return Math.max(min, Math.min(max, value));
    }
}

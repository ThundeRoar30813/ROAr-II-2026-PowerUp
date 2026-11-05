
package org.firstinspires.ftc.teamcode.Commands.turret;

public class TurretSafetyManager {

    public enum Status {
        SAFE,
        TARGET_CLAMPED,
        PHYSICAL_LIMIT,
        INVALID_TARGET
    }

    // Valores provisionales.
    // Deben sustituirse por los limites reales.
    private final double minAngleDeg;
    private final double maxAngleDeg;

    // Margen para reducir riesgo de tocar topes
    private final double safetyMarginDeg;

    private Status status = Status.SAFE;

    private double requestedAngleDeg = 0.0;
    private double safeTargetDeg = 0.0;

    public TurretSafetyManager(
            double minAngleDeg,
            double maxAngleDeg,
            double safetyMarginDeg
    ) {

        if (!Double.isFinite(minAngleDeg)
                || !Double.isFinite(maxAngleDeg)
                || !Double.isFinite(safetyMarginDeg)
                || minAngleDeg >= maxAngleDeg
                || safetyMarginDeg < 0
                || safetyMarginDeg * 2 >=
                maxAngleDeg - minAngleDeg) {

            throw new IllegalArgumentException(
                    "Invalid turret limits"
            );
        }

        this.minAngleDeg = minAngleDeg;
        this.maxAngleDeg = maxAngleDeg;
        this.safetyMarginDeg = safetyMarginDeg;
    }

    // ==========================================
    // CALCULAR TARGET SEGURO
    // ==========================================

    public double calculateSafeTarget(
            double requestedAngle,
            double currentAngle
    ) {

        if (!Double.isFinite(requestedAngle)
                || !Double.isFinite(currentAngle)) {

            status = Status.INVALID_TARGET;
            safeTargetDeg = Double.isFinite(currentAngle)
                    ? currentAngle : 0.0;

            return safeTargetDeg;
        }

        requestedAngleDeg = requestedAngle;

        double safeMin =
                minAngleDeg + safetyMarginDeg;

        double safeMax =
                maxAngleDeg - safetyMarginDeg;

        safeTargetDeg = clamp(
                requestedAngle,
                safeMin,
                safeMax
        );

        if (currentAngle < safeMin
                || currentAngle > safeMax) {

            status = Status.PHYSICAL_LIMIT;

        } else if (Math.abs(
                requestedAngle - safeTargetDeg
        ) > 0.001) {

            status = Status.TARGET_CLAMPED;

        } else {

            status = Status.SAFE;
        }

        return safeTargetDeg;
    }

    // ==========================================
    // AUTORIZAR MOVIMIENTO
    // ==========================================

    public boolean canMove(
            double currentAngle,
            double requestedPower
    ) {

        if (!Double.isFinite(currentAngle)
                || !Double.isFinite(requestedPower)) {
            return false;
        }

        double safeMin =
                minAngleDeg + safetyMarginDeg;

        double safeMax =
                maxAngleDeg - safetyMarginDeg;

        // Supone que potencia positiva aumenta
        // el angulo del encoder.
        if (currentAngle >= safeMax
                && requestedPower > 0.0) {
            return false;
        }

        if (currentAngle <= safeMin
                && requestedPower < 0.0) {
            return false;
        }

        return true;
    }

    // ==========================================
    // TELEMETRIA
    // ==========================================

    public Status getStatus() {
        return status;
    }

    public double getRequestedAngleDeg() {
        return requestedAngleDeg;
    }

    public double getSafeTargetDeg() {
        return safeTargetDeg;
    }

    public double getSafeMinDeg() {
        return minAngleDeg + safetyMarginDeg;
    }

    public double getSafeMaxDeg() {
        return maxAngleDeg - safetyMarginDeg;
    }

    private static double clamp(
            double value,
            double min,
            double max
    ) {
        return Math.max(min, Math.min(max, value));
    }
}

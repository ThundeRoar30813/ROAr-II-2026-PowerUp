package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemp;

public class HybridTurretTrackingCommand {

    // CONFIGURACION INICIAL CONSERVADORA
    private static final double MIN_ANGLE = -60.0;
    private static final double MAX_ANGLE = 60.0;

    private static final double MAX_VISION_CORRECTION = 3.0;
    private static final double VISION_GAIN = 0.20;
    private static final double VISION_ALPHA = 0.10;
    private static final double MAX_VISION_RATE = 12.0;

    private static final double MAX_TX = 5.0;
    private static final double MAX_TX_JUMP = 2.0;
    private static final long MAX_STALENESS_MS = 120;
    private static final double MAX_TARGET_SPEED = 180.0;

    // IMPORTANTE:
    // +1 o -1 se calibra experimentalmente.
    // Empezamos con la vision desactivada.
    private static final double VISION_SIGN = 1.0;

    private final DriveSubsystem drive;
    private final TurretSubsystemp turret;
    private final Limelight3A limelight;

    private double targetX;
    private double targetY;
    private int targetTagId;

    private boolean enabled = false;
    private boolean visionEnabled = false;

    private double fieldOffsetDeg = 0.0;
    private double visionCorrectionDeg = 0.0;
    private double desiredAngleDeg = 0.0;

    private double lastFrameTimestamp = Double.NaN;
    private double lastTx = Double.NaN;

    private long lastVisionMs = 0;
    private long lastUpdateNs = 0;

    private boolean tagVisible = false;
    private boolean lastVisionAccepted = false;

    public HybridTurretTrackingCommand(
            DriveSubsystem drive,
            TurretSubsystemp turret,
            Limelight3A limelight,
            double targetX,
            double targetY,
            int targetTagId) {

        this.drive = drive;
        this.turret = turret;
        this.limelight = limelight;

        this.targetX = targetX;
        this.targetY = targetY;
        this.targetTagId = targetTagId;
    }

    public void enable() {

        // La torreta debe estar apuntando
        // fisicamente al objetivo antes de habilitar.
        double fieldBearing = getFieldBearingDeg();
        double robotHeading = drive.getHeadingDegrees();
        double turretAngle = turret.getCurrentAngleDeg();

        fieldOffsetDeg = normalize(
                fieldBearing - robotHeading - turretAngle
        );

        visionCorrectionDeg = 0.0;
        lastTx = Double.NaN;
        lastFrameTimestamp = Double.NaN;

        lastUpdateNs = System.nanoTime();
        lastVisionMs = 0;

        tagVisible = false;
        lastVisionAccepted = false;

        desiredAngleDeg = turretAngle;
        enabled = true;
    }

    public void disable() {
        enabled = false;
        visionEnabled = false;
        tagVisible = false;
        lastVisionAccepted = false;
        turret.stop();
    }

    public void setVisionEnabled(boolean value) {
        visionEnabled = value;
        visionCorrectionDeg = 0.0;
        lastTx = Double.NaN;
        lastFrameTimestamp = Double.NaN;
        lastVisionMs = 0;
    }

    public void execute() {

        if (!enabled) {
            return;
        }

        long nowNs = System.nanoTime();
        long nowMs = nowNs / 1000000L;

        double dt = (nowNs - lastUpdateNs) / 1e9;
        lastUpdateNs = nowNs;

        if (dt <= 0.0 || dt > 0.2) {
            dt = 0.02;
        }

        // 1. APUNTADO GEOMETRICO
        double fieldBearing = getFieldBearingDeg();

        double geometricAngle = normalize(
                fieldBearing
                        - drive.getHeadingDegrees()
                        - fieldOffsetDeg
        );

        // 2. CORRECCION VISUAL
        tagVisible = false;
        lastVisionAccepted = false;

        if (visionEnabled) {

            LLResult result = limelight.getLatestResult();

            if (result != null) {

                long age = result.getStaleness();

                boolean fresh =
                        age >= 0
                                && age <= MAX_STALENESS_MS;

                double timestamp = result.getTimestamp();

                boolean newFrame =
                        Double.isFinite(timestamp)
                                && timestamp > 0.0
                                && (Double.isNaN(lastFrameTimestamp)
                                || timestamp > lastFrameTimestamp);

                if (newFrame) {

                    lastFrameTimestamp = timestamp;

                    if (fresh && result.isValid()) {

                        for (LLResultTypes.FiducialResult tag
                                : result.getFiducialResults()) {

                            if (tag.getFiducialId()
                                    != targetTagId) {
                                continue;
                            }

                            double tx =
                                    tag.getTargetXDegrees();

                            if (!Double.isFinite(tx)) {
                                break;
                            }

                            tagVisible = true;

                            boolean acceptable =
                                    Math.abs(tx) <= MAX_TX;

                            if (Double.isFinite(lastTx)) {
                                acceptable &=
                                        Math.abs(tx - lastTx)
                                                <= MAX_TX_JUMP;
                            }

                            if (acceptable) {

                                // Esta es una correccion
                                // experimental, no una
                                // calibracion definitiva.
                                double requestedCorrection =
                                        clamp(
                                                VISION_SIGN
                                                        * tx
                                                        * VISION_GAIN,
                                                -MAX_VISION_CORRECTION,
                                                MAX_VISION_CORRECTION
                                        );

                                double filtered =
                                        visionCorrectionDeg
                                                + VISION_ALPHA
                                                * (requestedCorrection
                                                - visionCorrectionDeg);

                                double maxStep =
                                        MAX_VISION_RATE * dt;

                                visionCorrectionDeg += clamp(
                                        filtered
                                                - visionCorrectionDeg,
                                        -maxStep,
                                        maxStep
                                );

                                lastTx = tx;
                                lastVisionMs = nowMs;
                                lastVisionAccepted = true;
                            }

                            break;
                        }
                    }
                }
            }

            // Si desaparece el tag,
            // regresar suavemente a odometria.
            if (nowMs - lastVisionMs > 250) {

                double maxStep = MAX_VISION_RATE * dt;

                visionCorrectionDeg += clamp(
                        -visionCorrectionDeg,
                        -maxStep,
                        maxStep
                );

                lastTx = Double.NaN;
            }

        } else {
            visionCorrectionDeg = 0.0;
        }

        // 3. ANGULO FINAL
        double requestedAngle =
                geometricAngle + visionCorrectionDeg;

        desiredAngleDeg = clamp(
                requestedAngle,
                MIN_ANGLE,
                MAX_ANGLE
        );

        // 4. VELOCIDAD DE REFERENCIA
        // En esta primera version usamos 0.
        // Despues integraremos feedforward
        // de velocidad angular real.
        turret.setTargetVelocityDegPerSec(0.0);
        turret.setTargetAngleDeg(desiredAngleDeg);

        // 5. PID EXISTENTE
        turret.update();
    }

    private double getFieldBearingDeg() {

        double dx = targetX - drive.getX();
        double dy = targetY - drive.getY();

        return Math.toDegrees(Math.atan2(dy, dx));
    }

    private double normalize(double angle) {

        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;

        return angle;
    }

    private double clamp(
            double value,
            double minimum,
            double maximum) {

        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isVisionEnabled() {
        return visionEnabled;
    }

    public boolean isTagVisible() {
        return tagVisible;
    }

    public boolean wasVisionAccepted() {
        return lastVisionAccepted;
    }

    public double getDesiredAngleDeg() {
        return desiredAngleDeg;
    }

    public double getVisionCorrectionDeg() {
        return visionCorrectionDeg;
    }

    public double getCurrentAngleDeg() {
        return turret.getCurrentAngleDeg();
    }
}
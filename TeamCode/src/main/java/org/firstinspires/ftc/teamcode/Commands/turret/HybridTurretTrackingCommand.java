
package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemp;

public class HybridTurretTrackingCommand {

    public enum TrackingMode {
        DISABLED,
        ODOMETRY,
        VISION_STABLE,
        VISION_VIBRATING,
        LIMIT_REACHED
    }


    private final TurretSafetyManager safety =
            new TurretSafetyManager(
                    -115.0,
                    120.0,
                    5.0
            );


    // ==========================================
    // CONFIGURACION
    // ==========================================

    // Limites provisionales: verificar con el robot
    private static final double MIN_TURRET_DEG = -115.0;
    private static final double MAX_TURRET_DEG = 120.0;

    private static final double MAX_TARGET_VELOCITY = 500.0;

    private static final double MAX_RESULT_AGE_MS = 150.0;

    // Limite de velocidad de correccion visual
    private static final double MAX_VISION_RATE_DEG_S = 45.0;

    // Cambiar solamente despues de comprobar el signo
    private static final double VISION_SIGN = 1.0;

    // ==========================================
    // COMPONENTES
    // ==========================================

    private final DriveSubsystem drive;
    private final TurretSubsystemp turret;
    private final Limelight3A limelight;

    private final LimelightVisionFilter visionFilter =
            new LimelightVisionFilter();

    // ==========================================
    // OBJETIVO
    // ==========================================

    private double targetX;
    private double targetY;
    private int targetTagId;

    private boolean enabled = false;

    // Offset de montaje / referencia de torreta
    private double turretFieldZeroDeg = 0.0;

    // Correccion visual aplicada gradualmente
    private double appliedVisionCorrectionDeg = 0.0;

    // Para feedforward
    private double previousDesiredAngleDeg = 0.0;
    private long previousTimeNs = 0;

    // Para no procesar dos veces la misma imagen
    private LLResult previousResult = null;

    private TrackingMode mode = TrackingMode.DISABLED;

    private double fieldAngleDeg = 0.0;
    private double desiredAngleDeg = 0.0;
    private double distanceToTarget = 0.0;
    private double targetVelocityDegPerSec = 0.0;

    // ==========================================
    // CONSTRUCTOR
    // ==========================================

    public HybridTurretTrackingCommand(
            DriveSubsystem drive,
            TurretSubsystemp turret,
            Limelight3A limelight,
            double targetX,
            double targetY,
            int targetTagId
    ) {
        this.drive = drive;
        this.turret = turret;
        this.limelight = limelight;

        this.targetX = targetX;
        this.targetY = targetY;
        this.targetTagId = targetTagId;
    }

    // ==========================================
    // ACTIVAR TRACKING
    // ==========================================

    public void enable() {

        // No reiniciamos el encoder.
        // El cero fisico debe calibrarse aparte.

        double robotX = drive.getX();
        double robotY = drive.getY();

        double angleToTarget = Math.toDegrees(
                Math.atan2(
                        targetY - robotY,
                        targetX - robotX
                )
        );

        double robotHeading = drive.getHeadingDegrees();

        // Convencion heredada del tracking anterior:
        // la torreta inicia apuntando al objetivo.
        turretFieldZeroDeg = normalizeDegrees(
                angleToTarget
                        - robotHeading
                        - turret.getCurrentAngleDeg()
        );

        visionFilter.reset();
        previousResult = null;
        appliedVisionCorrectionDeg = 0.0;

        previousDesiredAngleDeg = turret.getCurrentAngleDeg();
        previousTimeNs = System.nanoTime();

        enabled = true;
        mode = TrackingMode.ODOMETRY;
    }

    // ==========================================
    // DESACTIVAR
    // ==========================================

    public void disable() {

        enabled = false;
        mode = TrackingMode.DISABLED;

        turret.setTargetVelocityDegPerSec(0.0);
        turret.setTargetAngleDeg(turret.getCurrentAngleDeg());

        turret.stop();
    }

    // ==========================================
    // CAMBIAR OBJETIVO
    // ==========================================

    public void setTarget(
            double x,
            double y,
            int tagId
    ) {

        boolean changed =
                targetTagId != tagId
                        || targetX != x
                        || targetY != y;

        targetX = x;
        targetY = y;
        targetTagId = tagId;

        if (changed) {
            visionFilter.reset();
            previousResult = null;
            appliedVisionCorrectionDeg = 0.0;
        }
    }

    // ==========================================
    // EJECUCION PRINCIPAL
    // ==========================================

    public void execute() {

        if (!enabled) {
            return;
        }

        long nowNs = System.nanoTime();
        long nowMs = nowNs / 1_000_000L;

        double dt = (nowNs - previousTimeNs) / 1e9;

        if (dt <= 0.0 || dt > 0.25) {
            dt = 0.02;
        }

        previousTimeNs = nowNs;

        // --------------------------------------
        // 1. POSICION DEL ROBOT
        // --------------------------------------

        double robotX = drive.getX();
        double robotY = drive.getY();
        double robotHeading = drive.getHeadingDegrees();

        double dx = targetX - robotX;
        double dy = targetY - robotY;

        distanceToTarget = Math.hypot(dx, dy);

        fieldAngleDeg = Math.toDegrees(
                Math.atan2(dy, dx)
        );

        // --------------------------------------
        // 2. ANGULO BASE POR PINPOINT
        // --------------------------------------

        double odometryAngleDeg = normalizeDegrees(
                fieldAngleDeg
                        - robotHeading
                        - turretFieldZeroDeg
        );

        // --------------------------------------
        // 3. LEER LIMELIGHT
        // --------------------------------------

        LLResult result = limelight.getLatestResult();

        if (result != null && result != previousResult) {

            previousResult = result;

            boolean fresh =
                    result.getStaleness() >= 0.0
                            && result.getStaleness() <= MAX_RESULT_AGE_MS;

            if (result.isValid() && fresh) {

                for (LLResultTypes.FiducialResult tag :
                        result.getFiducialResults()) {

                    if (tag.getFiducialId() == targetTagId) {

                        visionFilter.addMeasurement(
                                tag.getTargetXDegrees(),
                                true,
                                nowMs
                        );

                        break;
                    }
                }
            }
        }

        visionFilter.checkTimeout(nowMs);

        // --------------------------------------
        // 4. CORRECCION VISUAL
        // --------------------------------------

        double requestedCorrection =
                VISION_SIGN * visionFilter.getCorrectionDeg();

        // Aplicar gradualmente la correccion
        double maxStep =
                MAX_VISION_RATE_DEG_S * dt;

        double difference =
                requestedCorrection
                        - appliedVisionCorrectionDeg;

        appliedVisionCorrectionDeg += clamp(
                difference,
                -maxStep,
                maxStep
        );

        // --------------------------------------
        // 5. ANGULO FINAL
        // --------------------------------------

        double requestedAngleDeg =
                odometryAngleDeg
                        + appliedVisionCorrectionDeg;


        desiredAngleDeg = safety.calculateSafeTarget(
                requestedAngleDeg,
                turret.getCurrentAngleDeg()
        );

        boolean limitReached =
                safety.getStatus()
                        != TurretSafetyManager.Status.SAFE;


        // --------------------------------------
        // 6. VELOCIDAD OBJETIVO PARA FEEDFORWARD
        // --------------------------------------

        double angleDifference = normalizeDegrees(
                desiredAngleDeg - previousDesiredAngleDeg
        );

        targetVelocityDegPerSec = clamp(
                angleDifference / dt,
                -MAX_TARGET_VELOCITY,
                MAX_TARGET_VELOCITY
        );

        // No aplicar feedforward hacia fuera
        // de los limites cuando el target
        // ya esta limitado.
        if (limitReached) {
            targetVelocityDegPerSec = 0.0;
        }

        previousDesiredAngleDeg = desiredAngleDeg;

        // --------------------------------------
        // 7. CONTROLAR TORRETA
        // --------------------------------------

        turret.setTargetAngleDeg(desiredAngleDeg);

        turret.setTargetVelocityDegPerSec(
                targetVelocityDegPerSec
        );

        turret.update();

        // --------------------------------------
        // 8. ESTADO DEL TRACKING
        // --------------------------------------

        if (limitReached) {

            mode = TrackingMode.LIMIT_REACHED;

        } else if (!visionFilter.isVisionReady()) {

            mode = TrackingMode.ODOMETRY;

        } else if (visionFilter.getMode()
                == LimelightVisionFilter.Mode.VIBRATING) {

            mode = TrackingMode.VISION_VIBRATING;

        } else {

            mode = TrackingMode.VISION_STABLE;
        }
    }

    // ==========================================
    // TELEMETRIA
    // ==========================================

    public TrackingMode getMode() {
        return mode;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public int getTargetTagId() {
        return targetTagId;
    }

    public double getDistanceToTarget() {
        return distanceToTarget;
    }

    public double getFieldAngleDeg() {
        return fieldAngleDeg;
    }

    public double getDesiredAngleDeg() {
        return desiredAngleDeg;
    }

    public double getAppliedVisionCorrectionDeg() {
        return appliedVisionCorrectionDeg;
    }

    public double getFilteredTx() {
        return visionFilter.getFilteredTx();
    }

    public double getVisionNoiseDeg() {
        return visionFilter.getStandardDeviation();
    }

    public double getTargetVelocityDegPerSec() {
        return targetVelocityDegPerSec;
    }

    public boolean isVisionReady() {
        return visionFilter.isVisionReady();
    }

    // ==========================================
    // UTILIDADES
    // ==========================================

    private static double normalizeDegrees(double angle) {

        while (angle > 180.0) {
            angle -= 360.0;
        }

        while (angle < -180.0) {
            angle += 360.0;
        }

        return angle;
    }

    private static double clamp(
            double value,
            double min,
            double max
    ) {
        return Math.max(min, Math.min(max, value));
    }
}


package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

public class TurretAimControllerV77 {

    // =====================================================
    // MODOS DE APUNTADO
    // =====================================================

    public enum Mode {
        COMBINED,
        PEDRO_ONLY,
        VISION_ONLY
    }

    // =====================================================
    // SUBSYSTEMS
    // =====================================================

    private final DriveSubsystem drive;
    private final TurretSubsystemV4 turret;
    private final TurretVisionV5 vision;

    // =====================================================
    // COORDENADAS APRILTAGS
    // UNIDADES: PULGADAS
    // =====================================================

    // ZONA SUPERIOR

    public static double TAG30_X = 64.76;
    public static double TAG30_Y = 81.39;

    public static double TAG33_X = 49.37;
    public static double TAG33_Y = 83.00;

    // TAG 34 - REFERENCIA ANTERIOR
    public static double TAG34_X = 51.0;
    public static double TAG34_Y = 57.0;

    // ZONA INFERIOR

    public static double TAG35_X = 55.0;
    public static double TAG35_Y = 57.0;

    public static double TAG37_X = 61.897345045022966;
    public static double TAG37_Y = 58.16307045879971;

    // =====================================================
    // DIRECCIONES
    // =====================================================

    // Mantener el signo que funciono en tus pruebas
    public static double TURRET_DIRECTION_SIGN = -1.0;

    // Signo de correccion horizontal de Limelight
    public static double TX_SIGN = -1.0;

    // Desfase entre cero mecanico de torreta
    // y direccion frontal del robot
    public static double TURRET_FIELD_ZERO_DEG = 0.0;

    // =====================================================
    // CORRECCION DE LIMELIGHT
    // =====================================================

    public static double TX_DEADBAND = 1.5;

    // Velocidad de integracion de la correccion
    public static double VISION_OFFSET_SPEED = 1.5;

    public static double MAX_VISION_OFFSET = 12.0;

    // Ganancia de vision pura
    public static double VISION_ONLY_KP = 0.60;

    // Antiguedad maxima aceptada de una medicion
    public static double MAX_VISION_AGE_MS = 250.0;

    // =====================================================
    // COMPENSACION DEL CHASIS
    // =====================================================

    public static double CHASSIS_VELOCITY_FILTER = 0.30;

    public static double MAX_CHASSIS_VELOCITY_DEG_S = 500.0;

    public static double MAX_TARGET_RATE_DEG_S = 500.0;

    // =====================================================
    // ESTADO
    // =====================================================

    private Mode mode = Mode.COMBINED;

    private boolean enabled = false;

    private String status = "DISABLED";

    private int selectedTag = -1;

    private double baseTarget = 0.0;
    private double visionOffset = 0.0;

    private double lastRobotHeading = Double.NaN;

    private double filteredChassisAngularVelocity = 0.0;

    private double lastCommandTarget = Double.NaN;

    private long lastUpdateNs = 0;

    private boolean validPedroTarget = false;
    private boolean validVisionTarget = false;

    // =====================================================
    // CONSTRUCTOR
    // =====================================================

    public TurretAimControllerV77(
            DriveSubsystem drive,
            TurretSubsystemV4 turret,
            TurretVisionV5 vision) {

        this.drive = drive;
        this.turret = turret;
        this.vision = vision;
    }

    // =====================================================
    // COORDENADAS SEGUN APRILTAG
    // =====================================================

    private double[] targetFor(int tagId) {

        double x;
        double y;

        switch (tagId) {

            case 30:
                x = TAG30_X;
                y = TAG30_Y;
                break;

            case 33:
                x = TAG33_X;
                y = TAG33_Y;
                break;

            case 34:
                x = TAG34_X;
                y = TAG34_Y;
                break;

            case 35:
                x = TAG35_X;
                y = TAG35_Y;
                break;

            case 37:
                x = TAG37_X;
                y = TAG37_Y;
                break;

            default:
                return null;
        }

        if (!Double.isFinite(x) ||
                !Double.isFinite(y)) {
            return null;
        }

        return new double[]{x, y};
    }

    // =====================================================
    // CONTROL DE MODO
    // =====================================================

    public void setMode(Mode nextMode) {

        if (nextMode == null) {
            return;
        }

        if (mode == nextMode) {
            return;
        }

        mode = nextMode;

        if (enabled) {
            resetTracking();
        }
    }

    public Mode getMode() {
        return mode;
    }

    // =====================================================
    // ACTIVAR TRACKING
    // =====================================================

    public void enable() {

        if (turret.isFault()) {
            enabled = false;
            status = "TURRET_FAULT";
            return;
        }

        turret.enableHold();

        if (!turret.isEnabled()) {
            enabled = false;
            status = "TURRET_NOT_ENABLED";
            return;
        }

        enabled = true;

        resetTracking();
    }

    // =====================================================
    // REINICIAR TRACKING
    // =====================================================

    private void resetTracking() {

        baseTarget = turret.getAngle();

        visionOffset = 0.0;

        selectedTag = vision.getSelectedTag();

        lastRobotHeading = Double.NaN;

        filteredChassisAngularVelocity = 0.0;

        lastCommandTarget = turret.getAngle();

        lastUpdateNs = System.nanoTime();

        validPedroTarget = false;
        validVisionTarget = false;

        status = "READY";
    }

    // =====================================================
    // DESACTIVAR TRACKING
    // =====================================================

    public void disable() {

        enabled = false;

        turret.setVelocityFeedforward(0);

        turret.disable();

        status = "DISABLED";
    }

    // =====================================================
    // NORMALIZAR ANGULOS
    // =====================================================

    private double wrap180(double angle) {

        angle %= 360.0;

        if (angle > 180.0) {
            angle -= 360.0;
        }

        if (angle < -180.0) {
            angle += 360.0;
        }

        return angle;
    }

    // =====================================================
    // BUSCAR ANGULO EQUIVALENTE DENTRO DE LIMITES
    // =====================================================

    private double findSafeAngle(
            double desiredAngle,
            double currentAngle) {

        double min = TurretSubsystemV4.MIN_ANGLE_DEG;
        double max = TurretSubsystemV4.MAX_ANGLE_DEG;

        double bestAngle = Double.NaN;
        double bestDistance = Double.POSITIVE_INFINITY;

        // Buscar vueltas equivalentes
        for (int k = -2; k <= 2; k++) {

            double candidate =
                    desiredAngle + 360.0 * k;

            if (candidate < min || candidate > max) {
                continue;
            }

            double distance =
                    Math.abs(candidate - currentAngle);

            if (distance < bestDistance) {

                bestDistance = distance;
                bestAngle = candidate;
            }
        }

        return bestAngle;
    }

    // =====================================================
    // LIMITAR VELOCIDAD DEL TARGET
    // =====================================================

    private double limitTargetRate(
            double desired,
            double dt) {

        if (!Double.isFinite(lastCommandTarget)) {
            lastCommandTarget = turret.getAngle();
        }

        double maxStep =
                MAX_TARGET_RATE_DEG_S * dt;

        double delta =
                desired - lastCommandTarget;

        delta = Range.clip(
                delta,
                -maxStep,
                maxStep
        );

        lastCommandTarget += delta;

        return lastCommandTarget;
    }

    // =====================================================
    // VERIFICAR LIMELIGHT
    // =====================================================

    private boolean isVisionUsable() {

        if (!vision.hasTarget()) {
            return false;
        }

        double tx = vision.getTx();
        double age = vision.getAgeMs();

        return Double.isFinite(tx)
                && Double.isFinite(age)
                && age >= 0
                && age <= MAX_VISION_AGE_MS;
    }

    // =====================================================
    // ACTUALIZACION PRINCIPAL
    // =====================================================

    public void update() {

        if (!enabled) {
            return;
        }

        if (turret.isFault()) {

            disable();

            status = "TURRET_FAULT";
            return;
        }

        long now = System.nanoTime();

        double dt =
                (now - lastUpdateNs) * 1e-9;

        lastUpdateNs = now;

        if (!Double.isFinite(dt)
                || dt < 0.001
                || dt > 0.15) {

            dt = 0.02;
        }

        // =========================================
        // APRILTAG ACTUAL
        // =========================================

        int currentTag = vision.getSelectedTag();

        if (currentTag != selectedTag) {

            selectedTag = currentTag;

            visionOffset = 0.0;

            lastRobotHeading = Double.NaN;

            filteredChassisAngularVelocity = 0.0;

            lastCommandTarget = turret.getAngle();

            status = "TAG_CHANGED";
        }

        // =========================================
        // DATOS DE PEDRO
        // =========================================

        double robotX = drive.getX();
        double robotY = drive.getY();

        double robotHeading =
                drive.getHeadingDegrees();

        double currentTurretAngle =
                turret.getAngle();

        double[] targetCoords =
                targetFor(selectedTag);

        validPedroTarget = false;

        validVisionTarget = isVisionUsable();

        double pedroTarget = currentTurretAngle;

        double velocityFeedforward = 0.0;

        // =========================================
        // CALCULO DE APUNTADO PEDRO
        // =========================================

        if (targetCoords != null
                && Double.isFinite(robotX)
                && Double.isFinite(robotY)
                && Double.isFinite(robotHeading)) {

            double deltaX =
                    targetCoords[0] - robotX;

            double deltaY =
                    targetCoords[1] - robotY;

            double distance =
                    Math.hypot(deltaX, deltaY);

            if (distance > 0.01) {

                validPedroTarget = true;

                double fieldAngle =
                        Math.toDegrees(
                                Math.atan2(deltaY, deltaX)
                        );

                double relativeAngle =
                        TURRET_DIRECTION_SIGN *
                                (fieldAngle - robotHeading
                                        - TURRET_FIELD_ZERO_DEG);

                pedroTarget = relativeAngle;

                // =====================================
                // VELOCIDAD ANGULAR DEL CHASIS
                // =====================================

                if (Double.isFinite(lastRobotHeading)) {

                    double headingDelta =
                            wrap180(
                                    robotHeading
                                            - lastRobotHeading
                            );

                    double rawAngularVelocity =
                            headingDelta / dt;

                    rawAngularVelocity = Range.clip(
                            rawAngularVelocity,
                            -MAX_CHASSIS_VELOCITY_DEG_S,
                            MAX_CHASSIS_VELOCITY_DEG_S
                    );

                    filteredChassisAngularVelocity =
                            (1.0 - CHASSIS_VELOCITY_FILTER)
                                    * filteredChassisAngularVelocity
                                    + CHASSIS_VELOCITY_FILTER
                                    * rawAngularVelocity;
                }

                lastRobotHeading = robotHeading;

                velocityFeedforward =
                        -TURRET_DIRECTION_SIGN *
                                filteredChassisAngularVelocity;
            }

        } else {

            lastRobotHeading = Double.NaN;

            filteredChassisAngularVelocity = 0.0;
        }

        // =========================================
        // SELECCIONAR MODO DE APUNTADO
        // =========================================

        double desiredTarget = currentTurretAngle;

        switch (mode) {

            case PEDRO_ONLY:

                if (!validPedroTarget) {

                    status = "NO_PEDRO_TARGET";
                    holdCurrentTarget();
                    return;
                }

                visionOffset = 0.0;

                desiredTarget = pedroTarget;

                status = "PEDRO_TRACKING";
                break;

            case VISION_ONLY:

                velocityFeedforward = 0.0;
                visionOffset = 0.0;

                if (!validVisionTarget) {

                    status = "NO_VISION_TARGET";
                    holdCurrentTarget();
                    return;
                }

                double txVision = vision.getTx();

                if (Math.abs(txVision) > TX_DEADBAND) {

                    desiredTarget =
                            currentTurretAngle
                                    + TX_SIGN
                                    * txVision
                                    * VISION_ONLY_KP;

                    status = "VISION_TRACKING";

                } else {

                    desiredTarget = currentTurretAngle;

                    status = "VISION_CENTERED";
                }
                break;

            case COMBINED:

            default:

                if (!validPedroTarget) {

                    status = "NO_PEDRO_TARGET";
                    holdCurrentTarget();
                    return;
                }

                if (validVisionTarget) {

                    double tx = vision.getTx();

                    if (Math.abs(tx) > TX_DEADBAND) {

                        double step =
                                Math.signum(tx)
                                        * VISION_OFFSET_SPEED
                                        * dt;

                        visionOffset +=
                                TX_SIGN * step;

                        visionOffset = Range.clip(
                                visionOffset,
                                -MAX_VISION_OFFSET,
                                MAX_VISION_OFFSET
                        );

                        status = "COMBINED_CORRECTING";

                    } else {

                        status = "COMBINED_CENTERED";
                    }

                } else {

                    status = "PEDRO_FALLBACK";
                }

                desiredTarget =
                        pedroTarget + visionOffset;
                break;
        }

        // =========================================
        // BUSCAR ANGULO SEGURO
        // =========================================

        double safeTarget =
                findSafeAngle(
                        desiredTarget,
                        currentTurretAngle
                );

        if (!Double.isFinite(safeTarget)) {

            status = "NO_SAFE_ANGLE";

            holdCurrentTarget();
            return;
        }

        // =========================================
        // LIMITAR CAMBIO BRUSCO DE OBJETIVO
        // =========================================

        double smoothTarget =
                limitTargetRate(
                        safeTarget,
                        dt
                );

        smoothTarget = Range.clip(
                smoothTarget,
                TurretSubsystemV4.MIN_ANGLE_DEG,
                TurretSubsystemV4.MAX_ANGLE_DEG
        );

        // =========================================
        // APLICAR OBJETIVO
        // =========================================

        baseTarget = smoothTarget;

        turret.setVelocityFeedforward(
                velocityFeedforward
        );

        turret.setTarget(baseTarget);

        turret.update();
    }

    // =====================================================
    // MANTENER POSICION
    // =====================================================

    private void holdCurrentTarget() {

        turret.setVelocityFeedforward(0);

        // Mantener el ultimo objetivo seguro,
        // sin perseguir una medicion invalida.
        double holdTarget = turret.getTarget();

        if (!Double.isFinite(holdTarget)) {
            holdTarget = turret.getAngle();
        }

        holdTarget = Range.clip(
                holdTarget,
                TurretSubsystemV4.MIN_ANGLE_DEG,
                TurretSubsystemV4.MAX_ANGLE_DEG
        );

        lastCommandTarget = holdTarget;
        baseTarget = holdTarget;

        turret.setTarget(holdTarget);
        turret.update();
    }

    // =====================================================
    // GETTERS
    // =====================================================

    public boolean isEnabled() {
        return enabled;
    }

    public String getStatus() {
        return status;
    }

    public double getBaseTarget() {
        return baseTarget;
    }

    public double getVisionOffset() {
        return visionOffset;
    }

    public int getSelectedTag() {
        return selectedTag;
    }

    public boolean hasValidPedroTarget() {
        return validPedroTarget;
    }

    public boolean hasValidVisionTarget() {
        return validVisionTarget;
    }

    public double getChassisAngularVelocity() {
        return filteredChassisAngularVelocity;
    }
}

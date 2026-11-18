package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

public class TurretAimControllerV77 {

    public enum Mode {
        COMBINED,
        PEDRO_ONLY,
        VISION_ONLY
    }

    private final DriveSubsystem drive;
    private final TurretSubsystemV4 turret;
    private final TurretVisionV5 vision;

    // ==========================================
    // APRILTAGS ALIANZA ROJA
    // ==========================================
    public static double TAG37_X = 60.897345045022966;
    public static double TAG37_Y = 62.16307045879971;

    public static double TAG34_X = 50.71;
    public static double TAG34_Y = 56.15;

    public static double TAG30_X = 62.96;
    public static double TAG30_Y = 84.29;

    public static double TAG33_X = 50.33;
    public static double TAG33_Y = 82.95;

    // ==========================================
    // APRILTAGS ALIANZA AZUL
    // ==========================================
    public static double TAG41_X = 89.95;
    public static double TAG41_Y = 57.11;

    public static double TAG38_X = 78.08;
    public static double TAG38_Y = 55.58;

    public static double TAG42_X = 89.00;
    public static double TAG42_Y = 82.57;

    public static double TAG45_X = 77.89;
    public static double TAG45_Y = 81.42;

    // ==========================================
    // OFFSET ANGULAR PARA ESQUINA HIGH
    // ==========================================
    public static double TAG37_AIM_OFFSET_DEG = 0.0;
    public static double TAG34_AIM_OFFSET_DEG = 0.0;
    public static double TAG30_AIM_OFFSET_DEG = 0.0;
    public static double TAG33_AIM_OFFSET_DEG = 0.0;

    public static double TAG41_AIM_OFFSET_DEG = 0.0;
    public static double TAG38_AIM_OFFSET_DEG = 0.0;
    public static double TAG42_AIM_OFFSET_DEG = 0.0;
    public static double TAG45_AIM_OFFSET_DEG = 0.0;

    // ==========================================
    // AJUSTES DE CONTROL
    // ==========================================
    public static double TURRET_DIRECTION_SIGN = -1.0;
    public static double TX_SIGN = -1.0;

    public static double TURRET_FIELD_ZERO_DEG = 0.0;

    public static double TX_DEADBAND = 1.5;
    public static double VISION_OFFSET_SPEED = 4.5;
    public static double MAX_VISION_OFFSET = 20.0;

    public static double VISION_ONLY_KP = 0.60;
    public static double MAX_VISION_AGE_MS = 250.0;

    public static double MAX_TARGET_RATE_DEG_S = 500.0;

    private Mode mode = Mode.COMBINED;

    private boolean enabled = false;
    private String status = "DISABLED";

    private int selectedTag = -1;

    private double baseTarget = 0.0;
    private double visionOffset = 0.0;

    private double lastCommandTarget = Double.NaN;
    private double lastHeading = Double.NaN;
    private double filteredHeadingVelocity = 0.0;

    private long lastUpdateNs = 0;

    private boolean validPedroTarget = false;
    private boolean validVisionTarget = false;

    public TurretAimControllerV77(
            DriveSubsystem drive,
            TurretSubsystemV4 turret,
            TurretVisionV5 vision) {

        this.drive = drive;
        this.turret = turret;
        this.vision = vision;
    }

    // ==========================================
    // COORDENADAS Y OFFSET POR TAG
    // ==========================================

    private double[] targetFor(int id) {
        switch (id) {
            case 30: return new double[]{TAG30_X, TAG30_Y};
            case 33: return new double[]{TAG33_X, TAG33_Y};
            case 34: return new double[]{TAG34_X, TAG34_Y};
            case 37: return new double[]{TAG37_X, TAG37_Y};
            case 38: return new double[]{TAG38_X, TAG38_Y};
            case 41: return new double[]{TAG41_X, TAG41_Y};
            case 42: return new double[]{TAG42_X, TAG42_Y};
            case 45: return new double[]{TAG45_X, TAG45_Y};
            default: return null;
        }
    }

    private double getAimOffsetFor(int id) {
        switch (id) {
            case 37: return TAG37_AIM_OFFSET_DEG;
            case 34: return TAG34_AIM_OFFSET_DEG;
            case 30: return TAG30_AIM_OFFSET_DEG;
            case 33: return TAG33_AIM_OFFSET_DEG;
            case 41: return TAG41_AIM_OFFSET_DEG;
            case 38: return TAG38_AIM_OFFSET_DEG;
            case 42: return TAG42_AIM_OFFSET_DEG;
            case 45: return TAG45_AIM_OFFSET_DEG;
            default: return 0.0;
        }
    }

    // ==========================================
    // MODOS
    // ==========================================

    public void setMode(Mode newMode) {
        if (newMode == null || newMode == mode) {
            return;
        }
        mode = newMode;
        if (enabled) {
            resetTracking();
        }
    }

    public Mode getMode() {
        return mode;
    }

    // ==========================================
    // ACTIVAR / DESACTIVAR
    // ==========================================

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

    private void resetTracking() {
        baseTarget = turret.getAngle();
        visionOffset = 0.0;

        selectedTag = vision.getSelectedTag();

        lastHeading = Double.NaN;
        filteredHeadingVelocity = 0.0;

        lastCommandTarget = turret.getAngle();
        lastUpdateNs = System.nanoTime();

        validPedroTarget = false;
        validVisionTarget = false;

        status = "READY";
    }

    public void disable() {
        enabled = false;
        turret.setVelocityFeedforward(0);
        turret.disable();
        status = "DISABLED";
    }

    // ==========================================
    // ANGULOS Y VISION
    // ==========================================

    private double wrap180(double angle) {
        angle %= 360.0;
        if (angle > 180.0) angle -= 360.0;
        if (angle < -180.0) angle += 360.0;
        return angle;
    }

    private double safeEquivalent(double desired, double current) {
        double min = TurretSubsystemV4.MIN_ANGLE_DEG;
        double max = TurretSubsystemV4.MAX_ANGLE_DEG;

        double best = Double.NaN;
        double bestDistance = Double.POSITIVE_INFINITY;

        for (int k = -2; k <= 2; k++) {
            double candidate = desired + 360.0 * k;
            if (candidate < min || candidate > max) {
                continue;
            }
            double distance = Math.abs(candidate - current);
            if (distance < bestDistance) {
                bestDistance = distance;
                best = candidate;
            }
        }
        return best;
    }

    private boolean usableVision() {
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

    // ==========================================
    // UPDATE
    // ==========================================

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
        double dt = (now - lastUpdateNs) * 1e-9;
        lastUpdateNs = now;

        if (!Double.isFinite(dt) || dt < 0.001 || dt > 0.15) {
            dt = 0.02;
        }

        int currentTag = vision.getSelectedTag();
        if (currentTag != selectedTag) {
            selectedTag = currentTag;
            visionOffset = 0.0;
            lastHeading = Double.NaN;
            filteredHeadingVelocity = 0.0;
            lastCommandTarget = turret.getAngle();
        }

        double robotX = drive.getX();
        double robotY = drive.getY();
        double heading = drive.getHeadingDegrees();
        double currentAngle = turret.getAngle();

        double[] target = targetFor(selectedTag);
        double aimOffsetDeg = getAimOffsetFor(selectedTag);

        validPedroTarget = false;
        validVisionTarget = usableVision();

        double pedroTarget = currentAngle;
        double velocityFF = 0.0;

        // PEDRO
        if (target != null
                && Double.isFinite(robotX)
                && Double.isFinite(robotY)
                && Double.isFinite(heading)) {

            double dx = target[0] - robotX;
            double dy = target[1] - robotY;

            if (Math.hypot(dx, dy) > 0.01) {
                validPedroTarget = true;
                double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));

                pedroTarget = TURRET_DIRECTION_SIGN *
                        (fieldAngle - heading - TURRET_FIELD_ZERO_DEG)
                        + aimOffsetDeg;

                if (Double.isFinite(lastHeading)) {
                    double headingDelta = wrap180(heading - lastHeading);
                    double headingVelocity = Range.clip(headingDelta / dt, -500, 500);
                    filteredHeadingVelocity = 0.7 * filteredHeadingVelocity + 0.3 * headingVelocity;
                }

                lastHeading = heading;
                velocityFF = -TURRET_DIRECTION_SIGN * filteredHeadingVelocity;
            }
        }

        // MODO
        double desired = currentAngle;

        switch (mode) {
            case PEDRO_ONLY:
                if (!validPedroTarget) {
                    status = "NO_PEDRO_TARGET";
                    holdTarget();
                    return;
                }
                visionOffset = 0;
                desired = pedroTarget;
                status = "PEDRO_TRACKING";
                break;

            case VISION_ONLY:
                velocityFF = 0;
                visionOffset = 0;

                if (!validVisionTarget) {
                    status = "NO_VISION_TARGET";
                    holdTarget();
                    return;
                }

                double visionTx = vision.getTx();
                if (Math.abs(visionTx) > TX_DEADBAND) {
                    desired = currentAngle + TX_SIGN * visionTx * VISION_ONLY_KP + aimOffsetDeg;
                    status = "VISION_TRACKING";
                } else {
                    desired = currentAngle;
                    status = "VISION_CENTERED";
                }
                break;

            case COMBINED:
            default:
                if (!validPedroTarget) {
                    status = "NO_PEDRO_TARGET";
                    holdTarget();
                    return;
                }

                if (validVisionTarget) {
                    double tx = vision.getTx();
                    if (Math.abs(tx) > TX_DEADBAND) {
                        double stepSpeed = Range.clip(Math.abs(tx) * 0.8, 1.5, 12.0);
                        visionOffset += TX_SIGN * Math.signum(tx) * stepSpeed * dt;
                        visionOffset = Range.clip(visionOffset, -MAX_VISION_OFFSET, MAX_VISION_OFFSET);
                        status = "COMBINED_CORRECTING";
                    } else {
                        status = "COMBINED_CENTERED";
                    }
                } else {
                    status = "PEDRO_FALLBACK";
                }

                desired = pedroTarget + visionOffset;
                break;
        }

        // LIMITES MECANICOS
        double safe = safeEquivalent(desired, currentAngle);
        if (!Double.isFinite(safe)) {
            status = "NO_SAFE_ANGLE";
            holdTarget();
            return;
        }

        if (!Double.isFinite(lastCommandTarget)) {
            lastCommandTarget = currentAngle;
        }

        double maxStep = MAX_TARGET_RATE_DEG_S * dt;
        lastCommandTarget += Range.clip(safe - lastCommandTarget, -maxStep, maxStep);

        baseTarget = Range.clip(
                lastCommandTarget,
                TurretSubsystemV4.MIN_ANGLE_DEG,
                TurretSubsystemV4.MAX_ANGLE_DEG
        );

        turret.setVelocityFeedforward(velocityFF);
        turret.setTarget(baseTarget);
        turret.update();
    }

    private void holdTarget() {
        turret.setVelocityFeedforward(0);
        double target = turret.getTarget();

        if (!Double.isFinite(target)) {
            target = turret.getAngle();
        }

        target = Range.clip(
                target,
                TurretSubsystemV4.MIN_ANGLE_DEG,
                TurretSubsystemV4.MAX_ANGLE_DEG
        );

        baseTarget = target;
        lastCommandTarget = target;

        turret.setTarget(target);
        turret.update();
    }

    // ==========================================
    // GETTERS
    // ==========================================

    public boolean isEnabled() { return enabled; }
    public String getStatus() { return status; }
    public double getBaseTarget() { return baseTarget; }
    public double getVisionOffset() { return visionOffset; }
    public int getSelectedTag() { return selectedTag; }
    public boolean hasValidPedroTarget() { return validPedroTarget; }
    public boolean hasValidVisionTarget() { return validVisionTarget; }
    public double getChassisAngularVelocity() { return filteredHeadingVelocity; }
}
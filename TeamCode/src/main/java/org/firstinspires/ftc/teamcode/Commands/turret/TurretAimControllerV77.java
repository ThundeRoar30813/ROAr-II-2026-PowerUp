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
    // APRILTAGS
    // ==========================================

    public static double TAG30_X = 63.0;
    public static double TAG30_Y = 83.0;

    public static double TAG33_X = 51.0;
    public static double TAG33_Y = 84.0;

    public static double TAG34_X = 51.0;
    public static double TAG34_Y = 57.0;

    public static double TAG35_X = 55.0;
    public static double TAG35_Y = 57.0;

    public static double TAG37_X = 56.897345045022966;
    public static double TAG37_Y = 57.16307045879971;

    // ==========================================
    // AJUSTES ORIGINALES
    // ==========================================

    public static double TURRET_DIRECTION_SIGN = -1.0;
    public static double TX_SIGN = -1.0;

    public static double TURRET_FIELD_ZERO_DEG = 0.0;

    public static double TX_DEADBAND = 1.5;
    public static double VISION_OFFSET_SPEED = 1.5;
    public static double MAX_VISION_OFFSET = 12.0;

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
    // COORDENADAS
    // ==========================================

    private double[] targetFor(int id) {

        switch (id) {

            case 30:
                return new double[]{TAG30_X, TAG30_Y};

            case 33:
                return new double[]{TAG33_X, TAG33_Y};

            case 34:
                return new double[]{TAG34_X, TAG34_Y};

            case 35:
                return new double[]{TAG35_X, TAG35_Y};

            case 37:
                return new double[]{TAG37_X, TAG37_Y};

            default:
                return null;
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
    // ACTIVAR
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

    // ==========================================
    // DESACTIVAR
    // ==========================================

    public void disable() {

        enabled = false;

        turret.setVelocityFeedforward(0);
        turret.disable();

        status = "DISABLED";
    }

    // ==========================================
    // ANGULOS
    // ==========================================

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

    private double safeEquivalent(
            double desired,
            double current) {

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

        if (!Double.isFinite(dt)
                || dt < 0.001
                || dt > 0.15) {

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

        validPedroTarget = false;
        validVisionTarget = usableVision();

        double pedroTarget = currentAngle;
        double velocityFF = 0.0;

        // ======================================
        // PEDRO
        // ======================================

        if (target != null
                && Double.isFinite(robotX)
                && Double.isFinite(robotY)
                && Double.isFinite(heading)) {

            double dx = target[0] - robotX;
            double dy = target[1] - robotY;

            if (Math.hypot(dx, dy) > 0.01) {

                validPedroTarget = true;

                double fieldAngle =
                        Math.toDegrees(Math.atan2(dy, dx));

                pedroTarget =
                        TURRET_DIRECTION_SIGN *
                                (fieldAngle
                                        - heading
                                        - TURRET_FIELD_ZERO_DEG);

                if (Double.isFinite(lastHeading)) {

                    double headingDelta =
                            wrap180(heading - lastHeading);

                    double headingVelocity =
                            Range.clip(
                                    headingDelta / dt,
                                    -500,
                                    500
                            );

                    filteredHeadingVelocity =
                            0.7 * filteredHeadingVelocity
                                    + 0.3 * headingVelocity;
                }

                lastHeading = heading;

                velocityFF =
                        -TURRET_DIRECTION_SIGN
                                * filteredHeadingVelocity;
            }
        }

        // ======================================
        // MODO
        // ======================================

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

                    desired =
                            currentAngle
                                    + TX_SIGN
                                    * visionTx
                                    * VISION_ONLY_KP;

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

                        visionOffset +=
                                TX_SIGN
                                        * Math.signum(tx)
                                        * VISION_OFFSET_SPEED
                                        * dt;

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

                desired = pedroTarget + visionOffset;
                break;
        }

        // ======================================
        // LIMITES MECANICOS
        // ======================================

        double safe = safeEquivalent(
                desired,
                currentAngle
        );

        if (!Double.isFinite(safe)) {

            status = "NO_SAFE_ANGLE";
            holdTarget();
            return;
        }

        if (!Double.isFinite(lastCommandTarget)) {
            lastCommandTarget = currentAngle;
        }

        double maxStep = MAX_TARGET_RATE_DEG_S * dt;

        lastCommandTarget += Range.clip(
                safe - lastCommandTarget,
                -maxStep,
                maxStep
        );

        baseTarget = Range.clip(
                lastCommandTarget,
                TurretSubsystemV4.MIN_ANGLE_DEG,
                TurretSubsystemV4.MAX_ANGLE_DEG
        );

        turret.setVelocityFeedforward(velocityFF);
        turret.setTarget(baseTarget);
        turret.update();
    }

    // ==========================================
    // HOLD
    // ==========================================

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
        return filteredHeadingVelocity;
    }
}
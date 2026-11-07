package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV3;

public class TurretAimingV9 {

    private final DriveSubsystem drive;
    private final TurretSubsystemV3 turret;
    private final TurretVisionV4 vision;

    // Signos del V9 que funcionaba
    public static double HEADING_SIGN = 1.0;
    public static double VISION_SIGN = -1.0;
    public static double BEARING_SIGN = -1.0;

    public static double MAX_TARGET_RATE = 600.0;
    public static double MAX_VISION_RATE = 160.0;
    public static double MAX_VISION_OFFSET = 20.0;
    public static double MAX_ACCEPTABLE_TX = 25.0;
    public static double MAX_STEP_DEG = 25.0;

    public static double KP_STILL = 1.5;
    public static double KP_MOVING = 7.0;

    public static double TX_DEADBAND_STILL = 1.2;
    public static double TX_DEADBAND_MOVING = 0.45;
    public static double TX_FILTER_ALPHA = 0.65;
    public static double MOVE_THRESHOLD_IN_S = 2.0;
    public static double VISION_RATE_SMOOTH = 0.40;

    public static double PREDICT_SECONDS = 0.14;
    public static double MAX_PREDICT_SECONDS = 0.16;
    public static double VELOCITY_FILTER_ALPHA = 0.70;
    public static double MAX_PLAUSIBLE_SPEED_IN_S = 90.0;

    public static double MIN_LOCK_DISTANCE_IN = 12.0;
    public static double MAX_LOCK_DISTANCE_IN = 180.0;

    // Limites provisionales de seguridad
    public static double SAFE_MIN = -100.0;
    public static double SAFE_MAX = 220.0;

    private boolean enabled = false;
    private boolean visionEnabled = false;
    private boolean locked = false;
    private boolean atLimit = false;

    private double targetX;
    private double targetY;

    private double referenceHeading;
    private double referenceTurret;
    private double referenceBearing;

    private double lastHeading;
    private double lastX;
    private double lastY;

    private double vx;
    private double vy;
    private double speed;

    private double desiredAngle;
    private double commandedAngle;
    private double visionOffset;

    private double filteredTx;
    private double visionRate;
    private double predictionDegrees;

    private boolean txInitialized = false;
    private long lastTime;

    private String status = "OFF";

    public TurretAimingV9(
            DriveSubsystem drive,
            TurretSubsystemV3 turret,
            TurretVisionV4 vision) {

        this.drive = drive;
        this.turret = turret;
        this.vision = vision;
    }

    public void enable() {

        if (turret.isStalled()) {
            status = "TURRET_FAULT";
            return;
        }

        referenceHeading = drive.getHeadingDegrees();
        lastHeading = referenceHeading;

        referenceTurret = turret.getAngle();

        lastX = drive.getX();
        lastY = drive.getY();

        vx = 0;
        vy = 0;
        speed = 0;
        predictionDegrees = 0;

        desiredAngle = referenceTurret;
        commandedAngle = referenceTurret;

        visionOffset = 0;
        filteredTx = 0;
        visionRate = 0;

        txInitialized = false;
        locked = false;
        atLimit = false;

        lastTime = System.nanoTime();

        turret.enable();
        turret.setTarget(referenceTurret);

        enabled = turret.isEnabled();

        status = enabled
                ? "WAITING_FOR_TAG_36"
                : "TURRET_DISABLED";
    }

    public void disable() {

        enabled = false;
        locked = false;
        atLimit = false;

        turret.disable();
        status = "OFF";
    }

    public void setVisionEnabled(boolean value) {

        visionEnabled = value;
        txInitialized = false;
        visionRate = 0;
    }

    public boolean acquireTarget(double distanceInches) {

        if (!enabled
                || !visionEnabled
                || !vision.hasTarget()
                || !Double.isFinite(distanceInches)
                || distanceInches < MIN_LOCK_DISTANCE_IN
                || distanceInches > MAX_LOCK_DISTANCE_IN
                || !Double.isFinite(vision.getTx())
                || Math.abs(vision.getTx())
                > MAX_ACCEPTABLE_TX) {

            status = "ACQUIRE_NEEDS_TAG_AND_DISTANCE";
            return false;
        }

        double cameraBearingDeg =
                drive.getHeadingDegrees()
                        - HEADING_SIGN * (
                        turret.getAngle()
                                + VISION_SIGN * vision.getTx()
                );

        double radians = Math.toRadians(cameraBearingDeg);

        targetX = drive.getX()
                + distanceInches * Math.cos(radians);

        targetY = drive.getY()
                + distanceInches * Math.sin(radians);

        referenceHeading = drive.getHeadingDegrees();
        referenceTurret = turret.getAngle();

        referenceBearing = Math.toDegrees(
                Math.atan2(
                        targetY - drive.getY(),
                        targetX - drive.getX()
                )
        );

        lastX = drive.getX();
        lastY = drive.getY();

        vx = 0;
        vy = 0;
        speed = 0;

        predictionDegrees = 0;
        visionOffset = 0;
        visionRate = 0;

        txInitialized = false;
        locked = true;
        atLimit = false;

        status = "LOCKED_ESTIMATED_TAG_36";

        return true;
    }

    public void update() {

        if (!enabled) return;

        if (!turret.isEnabled() || turret.isStalled()) {
            disable();
            status = "TURRET_FAULT_OR_DISABLED";
            return;
        }

        long now = System.nanoTime();

        double dt = (now - lastTime) * 1e-9;
        lastTime = now;

        if (!Double.isFinite(dt)
                || dt <= 0
                || dt > 0.12) {
            dt = 0.02;
        }

        double heading = drive.getHeadingDegrees();
        double x = drive.getX();
        double y = drive.getY();

        if (!Double.isFinite(heading)
                || !Double.isFinite(x)
                || !Double.isFinite(y)) {

            disable();
            status = "INVALID_POSE";
            return;
        }

        double headingStep = wrap(
                heading - lastHeading
        );

        lastHeading = heading;

        if (Math.abs(headingStep) > MAX_STEP_DEG) {
            disable();
            status = "HEADING_JUMP";
            return;
        }

        // Velocidad del robot
        double rawVx = (x - lastX) / dt;
        double rawVy = (y - lastY) / dt;

        lastX = x;
        lastY = y;

        if (Math.hypot(rawVx, rawVy)
                > MAX_PLAUSIBLE_SPEED_IN_S) {
            rawVx = vx;
            rawVy = vy;
        }

        vx += VELOCITY_FILTER_ALPHA * (rawVx - vx);
        vy += VELOCITY_FILTER_ALPHA * (rawVy - vy);

        speed = Math.hypot(vx, vy);

        boolean moving =
                speed >= MOVE_THRESHOLD_IN_S;

        double headingDelta = wrap(
                heading - referenceHeading
        );

        double bearingDelta = 0;
        predictionDegrees = 0;

        if (locked) {

            double dx = targetX - x;
            double dy = targetY - y;

            if (dx * dx + dy * dy < 16) {
                disable();
                status = "TARGET_TOO_CLOSE";
                return;
            }

            double currentBearing = Math.toDegrees(
                    Math.atan2(dy, dx)
            );

            bearingDelta = wrap(
                    currentBearing - referenceBearing
            );

            double predictionTime = Range.clip(
                    PREDICT_SECONDS,
                    0,
                    MAX_PREDICT_SECONDS
            );

            double predictedBearing = Math.toDegrees(
                    Math.atan2(
                            targetY - (y + vy * predictionTime),
                            targetX - (x + vx * predictionTime)
                    )
            );

            predictionDegrees = wrap(
                    predictedBearing - currentBearing
            );

            predictionDegrees = Range.clip(
                    predictionDegrees,
                    -6.0,
                    6.0
            );
        }

        // Correccion Limelight original V9
        if (visionEnabled
                && vision.hasTarget()
                && Double.isFinite(vision.getTx())
                && Math.abs(vision.getTx())
                <= MAX_ACCEPTABLE_TX) {

            double tx = vision.getTx();

            if (!txInitialized) {
                filteredTx = tx;
                txInitialized = true;
            } else {
                filteredTx += TX_FILTER_ALPHA
                        * (tx - filteredTx);
            }

            double deadband = moving
                    ? TX_DEADBAND_MOVING
                    : TX_DEADBAND_STILL;

            double kp = moving
                    ? KP_MOVING
                    : KP_STILL;

            double error =
                    Math.abs(filteredTx) <= deadband
                            ? 0
                            : filteredTx;

            double requestedRate = Range.clip(
                    VISION_SIGN * kp * error,
                    -MAX_VISION_RATE,
                    MAX_VISION_RATE
            );

            visionRate += VISION_RATE_SMOOTH
                    * (requestedRate - visionRate);

            visionOffset = Range.clip(
                    visionOffset + visionRate * dt,
                    -MAX_VISION_OFFSET,
                    MAX_VISION_OFFSET
            );

        } else {
            txInitialized = false;
            visionRate = 0;
        }

        // Objetivo geometrico original
        double rawDesired =
                referenceTurret
                        + HEADING_SIGN * headingDelta
                        + BEARING_SIGN * (
                        bearingDelta + predictionDegrees
                )
                        + visionOffset;

        if (!Double.isFinite(rawDesired)) {
            disable();
            status = "INVALID_TARGET";
            return;
        }

        // =================================================
        // RECUPERACION SEGURA DE LIMITES
        // =================================================

        double minAngle = Math.max(
                SAFE_MIN,
                TurretSubsystemV3.MIN_ANGLE
        );

        double maxAngle = Math.min(
                SAFE_MAX,
                TurretSubsystemV3.MAX_ANGLE
        );

        if (minAngle >= maxAngle) {
            disable();
            status = "INVALID_LIMITS";
            return;
        }

        // Busca angulos equivalentes dentro del recorrido.
        // Nunca reinicia el encoder.
        double selected = Double.NaN;
        double smallestMove = Double.POSITIVE_INFINITY;

        for (int turns = -2; turns <= 2; turns++) {

            double candidate =
                    rawDesired + 360.0 * turns;

            if (candidate < minAngle
                    || candidate > maxAngle) {
                continue;
            }

            double movement = Math.abs(
                    candidate - turret.getAngle()
            );

            if (movement < smallestMove) {
                smallestMove = movement;
                selected = candidate;
            }
        }

        if (Double.isFinite(selected)) {

            desiredAngle = selected;
            atLimit = false;

            status = locked
                    ? "TRACKING"
                    : "WAITING_FOR_TAG_36";

        } else {

            // Mantiene seguimiento activo.
            // Espera hasta que el objetivo sea alcanzable.
            desiredAngle = Range.clip(
                    rawDesired,
                    minAngle,
                    maxAngle
            );

            atLimit = true;
            status = "LIMIT_WAITING_FOR_RECOVERY";
        }

        double maxStep = MAX_TARGET_RATE * dt;

        commandedAngle += Range.clip(
                desiredAngle - commandedAngle,
                -maxStep,
                maxStep
        );

        commandedAngle = Range.clip(
                commandedAngle,
                minAngle,
                maxAngle
        );

        turret.setTarget(commandedAngle);
        turret.update();

        if (turret.isStalled()) {
            disable();
            status = "TURRET_FAULT";
        }
    }

    private static double wrap(double degrees) {

        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;

        return degrees;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isVisionEnabled() {
        return visionEnabled;
    }

    public boolean hasLock() {
        return locked;
    }

    public boolean isAtLimit() {
        return atLimit;
    }

    public String getStatus() {
        return status;
    }

    public double getTargetX() {
        return targetX;
    }

    public double getTargetY() {
        return targetY;
    }

    public double getDesiredAngle() {
        return desiredAngle;
    }

    public double getCommandedAngle() {
        return commandedAngle;
    }

    public double getVisionOffset() {
        return visionOffset;
    }

    public double getTx() {
        return filteredTx;
    }

    public double getSpeed() {
        return speed;
    }

    public double getVx() {
        return vx;
    }

    public double getVy() {
        return vy;
    }

    public double getPredictionDegrees() {
        return predictionDegrees;
    }

    public double getVisionRate() {
        return visionRate;
    }

    public double getDistance() {

        return locked
                ? Math.hypot(
                targetX - drive.getX(),
                targetY - drive.getY()
        )
                : Double.NaN;
    }
}
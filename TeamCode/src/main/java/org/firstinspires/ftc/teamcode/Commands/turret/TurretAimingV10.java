
package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV3;

public class TurretAimingV10 {

    private final DriveSubsystem drive;
    private final TurretSubsystemV3 turret;
    private final TurretVisionV4 vision;

    public static double HEADING_SIGN = +1.0;
    public static double VISION_SIGN = -1.0;
    public static double BEARING_SIGN = -1.0;

    public static double MAX_TARGET_RATE = 600.0;
    public static double MAX_VISION_RATE = 160.0;
    public static double MAX_VISION_OFFSET = 16.0;

    public static double VISION_P_MOVING = 0.35;
    public static double VISION_P_STILL = 0.20;
    public static double VISION_I_MOVING = 1.10;
    public static double VISION_I_STILL = 0.30;

    public static double MAX_INTEGRAL_OFFSET = 10.0;
    public static double MAX_DIRECT_OFFSET = 6.0;
    public static double MAX_ACCEPTABLE_TX = 25.0;
    public static double MAX_STEP_DEG = 25.0;

    public static double KP_STILL = 1.5;
    public static double KP_MOVING = 7.0;
    public static double TX_DEADBAND_STILL = 1.2;
    public static double TX_DEADBAND_MOVING = 0.45;
    public static double TX_FILTER_ALPHA = 0.80;
    public static double MOVE_THRESHOLD_IN_S = 1.0;
    public static double VISION_RATE_SMOOTH = 0.40;

    public static double PREDICT_SECONDS = 0.12;
    public static double MAX_PREDICT_SECONDS = 0.14;
    public static double VELOCITY_FILTER_ALPHA = 0.85;
    public static double MAX_PLAUSIBLE_SPEED_IN_S = 90.0;

    public static double MIN_LOCK_DISTANCE_IN = 12.0;
    public static double MAX_LOCK_DISTANCE_IN = 180.0;

    private boolean enabled = false;
    private boolean visionEnabled = false;
    private boolean locked = false;

    private double targetX, targetY;
    private double referenceHeading;
    private double referenceTurret;
    private double referenceBearing;

    private double lastHeading, lastX, lastY;
    private double vx, vy, speed;
    private double desiredAngle, commandedAngle;
    private double visionOffset;
    private double filteredTx, visionRate;
    private double predictionDegrees;
    private double integralOffset, directOffset;

    private boolean txInitialized = false;
    private long lastTime;
    private String status = "OFF";

    public TurretAimingV10(
            DriveSubsystem drive,
            TurretSubsystemV3 turret,
            TurretVisionV4 vision) {

        this.drive = drive;
        this.turret = turret;
        this.vision = vision;
    }

    public void enable() {

        if (turret.isStalled()) return;

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
        integralOffset = 0;
        directOffset = 0;

        txInitialized = false;
        locked = false;

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
        turret.disable();
        status = "OFF";
    }

    public void setVisionEnabled(boolean value) {

        visionEnabled = value;
        txInitialized = false;
        visionRate = 0;
        integralOffset = 0;
        directOffset = 0;
        visionOffset = 0;
    }

    public boolean acquireTarget(double distanceInches) {

        if (!enabled
                || !visionEnabled
                || !vision.hasTarget()
                || !Double.isFinite(distanceInches)
                || distanceInches < MIN_LOCK_DISTANCE_IN
                || distanceInches > MAX_LOCK_DISTANCE_IN
                || !Double.isFinite(vision.getTx())
                || Math.abs(vision.getTx()) > MAX_ACCEPTABLE_TX) {

            status = "ACQUIRE_NEEDS_TAG_AND_MEASURED_DISTANCE";
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
        integralOffset = 0;
        directOffset = 0;

        txInitialized = false;
        locked = true;

        status = "LOCKED_ESTIMATED_TAG_36";

        return true;
    }

    public void update() {

        if (!enabled) return;

        if (!turret.isEnabled() || turret.isStalled()) {
            disable();
            return;
        }

        long now = System.nanoTime();
        double dt = (now - lastTime) * 1e-9;
        lastTime = now;

        if (!Double.isFinite(dt) || dt <= 0 || dt > 0.12) {
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

        double headingStep = wrap(heading - lastHeading);
        lastHeading = heading;

        if (Math.abs(headingStep) > MAX_STEP_DEG) {
            disable();
            status = "HEADING_JUMP";
            return;
        }

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

        boolean moving = speed >= MOVE_THRESHOLD_IN_S;

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

        // Correccion visual proporcional + integral limitada.
        if (visionEnabled
                && vision.hasTarget()
                && Double.isFinite(vision.getTx())
                && Math.abs(vision.getTx()) <= MAX_ACCEPTABLE_TX) {

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

            double error = Math.abs(filteredTx) <= deadband
                    ? 0
                    : filteredTx;

            double pGain = moving
                    ? VISION_P_MOVING
                    : VISION_P_STILL;

            double iGain = moving
                    ? VISION_I_MOVING
                    : VISION_I_STILL;

            directOffset = Range.clip(
                    VISION_SIGN * pGain * error,
                    -MAX_DIRECT_OFFSET,
                    MAX_DIRECT_OFFSET
            );

            double nextIntegral = integralOffset
                    + VISION_SIGN * iGain * error * dt;

            if (Math.abs(nextIntegral) <= MAX_INTEGRAL_OFFSET
                    || Math.abs(nextIntegral) < Math.abs(integralOffset)) {

                integralOffset = Range.clip(
                        nextIntegral,
                        -MAX_INTEGRAL_OFFSET,
                        MAX_INTEGRAL_OFFSET
                );
            }

            if (error == 0) {

                integralOffset *= Math.max(
                        0.0,
                        1.0 - 0.35 * dt
                );
            }

            visionOffset = Range.clip(
                    directOffset + integralOffset,
                    -MAX_VISION_OFFSET,
                    MAX_VISION_OFFSET
            );

            visionRate = VISION_SIGN * iGain * error;

        } else {

            txInitialized = false;
            directOffset = 0;
            visionRate = 0;

            integralOffset *= Math.max(
                    0.0,
                    1.0 - 1.5 * dt
            );

            visionOffset = integralOffset;
        }

        desiredAngle = referenceTurret
                + HEADING_SIGN * headingDelta
                + BEARING_SIGN * (
                bearingDelta + predictionDegrees
        )
                + visionOffset;

        if (!Double.isFinite(desiredAngle)
                || desiredAngle < TurretSubsystemV3.MIN_ANGLE
                || desiredAngle > TurretSubsystemV3.MAX_ANGLE) {

            disable();
            status = "SOFTWARE_ANGLE_LIMIT";
            return;
        }

        double maxStep = MAX_TARGET_RATE * dt;

        commandedAngle += Range.clip(
                desiredAngle - commandedAngle,
                -maxStep,
                maxStep
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

    public double getDirectOffset() {
        return directOffset;
    }

    public double getIntegralOffset() {
        return integralOffset;
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

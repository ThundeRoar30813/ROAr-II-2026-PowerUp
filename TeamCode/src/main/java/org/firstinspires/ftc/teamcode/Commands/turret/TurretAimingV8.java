package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV3;

/**
 * V8: visual acquisition followed by odometry feed-forward and bounded vision trim.
 * Coordinates are local Pedro coordinates (inches). No absolute field pose required.
 * Assumes turret axis coincides with the Pinpoint robot pose origin initially.
 */
public class TurretAimingV8 {
    private final DriveSubsystem drive;
    private final TurretSubsystemV3 turret;
    private final TurretVisionV4 vision;

    // Verify both signs in a low-power test before operating normally.
    public static double HEADING_SIGN = +1.0; // inherited from working V7
    public static double VISION_SIGN = -1.0;  // inherited from working V7
    public static double BEARING_SIGN = -1.0; // must be calibrated by strafing

    public static double MAX_TARGET_RATE = 350.0;
    public static double MAX_VISION_RATE = 100.0;
    public static double VISION_KP = 3.5; // deg/s per deg Tx
    public static double TX_DEADBAND = 0.6;
    public static double MAX_VISION_OFFSET = 20.0;
    public static double MAX_ACCEPTABLE_TX = 25.0;
    public static double MAX_STEP_DEG = 25.0;
    public static double MIN_LOCK_DISTANCE_IN = 12.0;
    public static double MAX_LOCK_DISTANCE_IN = 180.0;
    public static double INITIAL_DISTANCE_IN = 72.0; // measured distance, not a field coordinate

    private boolean enabled = false;
    private boolean visionEnabled = false;
    private boolean locked = false;
    private double targetX, targetY;
    private double referenceHeading, referenceTurret, referenceBearing;
    private double lastHeading;
    private double desiredAngle, commandedAngle, visionOffset;
    private double lastTx = 0;
    private long lastTime;
    private String status = "OFF";

    public TurretAimingV8(DriveSubsystem drive, TurretSubsystemV3 turret, TurretVisionV4 vision) {
        this.drive = drive;
        this.turret = turret;
        this.vision = vision;
    }

    public void enable() {
        if (turret.isStalled()) return;
        referenceHeading = drive.getHeadingDegrees();
        lastHeading = referenceHeading;
        referenceTurret = turret.getAngle();
        desiredAngle = commandedAngle = referenceTurret;
        visionOffset = 0;
        locked = false;
        lastTime = System.nanoTime();
        turret.enable();
        turret.setTarget(referenceTurret);
        enabled = turret.isEnabled();
        status = enabled ? "WAITING_FOR_TAG_36" : "TURRET_DISABLED";
    }

    public void disable() {
        enabled = false;
        locked = false;
        turret.disable();
        status = "OFF";
    }

    public void setVisionEnabled(boolean value) {
        visionEnabled = value;
        // Turning vision off retains the acquired geometric target.
    }

    /**
     * Acquire a local target estimate using a measured horizontal distance to tag 36.
     * Distance is NOT inferred from Tx/Ty. Align turret to tag before calling.
     * Assumes camera and turret forward axes have been aligned mechanically.
     */
    public boolean acquireTarget(double distanceInches) {
        if (!enabled || !visionEnabled || !vision.hasTarget() ||
                !Double.isFinite(distanceInches) ||
                distanceInches < MIN_LOCK_DISTANCE_IN ||
                distanceInches > MAX_LOCK_DISTANCE_IN ||
                Math.abs(vision.getTx()) > MAX_ACCEPTABLE_TX) {
            status = "ACQUIRE_REQUIRES_VALID_TAG_AND_DISTANCE";
            return false;
        }
        // Calibrate bearing by the observed camera direction. The sign of Tx
        // is inherited from V7: vision correction to turret is -Tx.
        double cameraBearingDeg = drive.getHeadingDegrees()
                - HEADING_SIGN * (turret.getAngle() + VISION_SIGN * vision.getTx());
        double rad = Math.toRadians(cameraBearingDeg);
        targetX = drive.getX() + distanceInches * Math.cos(rad);
        targetY = drive.getY() + distanceInches * Math.sin(rad);
        referenceHeading = drive.getHeadingDegrees();
        referenceTurret = turret.getAngle();
        referenceBearing = Math.toDegrees(Math.atan2(targetY - drive.getY(), targetX - drive.getX()));
        visionOffset = 0;
        locked = true;
        status = "LOCKED_TAG_36_ESTIMATE";
        return true;
    }

    public void update() {
        if (!enabled) return;
        if (!turret.isEnabled() || turret.isStalled()) { disable(); return; }
        long now = System.nanoTime();
        double dt = (now - lastTime) * 1e-9;
        lastTime = now;
        if (dt <= 0 || dt > 0.1) dt = 0.02;

        double heading = drive.getHeadingDegrees();
        double headingStep = wrap(heading - lastHeading);
        lastHeading = heading;
        if (!Double.isFinite(heading) || Math.abs(headingStep) > MAX_STEP_DEG) {
            disable(); status = "BAD_ODOMETRY_HEADING"; return;
        }

        double headingDelta = wrap(heading - referenceHeading);
        double bearingDelta = 0;
        if (locked) {
            double dx = targetX - drive.getX();
            double dy = targetY - drive.getY();
            if (!Double.isFinite(dx) || !Double.isFinite(dy) || dx * dx + dy * dy < 16) {
                disable(); status = "BAD_TARGET_GEOMETRY"; return;
            }
            double currentBearing = Math.toDegrees(Math.atan2(dy, dx));
            bearingDelta = wrap(currentBearing - referenceBearing);
        }

        // One angle command only. V7 heading sign retained; bearing sign needs verification.
        desiredAngle = referenceTurret + HEADING_SIGN * headingDelta
                + BEARING_SIGN * bearingDelta + visionOffset;

        if (visionEnabled && vision.hasTarget() && Double.isFinite(vision.getTx())) {
            lastTx = vision.getTx();
            if (Math.abs(lastTx) <= MAX_ACCEPTABLE_TX) {
                double error = Math.abs(lastTx) <= TX_DEADBAND ? 0 : lastTx;
                double correctionRate = Range.clip(VISION_SIGN * VISION_KP * error,
                        -MAX_VISION_RATE, MAX_VISION_RATE);
                visionOffset = Range.clip(visionOffset + correctionRate * dt,
                        -MAX_VISION_OFFSET, MAX_VISION_OFFSET);
            }
        }
        desiredAngle = referenceTurret + HEADING_SIGN * headingDelta
                + BEARING_SIGN * bearingDelta + visionOffset;

        if (!Double.isFinite(desiredAngle) || desiredAngle < TurretSubsystemV3.MIN_ANGLE
                || desiredAngle > TurretSubsystemV3.MAX_ANGLE) {
            disable(); status = "SOFTWARE_ANGLE_LIMIT"; return;
        }
        double maxStep = MAX_TARGET_RATE * dt;
        commandedAngle += Range.clip(desiredAngle - commandedAngle, -maxStep, maxStep);
        turret.setTarget(commandedAngle);
        turret.update();
        if (turret.isStalled()) { disable(); status = "TURRET_FAULT"; }
    }

    private static double wrap(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    public boolean isEnabled() { return enabled; }
    public boolean isVisionEnabled() { return visionEnabled; }
    public boolean hasLock() { return locked; }
    public String getStatus() { return status; }
    public double getTargetX() { return targetX; }
    public double getTargetY() { return targetY; }
    public double getDesiredAngle() { return desiredAngle; }
    public double getCommandedAngle() { return commandedAngle; }
    public double getVisionOffset() { return visionOffset; }
    public double getTx() { return lastTx; }
    public double getDistance() { return locked ? Math.hypot(targetX-drive.getX(),targetY-drive.getY()) : Double.NaN; }
}

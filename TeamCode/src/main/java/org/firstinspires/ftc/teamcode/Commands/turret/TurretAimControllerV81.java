
package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

/**
 * V81: Seguimiento geometrico con Pedro Pathing
 * y correccion visual limitada de Limelight.
 *
 * Experimental: conservar V75 como respaldo.
 */
public class TurretAimControllerV81 {

    public static double TAG34_X = 51.0;
    public static double TAG34_Y = 57.0;
    public static double TAG37_X = 62.0;
    public static double TAG37_Y = 57.0;

    public static double MIN_DISTANCE = 8.0;

    public static double HEADING_SIGN = 1.0;
    public static double TX_SIGN = -1.0;

    public static double VISION_GAIN = 0.60;
    public static double VISION_LIMIT_DEG = 16.0;
    public static double TX_DEADBAND = 0.8;
    public static double MAX_TX = 27.0;
    public static double MAX_AGE_MS = 150.0;

    public static double VISION_FILTER_S = 0.15;
    public static double MAX_VISION_STEP_DEG = 2.0;

    public static double MAX_TARGET_SPEED = 280.0;
    public static double MAX_TARGET_ACCEL = 1400.0;
    public static double MAX_GEOMETRY_JUMP = 25.0;

    private final DriveSubsystem drive;
    private final TurretSubsystemV4 turret;
    private final TurretVisionV5 vision;

    private boolean enabled = false;
    private boolean anchored = false;

    private int tag = -1;

    private double anchorTurret;
    private double anchorHeading;
    private double anchorBearing;

    private double geometryTarget;
    private double desiredTarget;
    private double smoothTarget;
    private double profileVelocity;

    private double visionOffset = 0.0;
    private double lastTx = Double.NaN;

    private long lastNs;
    private long lastFrame = -1;

    private String status = "OFF";

    public TurretAimControllerV81(
            DriveSubsystem drive,
            TurretSubsystemV4 turret,
            TurretVisionV5 vision) {

        this.drive = drive;
        this.turret = turret;
        this.vision = vision;
    }

    private static double delta(double a, double b) {

        double d = a - b;

        while (d > 180.0) d -= 360.0;
        while (d < -180.0) d += 360.0;

        return d;
    }

    private double bearing(int id) {

        double x;
        double y;

        if (id == 34) {
            x = TAG34_X;
            y = TAG34_Y;
        } else if (id == 37) {
            x = TAG37_X;
            y = TAG37_Y;
        } else {
            return Double.NaN;
        }

        double dx = x - drive.getX();
        double dy = y - drive.getY();

        if (!Double.isFinite(dx)
                || !Double.isFinite(dy)
                || Math.hypot(dx, dy) < MIN_DISTANCE) {

            return Double.NaN;
        }

        return Math.toDegrees(Math.atan2(dy, dx));
    }

    private void anchor() {

        tag = vision.getSelectedTag();

        anchorHeading = drive.getHeadingDegrees();
        anchorBearing = bearing(tag);
        anchorTurret = turret.getAngle();

        anchored =
                Double.isFinite(anchorHeading)
                        && Double.isFinite(anchorBearing)
                        && Double.isFinite(anchorTurret);

        geometryTarget = anchorTurret;
        desiredTarget = anchorTurret;
        smoothTarget = anchorTurret;

        profileVelocity = 0.0;
        visionOffset = 0.0;
        lastTx = Double.NaN;

        lastFrame = vision.getFrameSequence();
        lastNs = System.nanoTime();

        status = anchored
                ? "GEOMETRY_READY"
                : "BAD_PEDRO_POSE";
    }

    public void enable() {

        if (turret.isFault()) {
            status = "TURRET_FAULT";
            return;
        }

        turret.enableHold();

        if (!turret.isEnabled()) {
            status = "NOT_ENABLED";
            return;
        }

        enabled = true;
        anchor();

        if (!anchored) {
            disable();
        }
    }

    public void disable() {

        enabled = false;

        turret.setVelocityFeedforward(0.0);
        turret.disable();

        if (!status.equals("BAD_PEDRO_POSE")) {
            status = "OFF";
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    public String getStatus() {
        return status;
    }

    public double getGeometryTarget() {
        return geometryTarget;
    }

    public double getDesiredTarget() {
        return desiredTarget;
    }

    public double getSmoothTarget() {
        return smoothTarget;
    }

    public double getVisionOffset() {
        return visionOffset;
    }

    public double getLastTx() {
        return lastTx;
    }

    public void update() {

        if (!enabled) return;

        if (turret.isFault()) {
            status = "TURRET_FAULT";
            disable();
            return;
        }

        if (vision.getSelectedTag() != tag) {
            anchor();
        }

        long now = System.nanoTime();

        double dt = (now - lastNs) * 1e-9;
        lastNs = now;

        if (!Double.isFinite(dt)
                || dt <= 0.0
                || dt > 0.15) {

            turret.setVelocityFeedforward(0.0);
            turret.setTarget(smoothTarget);
            turret.update();

            status = "LOOP_TIMING";
            return;
        }

        // ======================================
        // 1. GEOMETRIA PEDRO PATHING
        // ======================================

        double heading = drive.getHeadingDegrees();
        double currentBearing = bearing(tag);

        if (!Double.isFinite(heading)
                || !Double.isFinite(currentBearing)) {

            turret.setVelocityFeedforward(0.0);
            turret.setTarget(smoothTarget);
            turret.update();

            status = "BAD_PEDRO_POSE";
            return;
        }

        double nextGeometry =
                anchorTurret
                        + HEADING_SIGN
                        * delta(heading, anchorHeading)
                        - delta(currentBearing, anchorBearing);

        if (Math.abs(
                nextGeometry - geometryTarget
        ) > MAX_GEOMETRY_JUMP) {

            status = "POSE_JUMP";

            turret.setVelocityFeedforward(0.0);
            turret.setTarget(smoothTarget);
            turret.update();
            return;
        }

        geometryTarget = nextGeometry;

        // ======================================
        // 2. CORRECCION LIMELIGHT
        // ======================================

        long frame = vision.getFrameSequence();

        if (vision.isNewFrame()
                && frame != lastFrame) {

            lastFrame = frame;

            double tx = vision.getTx();

            boolean valid =
                    vision.hasTarget()
                            && Double.isFinite(tx)
                            && vision.getAgeMs() >= 0.0
                            && vision.getAgeMs() <= MAX_AGE_MS
                            && Math.abs(tx) <= MAX_TX;

            if (valid) {

                lastTx = tx;

                double error;

                if (Math.abs(tx) <= TX_DEADBAND) {
                    error = 0.0;
                } else {
                    error = Math.signum(tx)
                            * (Math.abs(tx) - TX_DEADBAND);
                }

                double measuredOffset = Range.clip(
                        TX_SIGN * VISION_GAIN * error,
                        -VISION_LIMIT_DEG,
                        VISION_LIMIT_DEG
                );

                double alpha =
                        1.0 - Math.exp(
                                -dt / Math.max(
                                        0.01,
                                        VISION_FILTER_S
                                )
                        );

                double step = Range.clip(
                        alpha * (
                                measuredOffset - visionOffset
                        ),
                        -MAX_VISION_STEP_DEG,
                        MAX_VISION_STEP_DEG
                );

                visionOffset += step;

                status = "TRACKING";

            } else {
                status = "PEDRO_NO_TAG";
            }
        }

        // ======================================
        // 3. OBJETIVO FINAL
        // ======================================

        desiredTarget = Range.clip(
                geometryTarget + visionOffset,
                TurretSubsystemV4.MIN_ANGLE_DEG,
                TurretSubsystemV4.MAX_ANGLE_DEG
        );

        // ======================================
        // 4. PERFIL DE MOVIMIENTO
        // ======================================

        double error = desiredTarget - smoothTarget;

        double stopSpeed = Math.sqrt(
                2.0 * MAX_TARGET_ACCEL
                        * Math.abs(error)
        );

        double wantedSpeed =
                Math.signum(error)
                        * Math.min(
                        MAX_TARGET_SPEED,
                        stopSpeed
                );

        profileVelocity += Range.clip(
                wantedSpeed - profileVelocity,
                -MAX_TARGET_ACCEL * dt,
                MAX_TARGET_ACCEL * dt
        );

        double step = profileVelocity * dt;

        if (Math.abs(step) >= Math.abs(error)
                && Math.signum(step)
                == Math.signum(error)) {

            smoothTarget = desiredTarget;
            profileVelocity = 0.0;

        } else {
            smoothTarget += step;
        }

        smoothTarget = Range.clip(
                smoothTarget,
                TurretSubsystemV4.MIN_ANGLE_DEG,
                TurretSubsystemV4.MAX_ANGLE_DEG
        );

        // Sin feedforward adicional durante
        // la primera validacion.
        turret.setVelocityFeedforward(0.0);
        turret.setTarget(smoothTarget);
        turret.update();
    }
}

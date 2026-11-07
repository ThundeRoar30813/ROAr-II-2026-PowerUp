
package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

public class TurretFieldTrackingCommand {

    // SUBSYSTEMS
    private final DriveSubsystem drive;
    private final TurretSubsystem turret;

    // TARGET
    private double targetX;
    private double targetY;

    private boolean enabled;

    // REFERENCIA ANGULAR
    private double turretFieldZeroDeg;

    private double previousRawAngle;
    private double unwrappedDesiredAngle;
    private double previousDesiredAngle;

    private long previousTime;

    // TELEMETRIA
    private double distanceToTarget;
    private double fieldAngleDeg;

    private static final double
            MAX_TARGET_SPEED_DEG_PER_SEC = 500.0;

    // =====================================================
    // CONSTRUCTOR
    // =====================================================

    public TurretFieldTrackingCommand(
            DriveSubsystem drive,
            TurretSubsystem turret,
            double targetX,
            double targetY
    ) {

        this.drive = drive;
        this.turret = turret;

        this.targetX = targetX;
        this.targetY = targetY;
    }

    // =====================================================
    // ENABLE
    // =====================================================

    public void enable() {

        double dx = targetX - drive.getX();
        double dy = targetY - drive.getY();

        distanceToTarget = Math.hypot(dx, dy);

        fieldAngleDeg =
                Math.toDegrees(Math.atan2(dy, dx));

        double heading = drive.getHeadingDegrees();

        if (!Double.isFinite(distanceToTarget)
                || distanceToTarget < 1e-6
                || !Double.isFinite(fieldAngleDeg)
                || !Double.isFinite(heading)) {

            disable();
            return;
        }

        // CONSERVAR ANGULO ACTUAL
        double currentAngle =
                turret.getCurrentAngleDeg();

        turretFieldZeroDeg = normalizeDegrees(
                heading
                        - fieldAngleDeg
                        - currentAngle
        );

        turret.enableControl();

        previousRawAngle = normalizeDegrees(
                heading
                        - fieldAngleDeg
                        - turretFieldZeroDeg
        );

        unwrappedDesiredAngle = currentAngle;
        previousDesiredAngle = currentAngle;

        previousTime = System.nanoTime();

        enabled = true;
    }

    // =====================================================
    // EXECUTE
    // =====================================================

    public void execute() {

        if (!enabled) {
            return;
        }

        if (!turret.isControlEnabled()) {

            disable();
            return;
        }

        // POSICION RELATIVA AL OBJETIVO
        double dx = targetX - drive.getX();
        double dy = targetY - drive.getY();

        distanceToTarget = Math.hypot(dx, dy);

        fieldAngleDeg =
                Math.toDegrees(Math.atan2(dy, dx));

        double heading =
                drive.getHeadingDegrees();

        if (!Double.isFinite(distanceToTarget)
                || distanceToTarget < 1e-6
                || !Double.isFinite(fieldAngleDeg)
                || !Double.isFinite(heading)) {

            disable();
            return;
        }

        // =================================================
        // ANGULO DESEADO
        // =================================================

        double rawDesired = normalizeDegrees(
                heading
                        - fieldAngleDeg
                        - turretFieldZeroDeg
        );

        // EVITAR SALTOS ENTRE +180 Y -180
        double angleChange = normalizeDegrees(
                rawDesired - previousRawAngle
        );

        previousRawAngle = rawDesired;

        // ANGULO CONTINUO
        unwrappedDesiredAngle += angleChange;

        // LIMITAR OBJETIVO A -110 / +230
        double desired = Range.clip(
                unwrappedDesiredAngle,
                TurretSubsystem.MIN_ANGLE_DEG,
                TurretSubsystem.MAX_ANGLE_DEG
        );

        // =================================================
        // TIEMPO
        // =================================================

        long now = System.nanoTime();

        double dt =
                (now - previousTime) / 1_000_000_000.0;

        previousTime = now;

        if (dt <= 0 || dt > 0.1) {
            dt = 0.02;
        }

        // =================================================
        // VELOCIDAD DEL TARGET
        // =================================================

        double velocity = Range.clip(
                (desired - previousDesiredAngle) / dt,
                -MAX_TARGET_SPEED_DEG_PER_SEC,
                MAX_TARGET_SPEED_DEG_PER_SEC
        );

        previousDesiredAngle = desired;

        // =================================================
        // ACTUALIZAR TORRETA
        // =================================================

        turret.setTargetAngleDeg(desired);

        turret.setTargetVelocityDegPerSec(velocity);

        turret.update();
    }

    // =====================================================
    // DISABLE
    // =====================================================

    public void disable() {

        enabled = false;
        turret.disableControl();
    }

    // =====================================================
    // GETTERS
    // =====================================================

    public boolean isEnabled() {
        return enabled;
    }

    public double getTargetX() {
        return targetX;
    }

    public double getTargetY() {
        return targetY;
    }

    public double getDistanceToTarget() {
        return distanceToTarget;
    }

    public double getFieldAngleDeg() {
        return fieldAngleDeg;
    }

    public double getUnwrappedDesiredAngle() {
        return unwrappedDesiredAngle;
    }

    // =====================================================
    // CAMBIAR TARGET
    // =====================================================

    public void setTarget(double x, double y) {

        targetX = x;
        targetY = y;

        if (enabled) {
            enable();
        }
    }

    // =====================================================
    // NORMALIZAR ANGULO
    // =====================================================

    private static double normalizeDegrees(double angle) {

        angle = (angle + 180.0) % 360.0;

        if (angle < 0) {
            angle += 360.0;
        }

        return angle - 180.0;
    }
}

package org.firstinspires.ftc.teamcode.Commands.turret;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemp;

public class TurretFieldTrackingCommand1 {
    private final DriveSubsystem drive;
    private final TurretSubsystemp turret;

    private double targetX;
    private double targetY;

    private boolean enabled = false;

    private double turretFieldZeroDeg = 0.0;

    // =========================================================
    // VELOCIDAD DEL TARGET
    // =========================================================

    private double previousDesiredAngle = 0.0;
    private long previousTime;

    private double targetVelocityDegPerSec = 0.0;

    /*
     * Evita picos absurdos por ruido o por un loop extraño.
     */
    private static final double MAX_TARGET_VELOCITY = 500.0;

    // =========================================================
    // CONSTRUCTOR
    // =========================================================

    public TurretFieldTrackingCommand1(
            DriveSubsystem drive,
            TurretSubsystemp turret,
            double targetX,
            double targetY
    ) {

        this.drive = drive;
        this.turret = turret;

        this.targetX = targetX;
        this.targetY = targetY;
    }

    // =========================================================
    // ENABLE
    // =========================================================

    public void enable() {

        double robotX =
                drive.getX();

        double robotY =
                drive.getY();

        double dx =
                targetX - robotX;

        double dy =
                targetY - robotY;

        double fieldAngleDeg =
                Math.toDegrees(
                        Math.atan2(dy, dx)
                );

        double robotHeadingDeg =
                drive.getHeadingDegrees();

        turret.resetZero();

        turretFieldZeroDeg =
                normalizeDegrees(
                        fieldAngleDeg
                                - robotHeadingDeg
                );

        previousDesiredAngle = 0.0;

        targetVelocityDegPerSec = 0.0;

        previousTime =
                System.nanoTime();

        enabled = true;
    }

    // =========================================================
    // EXECUTE
    // =========================================================

    public void execute() {

        if (!enabled) {
            return;
        }

        // =====================================================
        // PEDRO POSE
        // =====================================================

        double robotX =
                drive.getX();

        double robotY =
                drive.getY();

        double robotHeadingDeg =
                drive.getHeadingDegrees();

        // =====================================================
        // VECTOR AL OBJETIVO
        // =====================================================

        double dx =
                targetX - robotX;

        double dy =
                targetY - robotY;

        // =====================================================
        // ÁNGULO GLOBAL AL TARGET
        // =====================================================

        double fieldAngleDeg =
                Math.toDegrees(
                        Math.atan2(dy, dx)
                );

        // =====================================================
        // TARGET DE TORRETA
        // =====================================================

        double desiredAngle =
                normalizeDegrees(
                        fieldAngleDeg
                                - robotHeadingDeg
                                - turretFieldZeroDeg
                );

        // =====================================================
        // TIEMPO
        // =====================================================

        long now =
                System.nanoTime();

        double dt =
                (now - previousTime)
                        / 1_000_000_000.0;

        previousTime = now;

        if (dt <= 0 || dt > 0.1) {
            dt = 0.02;
        }

        // =====================================================
        // VELOCIDAD DEL TARGET
        // =====================================================

        /*
         * MUY IMPORTANTE:
         *
         * NO hacemos:
         *
         * desiredAngle - previousDesiredAngle
         *
         * directamente.
         *
         * Porque:
         *
         * 179 -> -179
         *
         * realmente son +2 grados,
         * NO -358 grados.
         */

        double angleChange =
                angleDifferenceDegrees(
                        desiredAngle,
                        previousDesiredAngle
                );

        targetVelocityDegPerSec =
                angleChange / dt;

        targetVelocityDegPerSec =
                clamp(
                        targetVelocityDegPerSec,
                        -MAX_TARGET_VELOCITY,
                        MAX_TARGET_VELOCITY
                );

        previousDesiredAngle =
                desiredAngle;

        // =====================================================
        // ENVIAR AL SUBSYSTEM
        // =====================================================

        turret.setTargetAngleDeg(
                desiredAngle
        );

        turret.setTargetVelocityDegPerSec(
                targetVelocityDegPerSec
        );

        turret.update();
    }

    // =========================================================
    // DISABLE
    // =========================================================

    public void disable() {

        enabled = false;

        targetVelocityDegPerSec = 0.0;

        turret.stop();
    }

    // =========================================================
    // TARGET
    // =========================================================

    public void setTarget(
            double x,
            double y
    ) {

        targetX = x;
        targetY = y;
    }

    public double getTargetX() {

        return targetX;
    }

    public double getTargetY() {

        return targetY;
    }

    // =========================================================
    // INFO
    // =========================================================

    public boolean isEnabled() {

        return enabled;
    }

    public double getTargetVelocityDegPerSec() {

        return targetVelocityDegPerSec;
    }

    public double getDistanceToTarget() {

        double dx =
                targetX - drive.getX();

        double dy =
                targetY - drive.getY();

        return Math.hypot(
                dx,
                dy
        );
    }

    public double getFieldAngleDeg() {

        double dx =
                targetX - drive.getX();

        double dy =
                targetY - drive.getY();

        return Math.toDegrees(
                Math.atan2(
                        dy,
                        dx
                )
        );
    }

    // =========================================================
    // ANGLE DIFFERENCE
    // =========================================================

    private double angleDifferenceDegrees(
            double current,
            double previous
    ) {

        return normalizeDegrees(
                current - previous
        );
    }

    // =========================================================
    // NORMALIZE
    // =========================================================

    private double normalizeDegrees(
            double angle
    ) {

        while (angle > 180.0) {
            angle -= 360.0;
        }

        while (angle < -180.0) {
            angle += 360.0;
        }

        return angle;
    }

    // =========================================================
    // CLAMP
    // =========================================================

    private double clamp(
            double value,
            double min,
            double max
    ) {

        return Math.max(
                min,
                Math.min(
                        max,
                        value
                )
        );
    }
}

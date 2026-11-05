
package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

public class TurretFieldTrackingCommand {

    private final DriveSubsystem drive;
    private final TurretSubsystem turret;

    private double targetX;
    private double targetY;

    private boolean enabled = false;

    private double turretFieldZeroDeg = 0.0;
    private double distanceToTarget = 0.0;
    private double desiredAngleDeg = 0.0;

    // Versión base: sin Limelight ni feedforward.
    private static final double MAX_TARGET_RATE = 500.0;

    private double previousTarget;
    private long previousTime;

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

    public void enable() {

        double fieldAngle = calculateFieldAngle();
        double robotHeading = drive.getHeadingDegrees();

        // Conserva el ángulo físico actual como referencia.
        turretFieldZeroDeg = normalize180(
                robotHeading
                        - fieldAngle
                        - turret.getCurrentAngleDeg()
        );

        previousTarget = turret.getCurrentAngleDeg();
        previousTime = System.nanoTime();

        enabled = true;
    }

    public void disable() {
        enabled = false;
        turret.stop();
    }

    public void execute() {

        if (!enabled) {
            return;
        }

        double fieldAngle = calculateFieldAngle();
        double robotHeading = drive.getHeadingDegrees();

        desiredAngleDeg = normalize180(
                robotHeading
                        - fieldAngle
                        - turretFieldZeroDeg
        );

        // Evita ordenar vueltas completas fuera
        // del recorrido permitido.
        double commandedAngle = clamp(
                desiredAngleDeg,
                TurretSubsystem.MIN_ANGLE_DEG,
                TurretSubsystem.MAX_ANGLE_DEG
        );

        long now = System.nanoTime();

        double dt = (now - previousTime) / 1e9;
        previousTime = now;

        if (dt <= 0 || dt > 0.1) {
            dt = 0.02;
        }

        double targetVelocity = clamp(
                (commandedAngle - previousTarget) / dt,
                -MAX_TARGET_RATE,
                MAX_TARGET_RATE
        );

        previousTarget = commandedAngle;

        turret.setTargetAngleDeg(commandedAngle);
        turret.setTargetVelocityDegPerSec(targetVelocity);
        turret.update();
    }

    private double calculateFieldAngle() {

        double dx = targetX - drive.getX();
        double dy = targetY - drive.getY();

        distanceToTarget = Math.hypot(dx, dy);

        return Math.toDegrees(Math.atan2(dy, dx));
    }

    private static double normalize180(double angle) {

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
            double minimum,
            double maximum
    ) {
        return Math.max(minimum, Math.min(maximum, value));
    }

    public void setTarget(double x, double y) {
        targetX = x;
        targetY = y;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public double getDistanceToTarget() {
        return distanceToTarget;
    }

    public double getDesiredAngleDeg() {
        return desiredAngleDeg;
    }
}


package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV3;

public class TurretAimingV3 {

    private final DriveSubsystem drive;
    private final TurretSubsystemV3 turret;

    private double targetX;
    private double targetY;

    private boolean enabled = false;
    private boolean reachable = true;

    private double referenceOffset = 0;
    private double desiredAngle = 0;
    private double fieldAngle = 0;
    private double distance = 0;

    // Cambiar solamente si la prueba demuestra
    // que la compensacion del chasis esta invertida.
    private static final double TRACKING_SIGN = 1.0;

    public TurretAimingV3(
            DriveSubsystem drive,
            TurretSubsystemV3 turret,
            double targetX,
            double targetY
    ) {
        this.drive = drive;
        this.turret = turret;
        this.targetX = targetX;
        this.targetY = targetY;
    }

    // =====================================================
    // NORMALIZAR ANGULOS
    // =====================================================

    private static double wrap(double angle) {

        angle = (angle + 180.0) % 360.0;

        if (angle < 0.0) {
            angle += 360.0;
        }

        return angle - 180.0;
    }

    // =====================================================
    // CALCULAR ANGULO HACIA EL OBJETIVO
    // =====================================================

    private double calculateBaseAngle() {

        double dx = targetX - drive.getX();
        double dy = targetY - drive.getY();

        distance = Math.hypot(dx, dy);

        fieldAngle = Math.toDegrees(
                Math.atan2(dy, dx)
        );

        double heading = drive.getHeadingDegrees();

        // Convencion utilizada en el tracking anterior.
        return TRACKING_SIGN * (heading - fieldAngle);
    }

    // =====================================================
    // ACTIVAR TRACKING
    // =====================================================

    public void enable() {

        double dx = targetX - drive.getX();
        double dy = targetY - drive.getY();

        if (!Double.isFinite(dx)
                || !Double.isFinite(dy)
                || Math.hypot(dx, dy) < 1e-6
                || !Double.isFinite(drive.getHeadingDegrees())
                || turret.isStalled()) {

            disable();
            return;
        }

        // La torreta debe apuntar fisicamente al target
        // antes de activar el tracking.
        double currentAngle = turret.getAngle();

        referenceOffset = wrap(
                calculateBaseAngle() - currentAngle
        );

        desiredAngle = currentAngle;

        turret.enable();

        enabled = turret.isEnabled();
    }

    // =====================================================
    // EJECUTAR TRACKING
    // =====================================================

    public void execute() {

        if (!enabled) {
            return;
        }

        if (!turret.isEnabled() || turret.isStalled()) {
            enabled = false;
            return;
        }

        double dx = targetX - drive.getX();
        double dy = targetY - drive.getY();

        if (!Double.isFinite(dx)
                || !Double.isFinite(dy)
                || Math.hypot(dx, dy) < 1e-6
                || !Double.isFinite(drive.getHeadingDegrees())) {

            disable();
            return;
        }

        double base = wrap(
                calculateBaseAngle() - referenceOffset
        );

        double current = turret.getAngle();

        double best = 0;
        double bestDistance = Double.POSITIVE_INFINITY;

        reachable = false;

        // =================================================
        // BUSCAR ANGULO EQUIVALENTE ALCANZABLE
        // =================================================

        for (int k = -2; k <= 2; k++) {

            double candidate = base + 360.0 * k;

            if (candidate >= TurretSubsystemV3.MIN_ANGLE
                    && candidate <= TurretSubsystemV3.MAX_ANGLE) {

                double candidateDistance =
                        Math.abs(candidate - current);

                if (candidateDistance < bestDistance) {

                    bestDistance = candidateDistance;
                    best = candidate;
                    reachable = true;
                }
            }
        }

        // =================================================
        // SI EL OBJETIVO NO ES ALCANZABLE
        // =================================================

        if (!reachable) {

            double min = TurretSubsystemV3.MIN_ANGLE;
            double max = TurretSubsystemV3.MAX_ANGLE;

            double minError = Math.abs(wrap(base - min));
            double maxError = Math.abs(wrap(base - max));

            best = minError <= maxError ? min : max;
        }

        desiredAngle = Range.clip(
                best,
                TurretSubsystemV3.MIN_ANGLE,
                TurretSubsystemV3.MAX_ANGLE
        );

        turret.setTarget(desiredAngle);
        turret.update();
    }

    // =====================================================
    // DESACTIVAR TRACKING
    // =====================================================

    public void disable() {

        enabled = false;
        turret.disable();
    }

    // =====================================================
    // CAMBIAR TARGET
    // =====================================================

    public void setTarget(double x, double y) {

        targetX = x;
        targetY = y;
    }

    // =====================================================
    // GETTERS
    // =====================================================

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isReachable() {
        return reachable;
    }

    public double getDesiredAngle() {
        return desiredAngle;
    }

    public double getFieldAngle() {
        return fieldAngle;
    }

    public double getDistance() {
        return distance;
    }

    public double getTargetX() {
        return targetX;
    }

    public double getTargetY() {
        return targetY;
    }
}

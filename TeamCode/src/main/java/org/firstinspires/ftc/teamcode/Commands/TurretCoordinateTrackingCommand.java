
package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

public class TurretCoordinateTrackingCommand {

    private final DriveSubsystem drive;
    private final TurretSubsystem turret;

    // Posición del objetivo en el campo
    private double targetX;
    private double targetY;

    // Signos por verificar durante la primera prueba
    private static final double HEADING_SIGN = 1.0;
    private static final double TURRET_SIGN = -1.0;

    // Margen de seguridad de los cables
    private static final double SAFE_MIN = -115.0;
    private static final double SAFE_MAX = 120.0;

    private static final double MAX_TARGET_SPEED = 500.0;

    private boolean enabled = false;
    private boolean reachable = false;

    private double referenceOffsetDeg = 0.0;
    private double requestedAngleDeg = 0.0;
    private double commandedAngleDeg = 0.0;

    private double previousTarget = 0.0;
    private long previousTime;

    public TurretCoordinateTrackingCommand(
            DriveSubsystem drive,
            TurretSubsystem turret
    ) {
        this.drive = drive;
        this.turret = turret;
    }

    public void setTarget(double x, double y) {
        targetX = x;
        targetY = y;
    }

    public void enable() {

        double bearing = calculateFieldBearing();

        double relativeBearing = normalize180(
                bearing
                        - HEADING_SIGN * drive.getHeadingDegrees()
        );

        // Se calibra usando la dirección física actual.
        referenceOffsetDeg =
                turret.getCurrentAngleDeg()
                        - TURRET_SIGN * relativeBearing;

        previousTarget = turret.getCurrentAngleDeg();
        previousTime = System.nanoTime();

        enabled = true;
    }

    public void disable() {
        enabled = false;
        turret.stop();
    }

    public void update() {

        if (!enabled) {
            return;
        }

        double bearing = calculateFieldBearing();

        double relativeBearing = normalize180(
                bearing
                        - HEADING_SIGN * drive.getHeadingDegrees()
        );

        double nominalAngle =
                referenceOffsetDeg
                        + TURRET_SIGN * relativeBearing;

        // Buscar una vuelta equivalente que esté dentro
        // del recorrido físico permitido.
        double bestAngle = nominalAngle;
        double bestDistance = Double.POSITIVE_INFINITY;
        boolean found = false;

        double actual = turret.getCurrentAngleDeg();

        for (int k = -3; k <= 3; k++) {

            double candidate = nominalAngle + 360.0 * k;

            if (candidate >= SAFE_MIN
                    && candidate <= SAFE_MAX) {

                double distance =
                        Math.abs(candidate - actual);

                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestAngle = candidate;
                    found = true;
                }
            }
        }

        reachable = found;

        if (found) {
            requestedAngleDeg = bestAngle;
            commandedAngleDeg = bestAngle;
        } else {

            // No hay un ángulo equivalente seguro.
            // Seleccionar el extremo permitido más cercano.
            double nearest = nominalAngle
                    + 360.0 * Math.round(
                    (actual - nominalAngle) / 360.0
            );

            requestedAngleDeg = nearest;

            commandedAngleDeg = clamp(
                    nearest,
                    SAFE_MIN,
                    SAFE_MAX
            );
        }

        long now = System.nanoTime();

        double dt = (now - previousTime) / 1e9;
        previousTime = now;

        if (dt <= 0 || dt > 0.1) {
            dt = 0.02;
        }

        double targetVelocity = clamp(
                (commandedAngleDeg - previousTarget) / dt,
                -MAX_TARGET_SPEED,
                MAX_TARGET_SPEED
        );

        previousTarget = commandedAngleDeg;

        turret.setTargetAngleDeg(commandedAngleDeg);
        turret.setTargetVelocityDegPerSec(targetVelocity);
        turret.update();
    }

    private double calculateFieldBearing() {

        double dx = targetX - drive.getX();
        double dy = targetY - drive.getY();

        return Math.toDegrees(Math.atan2(dy, dx));
    }

    private static double normalize180(double angle) {

        angle = (angle + 180.0) % 360.0;

        if (angle < 0) {
            angle += 360.0;
        }

        return angle - 180.0;
    }

    private static double clamp(
            double value,
            double min,
            double max
    ) {
        return Math.max(min, Math.min(max, value));
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isReachable() {
        return reachable;
    }

    public double getRequestedAngleDeg() {
        return requestedAngleDeg;
    }

    public double getCommandedAngleDeg() {
        return commandedAngleDeg;
    }

    public double getTargetX() {
        return targetX;
    }

    public double getTargetY() {
        return targetY;
    }
}

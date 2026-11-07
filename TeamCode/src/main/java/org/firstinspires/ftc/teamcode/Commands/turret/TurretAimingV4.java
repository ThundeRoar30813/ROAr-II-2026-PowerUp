
package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV3;

public class TurretAimingV4 {

    private final DriveSubsystem drive;
    private final TurretSubsystemV3 turret;
    private final TurretVisionV4 vision;

    private boolean enabled = false;
    private boolean firstUpdate = true;

    private double targetAngle = 0.0;
    private double lastHeading = 0.0;

    private double headingDelta = 0.0;
    private double visionCorrection = 0.0;

    private long lastTimeNs = 0;

    // Positivo = antihorario en Pedro y torreta.
    // Robot gira +10 -> torreta compensa -10.
    private static final double HEADING_SIGN = -1.0;

    // Tx positivo: Tag a la derecha de la imagen.
    // Bajo convencion de torreta CCW positiva:
    // correccion negativa.
    private static final double VISION_SIGN = -1.0;

    // Ganancia visual en 1/segundo.
    private static final double VISION_GAIN = 0.7;

    // Correccion visual maxima en grados/segundo.
    private static final double MAX_VISION_RATE = 12.0;

    // Limite de cambio de consigna.
    private static final double MAX_TARGET_RATE = 60.0;

    // Evita correcciones visuales cuando
    // el Tag esta practicamente centrado.
    private static final double TX_DEADBAND = 0.5;

    // Si el error es grande, no activar.
    private static final double MAX_START_TX = 3.0;

    // Proteccion frente a saltos de heading.
    private static final double MAX_HEADING_JUMP = 25.0;

    public TurretAimingV4(
            DriveSubsystem drive,
            TurretSubsystemV3 turret,
            TurretVisionV4 vision
    ) {
        this.drive = drive;
        this.turret = turret;
        this.vision = vision;
    }

    private static double wrap(double angle) {
        angle = (angle + 180.0) % 360.0;

        if (angle < 0) {
            angle += 360.0;
        }

        return angle - 180.0;
    }

    public boolean enable() {

        if (turret.isStalled()
                || !vision.hasTarget()
                || Math.abs(vision.getTx()) > MAX_START_TX) {
            return false;
        }

        double heading = drive.getHeadingDegrees();
        double actual = turret.getAngle();

        if (!Double.isFinite(heading)
                || !Double.isFinite(actual)) {
            return false;
        }

        // Captura la posicion actual.
        targetAngle = actual;
        lastHeading = heading;

        headingDelta = 0;
        visionCorrection = 0;

        lastTimeNs = System.nanoTime();
        firstUpdate = true;

        turret.enable();

        if (!turret.isEnabled()) {
            return false;
        }

        turret.setTarget(actual);
        enabled = true;

        return true;
    }

    public void update() {

        if (!enabled) {
            return;
        }

        if (turret.isStalled()
                || !turret.isEnabled()) {
            disable();
            return;
        }

        long now = System.nanoTime();

        double dt = (now - lastTimeNs) / 1e9;
        lastTimeNs = now;

        if (dt <= 0 || dt > 0.1) {
            dt = 0.02;
        }

        double heading = drive.getHeadingDegrees();

        if (!Double.isFinite(heading)) {
            disable();
            return;
        }

        if (firstUpdate) {

            firstUpdate = false;
            lastHeading = heading;

            turret.setTarget(targetAngle);
            turret.update();
            return;
        }

        // ==================================
        // COMPENSACION DEL HEADING
        // ==================================

        headingDelta = wrap(
                heading - lastHeading
        );

        lastHeading = heading;

        if (Math.abs(headingDelta)
                > MAX_HEADING_JUMP) {

            disable();
            return;
        }

        double odometryChange =
                HEADING_SIGN * headingDelta;

        // ==================================
        // CORRECCION LIMELIGHT
        // ==================================

        visionCorrection = 0;

        if (vision.hasTarget()) {

            double tx = vision.getTx();

            if (Math.abs(tx) > TX_DEADBAND) {

                double rate = Range.clip(
                        VISION_SIGN * VISION_GAIN * tx,
                        -MAX_VISION_RATE,
                        MAX_VISION_RATE
                );

                visionCorrection = rate * dt;
            }
        }

        // ==================================
        // CAMBIO TOTAL
        // ==================================

        double change =
                odometryChange + visionCorrection;

        double maxChange =
                MAX_TARGET_RATE * dt;

        change = Range.clip(
                change,
                -maxChange,
                maxChange
        );

        targetAngle += change;

        // ==================================
        // LIMITES MECANICOS
        // ==================================

        if (targetAngle <
                TurretSubsystemV3.MIN_ANGLE
                || targetAngle >
                TurretSubsystemV3.MAX_ANGLE) {

            disable();
            return;
        }

        turret.setTarget(targetAngle);
        turret.update();
    }

    public void disable() {
        enabled = false;
        turret.disable();
    }

    public boolean isEnabled() {
        return enabled;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getHeadingDelta() {
        return headingDelta;
    }

    public double getVisionCorrection() {
        return visionCorrection;
    }
}

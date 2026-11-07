
package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV3;

public class TurretAimingV5 {

    private final DriveSubsystem drive;
    private final TurretSubsystemV3 turret;
    private final TurretVisionV4 vision;

    private boolean enabled = false;
    private boolean visionEnabled = false;

    private double referenceHeading = 0;
    private double referenceTurretAngle = 0;

    private double headingChange = 0;
    private double desiredAngle = 0;
    private double commandedAngle = 0;

    private double visionOffset = 0;
    private double visionCorrection = 0;

    private long lastTimeNs = 0;

    // Robot +20 grados -> torreta -20 grados.
    private static final double HEADING_SIGN = 1.0;

    // Mantener desactivado hasta validar
    // compensacion por heading.
    private static final double VISION_SIGN = -1.0;

    private static final double VISION_GAIN = 0.5;
    private static final double MAX_VISION_RATE = 8.0;

    private static final double MAX_TARGET_RATE = 120.0;

    private static final double MAX_HEADING_STEP = 25.0;

    private static final double TX_DEADBAND = 0.5;

    private double filteredTx = 0.0;
    private boolean txInitialized = false;

    private static final double TX_FILTER_ALPHA = 0.15;

    public TurretAimingV5(
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

        if (turret.isStalled()) {
            return false;
        }

        double heading = drive.getHeadingDegrees();
        double actual = turret.getAngle();

        if (!Double.isFinite(heading)
                || !Double.isFinite(actual)) {
            return false;
        }

        referenceHeading = heading;
        referenceTurretAngle = actual;

        headingChange = 0;
        visionOffset = 0;
        visionCorrection = 0;

        desiredAngle = actual;
        commandedAngle = actual;

        lastTimeNs = System.nanoTime();

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

        // ===================================
        // COMPENSACION DEL CHASIS
        // ===================================

        double newHeadingChange = wrap(
                heading - referenceHeading
        );

        double headingStep = wrap(
                newHeadingChange - headingChange
        );

        if (Math.abs(headingStep)
                > MAX_HEADING_STEP) {
            disable();
            return;
        }

        headingChange = newHeadingChange;

        // ===================================
        // CORRECCION VISUAL OPCIONAL
        // ===================================

        visionCorrection = 0;

        if (visionEnabled && vision.hasTarget()) {

            double tx = vision.getTx();

            if (Math.abs(tx) > TX_DEADBAND) {

                double rate = Range.clip(
                        VISION_SIGN * VISION_GAIN * tx,
                        -MAX_VISION_RATE,
                        MAX_VISION_RATE
                );

                visionCorrection = rate * dt;
                visionOffset += visionCorrection;
            }
        }

        // ===================================
        // OBJETIVO ABSOLUTO
        // ===================================

        desiredAngle =
                referenceTurretAngle
                        + HEADING_SIGN * headingChange
                        + visionOffset;

        // Si no es alcanzable, detener.
        if (desiredAngle <
                TurretSubsystemV3.MIN_ANGLE
                || desiredAngle >
                TurretSubsystemV3.MAX_ANGLE) {
            disable();
            return;
        }

        // ===================================
        // RATE LIMIT SIN PERDER ANGULO
        // ===================================

        double maxStep = MAX_TARGET_RATE * dt;

        commandedAngle += Range.clip(
                desiredAngle - commandedAngle,
                -maxStep,
                maxStep
        );

        turret.setTarget(commandedAngle);
        turret.update();
    }

    public void disable() {
        enabled = false;
        turret.disable();
    }

    public void setVisionEnabled(boolean value) {
        visionEnabled = value;
        visionOffset = 0;
        visionCorrection = 0;

        // Al cambiar modo, recalibrar referencia
        // para no provocar saltos.
        if (enabled) {
            referenceHeading = drive.getHeadingDegrees();
            referenceTurretAngle = turret.getAngle();
            headingChange = 0;
            desiredAngle = referenceTurretAngle;
            commandedAngle = referenceTurretAngle;
            turret.setTarget(commandedAngle);
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isVisionEnabled() {
        return visionEnabled;
    }

    public double getDesiredAngle() {
        return desiredAngle;
    }

    public double getCommandedAngle() {
        return commandedAngle;
    }

    public double getHeadingChange() {
        return headingChange;
    }

    public double getVisionCorrection() {
        return visionCorrection;
    }

    public double getVisionOffset() {
        return visionOffset;
    }
}

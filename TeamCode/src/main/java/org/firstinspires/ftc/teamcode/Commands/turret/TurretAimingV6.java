package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV3;

public class TurretAimingV6 {

    // ==========================================
    // SUBSYSTEMS
    // ==========================================

    private final DriveSubsystem drive;
    private final TurretSubsystemV3 turret;
    private final TurretVisionV4 vision;

    // ==========================================
    // SIGNOS YA CALIBRADOS
    // ==========================================

    private static final double HEADING_SIGN = +1.0;
    private static final double VISION_SIGN = -1.0;

    // ==========================================
    // FILTRO DE LIMELIGHT
    // ==========================================

    // 0.0 = filtrado muy fuerte
    // 1.0 = sin filtrado
    private static final double TX_FILTER_ALPHA = 0.25;

    // Zona estable: deja de corregir
    private static final double TX_DEADBAND = 0.8;

    // Reactiva la correccion visual
    private static final double TX_RESUME_THRESHOLD = 1.5;

    // Limite de velocidad visual
    private static final double MAX_VISION_RATE = 35.0;

    // ==========================================
    // GANANCIAS PROGRESIVAS
    // ==========================================

    private static final double GAIN_FAR = 2.0;
    private static final double GAIN_MEDIUM = 1.0;
    private static final double GAIN_NEAR = 0.35;

    // ==========================================
    // LIMITES DE MOVIMIENTO
    // ==========================================

    private static final double MAX_TARGET_RATE = 120.0;
    private static final double MAX_HEADING_STEP = 25.0;

    // ==========================================
    // ESTADO
    // ==========================================

    private boolean enabled = false;
    private boolean visionEnabled = false;

    private double referenceHeading = 0;
    private double referenceTurretAngle = 0;

    private double headingChange = 0;
    private double lastHeadingChange = 0;

    private double desiredAngle = 0;
    private double commandedAngle = 0;

    private double visionOffset = 0;
    private double visionCorrection = 0;

    private double filteredTx = 0;
    private boolean txInitialized = false;
    private boolean correctingVision = false;

    private long lastTime = 0;

    // ==========================================
    // CONSTRUCTOR
    // ==========================================

    public TurretAimingV6(
            DriveSubsystem drive,
            TurretSubsystemV3 turret,
            TurretVisionV4 vision
    ) {
        this.drive = drive;
        this.turret = turret;
        this.vision = vision;
    }

    // ==========================================
    // ENABLE
    // ==========================================

    public void enable() {

        if (turret.isStalled()) {
            return;
        }

        referenceHeading = drive.getHeadingDegrees();
        referenceTurretAngle = turret.getAngle();

        headingChange = 0;
        lastHeadingChange = 0;

        desiredAngle = referenceTurretAngle;
        commandedAngle = referenceTurretAngle;

        visionOffset = 0;
        visionCorrection = 0;

        resetVisionFilter();

        lastTime = System.nanoTime();

        turret.enable();
        turret.setTarget(referenceTurretAngle);

        enabled = turret.isEnabled();
    }

    // ==========================================
    // DISABLE
    // ==========================================

    public void disable() {

        enabled = false;

        turret.disable();

        visionCorrection = 0;

        resetVisionFilter();
    }

    // ==========================================
    // VISION ON / OFF
    // ==========================================

    public void setVisionEnabled(boolean value) {

        if (visionEnabled == value) {
            return;
        }

        visionEnabled = value;

        visionCorrection = 0;

        resetVisionFilter();
    }

    // ==========================================
    // REINICIAR FILTRO
    // ==========================================

    private void resetVisionFilter() {

        filteredTx = 0;
        txInitialized = false;
        correctingVision = false;
    }

    // ==========================================
    // UPDATE
    // ==========================================

    public void update() {

        if (!enabled) {
            return;
        }

        if (turret.isStalled() || !turret.isEnabled()) {
            disable();
            return;
        }

        long now = System.nanoTime();

        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        if (dt <= 0 || dt > 0.1) {
            dt = 0.02;
        }

        // ======================================
        // COMPENSACION DEL CHASIS
        // ======================================

        double currentHeading = drive.getHeadingDegrees();

        double newHeadingChange = wrap(
                currentHeading - referenceHeading
        );

        double headingStep = wrap(
                newHeadingChange - lastHeadingChange
        );

        if (Math.abs(headingStep) > MAX_HEADING_STEP) {
            disable();
            return;
        }

        headingChange = newHeadingChange;
        lastHeadingChange = newHeadingChange;

        // ======================================
        // CORRECCION VISUAL
        // ======================================

        visionCorrection = 0;

        boolean validTarget =
                visionEnabled
                        && vision.hasTarget()
                        && Double.isFinite(vision.getTx());

        if (validTarget) {

            double rawTx = vision.getTx();

            // Filtro exponencial para vibraciones
            if (!txInitialized) {

                filteredTx = rawTx;
                txInitialized = true;

            } else {

                filteredTx += TX_FILTER_ALPHA
                        * (rawTx - filteredTx);
            }

            double absTx = Math.abs(filteredTx);

            // ==================================
            // HISTERESIS
            // ==================================

            if (correctingVision) {

                if (absTx <= TX_DEADBAND) {
                    correctingVision = false;
                }

            } else {

                if (absTx >= TX_RESUME_THRESHOLD) {
                    correctingVision = true;
                }
            }

            // ==================================
            // GANANCIA PROGRESIVA
            // ==================================

            if (correctingVision) {

                double gain;

                if (absTx > 5.0) {

                    // Lejos: rapido
                    gain = GAIN_FAR;

                } else if (absTx > 2.0) {

                    // Distancia media
                    gain = GAIN_MEDIUM;

                } else {

                    // Cerca: ajuste suave
                    gain = GAIN_NEAR;
                }

                visionCorrection = Range.clip(
                        VISION_SIGN * gain * filteredTx,
                        -MAX_VISION_RATE,
                        MAX_VISION_RATE
                );

                // Integrar correccion visual
                visionOffset += visionCorrection * dt;
            }

        } else {

            // Sin deteccion no se generan
            // correcciones visuales nuevas.
            resetVisionFilter();
        }

        // ======================================
        // OBJETIVO DE TORRETA
        // ======================================

        desiredAngle =
                referenceTurretAngle
                        + HEADING_SIGN * headingChange
                        + visionOffset;

        // Proteccion de limites
        if (desiredAngle < TurretSubsystemV3.MIN_ANGLE
                || desiredAngle > TurretSubsystemV3.MAX_ANGLE) {

            disable();
            return;
        }

        // ======================================
        // LIMITAR VELOCIDAD DEL OBJETIVO
        // ======================================

        double maxStep = MAX_TARGET_RATE * dt;

        commandedAngle += Range.clip(
                desiredAngle - commandedAngle,
                -maxStep,
                maxStep
        );

        // ======================================
        // ACTUALIZAR TORRETA
        // ======================================

        turret.setTarget(commandedAngle);
        turret.update();

        if (turret.isStalled()) {
            disable();
        }
    }

    // ==========================================
    // NORMALIZAR ANGULO
    // ==========================================

    private double wrap(double degrees) {

        while (degrees > 180.0) {
            degrees -= 360.0;
        }

        while (degrees < -180.0) {
            degrees += 360.0;
        }

        return degrees;
    }

    // ==========================================
    // TELEMETRIA
    // ==========================================

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isVisionEnabled() {
        return visionEnabled;
    }

    public double getHeadingChange() {
        return headingChange;
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

    public double getVisionCorrection() {
        return visionCorrection;
    }

    public double getFilteredTx() {
        return filteredTx;
    }

    public boolean isCorrectingVision() {
        return correctingVision;
    }
}
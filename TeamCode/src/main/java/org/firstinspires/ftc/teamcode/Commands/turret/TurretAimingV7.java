
package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV3;

public class TurretAimingV7 {

    // ==========================================
    // SUBSYSTEMS
    // ==========================================

    private final DriveSubsystem drive;
    private final TurretSubsystemV3 turret;
    private final TurretVisionV4 vision;

    // ==========================================
    // SIGNOS CALIBRADOS
    // ==========================================

    private static final double HEADING_SIGN = +1.0;
    private static final double VISION_SIGN = -1.0;

    // ==========================================
    // CONTROL VISUAL
    // ==========================================

    private static final double TX_DEADBAND = 0.5;
    private static final double TX_RESUME = 1.0;

    private static final double GAIN_FAR = 6.0;
    private static final double GAIN_MEDIUM = 4.0;
    private static final double GAIN_NEAR = 1.2;

    private static final double MAX_VISION_RATE = 90.0;

    // Filtro exponencial
    private static final double FILTER_ALPHA_MOVING = 0.30;
    private static final double FILTER_ALPHA_STOPPED = 0.40;

    // Rechazar un salto aislado muy grande
    private static final double TX_JUMP_THRESHOLD = 2.5;
    private static final int TX_JUMP_CONFIRM_FRAMES = 2;

    // ==========================================
    // DETECCION DE GIRO
    // ==========================================

    private static final double MOVING_RATE_DEG_S = 5.0;
    private static final double STOPPED_RATE_DEG_S = 2.0;

    // Esperar a que el chasis esté quieto
    private static final long STOP_CONFIRM_NS = 100_000_000L;

    // Recuperación visual gradual
    private static final long SETTLE_TIME_NS = 600_000_000L;

    // Mientras gira, usar menos vision
    private static final double MOVING_VISION_SCALE = 0.85;

    // ==========================================
    // LIMITES
    // ==========================================

    private static final double MAX_TARGET_RATE = 350.0;
    private static final double MAX_HEADING_STEP = 25.0;

    // ==========================================
    // ESTADO
    // ==========================================

    private boolean enabled = false;
    private boolean visionEnabled = false;

    private double referenceHeading = 0;
    private double referenceTurretAngle = 0;

    private double lastHeading = 0;
    private double headingChange = 0;

    private double desiredAngle = 0;
    private double commandedAngle = 0;

    private double visionOffset = 0;
    private double visionCorrection = 0;

    private double filteredTx = 0;
    private boolean txInitialized = false;
    private boolean correctingVision = false;

    private double headingRate = 0;
    private boolean chassisMoving = false;

    private long stoppedCandidateSince = 0;
    private long settleStart = 0;

    private double visionScale = 1.0;

    private double pendingJumpTx = 0;
    private int jumpFrames = 0;

    private long lastTime = 0;

    // ==========================================
    // CONSTRUCTOR
    // ==========================================

    public TurretAimingV7(
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

        lastHeading = referenceHeading;
        headingChange = 0;

        desiredAngle = referenceTurretAngle;
        commandedAngle = referenceTurretAngle;

        visionOffset = 0;
        visionCorrection = 0;

        headingRate = 0;
        chassisMoving = false;

        stoppedCandidateSince = 0;
        settleStart = 0;
        visionScale = 1.0;

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
    // VISION ON/OFF
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

        pendingJumpTx = 0;
        jumpFrames = 0;
    }

    // ==========================================
    // UPDATE
    // ==========================================

    public void update() {

        if (!enabled) {
            return;
        }

        if (!turret.isEnabled() || turret.isStalled()) {
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
        // HEADING Y VELOCIDAD ANGULAR
        // ======================================

        double currentHeading = drive.getHeadingDegrees();

        double headingStep = wrap(
                currentHeading - lastHeading
        );

        lastHeading = currentHeading;

        if (Math.abs(headingStep) > MAX_HEADING_STEP) {
            disable();
            return;
        }

        headingChange = wrap(
                currentHeading - referenceHeading
        );

        headingRate = headingStep / dt;

        // ======================================
        // DETECCION DE CHASIS EN MOVIMIENTO
        // ======================================

        double absRate = Math.abs(headingRate);

        if (absRate >= MOVING_RATE_DEG_S) {

            chassisMoving = true;
            stoppedCandidateSince = 0;
            settleStart = 0;

        } else if (chassisMoving) {

            if (absRate <= STOPPED_RATE_DEG_S) {

                if (stoppedCandidateSince == 0) {
                    stoppedCandidateSince = now;
                }

                if (now - stoppedCandidateSince
                        >= STOP_CONFIRM_NS) {

                    chassisMoving = false;
                    settleStart = now;
                    stoppedCandidateSince = 0;
                }

            } else {

                stoppedCandidateSince = 0;
            }
        }

        // ======================================
        // TRANSICION VISUAL SUAVE
        // ======================================

        if (chassisMoving) {

            visionScale = MOVING_VISION_SCALE;

        } else if (settleStart != 0) {

            double progress = Range.clip(
                    (double) (now - settleStart)
                            / SETTLE_TIME_NS,
                    0.0,
                    1.0
            );

            // Suavizado progresivo
            double smooth =
                    progress * progress
                            * (3.0 - 2.0 * progress);

            visionScale =
                    MOVING_VISION_SCALE
                            + (1.0 - MOVING_VISION_SCALE)
                            * smooth;

            if (progress >= 1.0) {
                settleStart = 0;
                visionScale = 1.0;
            }

        } else {

            visionScale = 1.0;
        }

        // ======================================
        // VISION
        // ======================================

        visionCorrection = 0;

        boolean validTarget =
                visionEnabled
                        && vision.hasTarget()
                        && Double.isFinite(vision.getTx());

        if (validTarget) {

            double rawTx = vision.getTx();

            // ==================================
            // RECHAZO DE SALTOS AISLADOS
            // ==================================

            if (txInitialized
                    && Math.abs(rawTx - filteredTx)
                    > TX_JUMP_THRESHOLD) {

                if (jumpFrames == 0
                        || Math.abs(rawTx - pendingJumpTx)
                        > 1.0) {

                    pendingJumpTx = rawTx;
                    jumpFrames = 1;

                } else {

                    jumpFrames++;
                }

                if (jumpFrames
                        < TX_JUMP_CONFIRM_FRAMES) {

                    rawTx = filteredTx;
                }

            } else {

                jumpFrames = 0;
            }

            // ==================================
            // FILTRO EXPONENCIAL
            // ==================================

            double alpha = chassisMoving
                    ? FILTER_ALPHA_MOVING
                    : FILTER_ALPHA_STOPPED;

            if (!txInitialized) {

                filteredTx = rawTx;
                txInitialized = true;

            } else {

                filteredTx += alpha
                        * (rawTx - filteredTx);
            }

            // ==================================
            // HISTERESIS
            // ==================================

            double absTx = Math.abs(filteredTx);

            if (correctingVision) {

                if (absTx <= TX_DEADBAND) {
                    correctingVision = false;
                }

            } else {

                if (absTx >= TX_RESUME) {
                    correctingVision = true;
                }
            }

            // ==================================
            // GANANCIA PROGRESIVA
            // ==================================

            if (correctingVision) {

                double gain;

                if (absTx > 5.0) {
                    gain = GAIN_FAR;
                } else if (absTx > 2.0) {
                    gain = GAIN_MEDIUM;
                } else {
                    gain = GAIN_NEAR;
                }

                visionCorrection = Range.clip(
                        VISION_SIGN
                                * gain
                                * filteredTx
                                * visionScale,
                        -MAX_VISION_RATE,
                        MAX_VISION_RATE
                );

                visionOffset += visionCorrection * dt;
            }

        } else {

            resetVisionFilter();
        }

        // ======================================
        // TARGET FINAL
        // ======================================

        desiredAngle =
                referenceTurretAngle
                        + HEADING_SIGN * headingChange
                        + visionOffset;

        if (desiredAngle < TurretSubsystemV3.MIN_ANGLE
                || desiredAngle > TurretSubsystemV3.MAX_ANGLE) {

            disable();
            return;
        }

        // ======================================
        // LIMITADOR DE TARGET
        // ======================================

        double maxStep = MAX_TARGET_RATE * dt;

        commandedAngle += Range.clip(
                desiredAngle - commandedAngle,
                -maxStep,
                maxStep
        );

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

    public double getHeadingRate() {
        return headingRate;
    }

    public boolean isChassisMoving() {
        return chassisMoving;
    }

    public double getVisionScale() {
        return visionScale;
    }

    public double getFilteredTx() {
        return filteredTx;
    }

    public boolean isCorrectingVision() {
        return correctingVision;
    }

    public double getVisionOffset() {
        return visionOffset;
    }

    public double getVisionCorrection() {
        return visionCorrection;
    }

    public double getDesiredAngle() {
        return desiredAngle;
    }

    public double getCommandedAngle() {
        return commandedAngle;
    }
}

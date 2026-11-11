
package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

public class TurretAimControllerV7Diagnostic {

    public enum Mode {
        VISION_ONLY,
        PEDRO_ONLY,
        COMBINED
    }

    private final DriveSubsystem drive;
    private final TurretSubsystemV4 turret;
    private final TurretVisionV5 vision;

    // Mantener los signos comprobados
    public static double HEADING_COMP_SIGN = 1.0;
    public static double TX_TO_TURRET_SIGN = -1.0;

    // Vision
    public static double TX_ALPHA_FAR = 0.65;
    public static double TX_ALPHA_NEAR = 0.30;
    public static double TX_NEAR_THRESHOLD = 3.0;
    public static double TX_DEADBAND = 0.65;

    public static double VISION_KP = 1.0;
    public static double MAX_VALID_TX = 27.0;
    public static double MAX_VISION_AGE_MS = 150.0;
    public static double MIN_VISION_INTERVAL_S = 0.015;

    // Fusion: que tan rapido adopta
    // el objetivo calculado por vision
    public static double FUSION_GAIN = 0.65;
    public static double VISION_ONLY_GAIN = 0.65;

    // Limite de cambio del objetivo visual
    public static double VISION_RATE_DEG_S = 300.0;

    // Compensacion de Pedro
    public static double RATE_FILTER_TIME_S = 0.045;
    public static double HEADING_LEAD_TIME_S = 0.025;
    public static double MAX_HEADING_LEAD_DEG = 6.0;
    public static double MAX_HEADING_STEP = 45.0;
    public static double MAX_CHASSIS_RATE = 350.0;

    // Perfil de movimiento
    public static double MAX_TARGET_SPEED = 300.0;
    public static double MAX_TARGET_ACCEL = 1800.0;

    // Suavizado del feedforward
    public static double FF_FILTER_TIME_S = 0.055;

    private Mode mode = Mode.VISION_ONLY;
    private boolean enabled = false;

    private double lastHeading;
    private double baseTarget;

    private double smoothTarget;
    private double smoothVelocity;

    private double filteredTx = Double.NaN;

    private double chassisRate = 0.0;
    private double headingLead = 0.0;
    private double filteredFF = 0.0;

    private double lastVisionTime = 0.0;
    private long lastProcessedFrame = -1;
    private long lastNs;

    private String status = "DISABLED";

    public TurretAimControllerV7Diagnostic(
            DriveSubsystem drive,
            TurretSubsystemV4 turret,
            TurretVisionV5 vision) {

        this.drive = drive;
        this.turret = turret;
        this.vision = vision;
    }

    public void setMode(Mode newMode) {

        if (newMode == null || newMode == mode) {
            return;
        }

        mode = newMode;

        if (enabled) {
            resetReference();
        }
    }

    public Mode getMode() {
        return mode;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public String getStatus() {
        return status;
    }

    public double getFilteredTx() {
        return filteredTx;
    }

    public double getBaseTarget() {
        return baseTarget;
    }

    public double getSmoothTarget() {
        return smoothTarget;
    }

    public double getSmoothVelocity() {
        return smoothVelocity;
    }

    public double getChassisRate() {
        return chassisRate;
    }

    public double getHeadingLead() {
        return headingLead;
    }

    public double getFilteredFF() {
        return filteredFF;
    }

    public void enable() {

        if (turret.isFault()) {
            status = "FAULT";
            return;
        }

        turret.enableHold();

        if (!turret.isEnabled()) {
            status = "NOT_ENABLED";
            return;
        }

        enabled = true;
        resetReference();
    }

    private void resetReference() {

        lastHeading = drive.getHeadingDegrees();

        baseTarget = turret.getAngle();
        smoothTarget = baseTarget;
        smoothVelocity = 0.0;

        filteredTx = Double.NaN;
        chassisRate = 0.0;
        headingLead = 0.0;
        filteredFF = 0.0;

        lastVisionTime = 0.0;
        lastProcessedFrame = vision.getFrameSequence();

        lastNs = System.nanoTime();

        turret.setVelocityFeedforward(0.0);
        turret.setTarget(smoothTarget);

        status = "READY_" + mode.name();
    }

    public void disable() {

        enabled = false;
        smoothVelocity = 0.0;
        filteredFF = 0.0;

        turret.setVelocityFeedforward(0.0);
        turret.disable();

        status = "DISABLED";
    }

    private double angleDifference(
            double current,
            double previous) {

        double diff = current - previous;

        while (diff > 180.0) {
            diff -= 360.0;
        }

        while (diff < -180.0) {
            diff += 360.0;
        }

        return diff;
    }

    public void update() {

        if (!enabled) {
            return;
        }

        if (turret.isFault()) {
            disable();
            status = "FAULT";
            return;
        }

        long now = System.nanoTime();

        double rawDt = (now - lastNs) / 1e9;
        lastNs = now;

        boolean validDt =
                Double.isFinite(rawDt)
                        && rawDt > 0.001
                        && rawDt <= 0.15;

        double dt = validDt ? rawDt : 0.02;

        // =====================================
        // 1. PEDRO HEADING
        // =====================================

        double heading = drive.getHeadingDegrees();

        if (!Double.isFinite(heading)) {
            turret.setVelocityFeedforward(0.0);
            turret.update();
            status = "INVALID_HEADING";
            return;
        }

        double deltaHeading =
                angleDifference(heading, lastHeading);

        lastHeading = heading;

        boolean usePedro =
                mode == Mode.PEDRO_ONLY
                        || mode == Mode.COMBINED;

        if (usePedro) {

            boolean validStep =
                    validDt
                            && Math.abs(deltaHeading)
                            <= MAX_HEADING_STEP;

            if (validStep) {

                baseTarget +=
                        HEADING_COMP_SIGN
                                * deltaHeading;
            }

            double measuredRate = 0.0;

            if (validStep) {
                measuredRate = Range.clip(
                        deltaHeading / dt,
                        -MAX_CHASSIS_RATE,
                        MAX_CHASSIS_RATE
                );
            }

            double rateAlpha =
                    1.0 - Math.exp(
                            -dt / RATE_FILTER_TIME_S
                    );

            chassisRate += rateAlpha
                    * (measuredRate - chassisRate);

            headingLead = Range.clip(
                    HEADING_COMP_SIGN
                            * chassisRate
                            * HEADING_LEAD_TIME_S,
                    -MAX_HEADING_LEAD_DEG,
                    MAX_HEADING_LEAD_DEG
            );

        } else {

            chassisRate = 0.0;
            headingLead = 0.0;
        }

        // =====================================
        // 2. LIMELIGHT
        // =====================================

        boolean useVision =
                mode == Mode.VISION_ONLY
                        || mode == Mode.COMBINED;

        if (useVision) {

            double age = vision.getAgeMs();

            boolean fresh =
                    Double.isFinite(age)
                            && age >= 0.0
                            && age <= MAX_VISION_AGE_MS;

            double nowSeconds = now / 1e9;

            long sequence = vision.getFrameSequence();

            boolean newFrame =
                    vision.isNewFrame()
                            && sequence != lastProcessedFrame;

            if (vision.hasTarget() && fresh) {

                double rawTx = vision.getTx();

                if (Double.isFinite(rawTx)
                        && Math.abs(rawTx)
                        <= MAX_VALID_TX) {

                    if (newFrame
                            && nowSeconds - lastVisionTime
                            >= MIN_VISION_INTERVAL_S) {

                        lastProcessedFrame = sequence;

                        double visionDt =
                                lastVisionTime == 0.0
                                        ? 0.02
                                        : Range.clip(
                                        nowSeconds
                                                - lastVisionTime,
                                        0.005,
                                        0.10
                                );

                        lastVisionTime = nowSeconds;

                        // Filtro adaptativo
                        double alpha =
                                Math.abs(rawTx)
                                        > TX_NEAR_THRESHOLD
                                        ? TX_ALPHA_FAR
                                        : TX_ALPHA_NEAR;

                        if (!Double.isFinite(filteredTx)) {
                            filteredTx = rawTx;
                        } else {
                            filteredTx += alpha
                                    * (rawTx - filteredTx);
                        }

                        // Error visual con zona muerta
                        double errorTx = 0.0;

                        if (Math.abs(filteredTx)
                                > TX_DEADBAND) {

                            errorTx =
                                    Math.signum(filteredTx)
                                            * (Math.abs(filteredTx)
                                            - TX_DEADBAND);
                        }

                        // Solo corregir cuando existe
                        // error visual significativo.
                        //
                        // Importante: si tx esta centrado,
                        // no perseguir turret.getAngle().
                        if (errorTx != 0.0) {

                            double visualError =
                                    TX_TO_TURRET_SIGN
                                            * errorTx
                                            * VISION_KP;

                            // Objetivo observado por vision
                            double visualTarget =
                                    turret.getAngle()
                                            + visualError;

                            // Compensar el heading lead
                            // para comparar objetivos
                            // en la misma referencia.
                            double targetWithoutLead =
                                    visualTarget
                                            - headingLead;

                            // Error entre el objetivo
                            // visual y el de Pedro.
                            //
                            // No sumamos visualError
                            // indefinidamente.
                            double targetDifference =
                                    targetWithoutLead
                                            - baseTarget;

                            double gain =
                                    mode == Mode.COMBINED
                                            ? FUSION_GAIN
                                            : VISION_ONLY_GAIN;

                            double requestedCorrection =
                                    targetDifference
                                            * gain;

                            double maxCorrection =
                                    VISION_RATE_DEG_S
                                            * visionDt;

                            double correction =
                                    Range.clip(
                                            requestedCorrection,
                                            -maxCorrection,
                                            maxCorrection
                                    );

                            baseTarget += correction;

                            status =
                                    mode == Mode.COMBINED
                                            ? "FAST_FUSION"
                                            : "VISION_TRACKING";

                        } else {

                            // Conservar el objetivo
                            // cuando tx esta centrado.
                            status = "VISION_CENTERED";
                        }
                    }

                } else {

                    status = "INVALID_TX";
                }

            } else {

                filteredTx = Double.NaN;

                status = vision.hasTarget()
                        ? "STALE_VISION"
                        : "NO_TAG";
            }

        } else {

            filteredTx = Double.NaN;
            status = "PEDRO_ONLY";
        }

        // =====================================
        // 3. LIMITES MECANICOS
        // =====================================

        double min =
                TurretSubsystemV4.MIN_ANGLE_DEG;

        double max =
                TurretSubsystemV4.MAX_ANGLE_DEG;

        double requestedTarget =
                baseTarget + headingLead;

        double desiredTarget =
                Range.clip(
                        requestedTarget,
                        min,
                        max
                );

        if (requestedTarget != desiredTarget) {

            baseTarget =
                    desiredTarget - headingLead;

            status = "ANGLE_LIMIT";
        }

        // =====================================
        // 4. PERFIL DE MOVIMIENTO
        // =====================================

        double remaining =
                desiredTarget - smoothTarget;

        double acceleration =
                Math.max(1.0, MAX_TARGET_ACCEL);

        double maxSpeed =
                Math.max(1.0, MAX_TARGET_SPEED);

        double brakingSpeed =
                Math.sqrt(
                        2.0
                                * acceleration
                                * Math.abs(remaining)
                );

        double desiredVelocity =
                Math.signum(remaining)
                        * Math.min(
                        maxSpeed,
                        brakingSpeed
                );

        double maxDeltaVelocity =
                acceleration * dt;

        smoothVelocity += Range.clip(
                desiredVelocity - smoothVelocity,
                -maxDeltaVelocity,
                maxDeltaVelocity
        );

        double step = smoothVelocity * dt;

        if (Math.signum(step)
                == Math.signum(remaining)
                && Math.abs(step)
                >= Math.abs(remaining)) {

            smoothTarget = desiredTarget;

            // Evitar saltos grandes de
            // posicion al alcanzar el target.
            // El feedforward se filtra abajo.
            smoothVelocity = 0.0;

        } else {

            smoothTarget += step;
        }

        smoothTarget = Range.clip(
                smoothTarget,
                min,
                max
        );

        if ((smoothTarget >= max
                && smoothVelocity > 0.0)
                || (smoothTarget <= min
                && smoothVelocity < 0.0)) {

            smoothVelocity = 0.0;
        }

        // =====================================
        // 5. FEEDFORWARD SUAVIZADO
        // =====================================

        double ffTime =
                Math.max(0.005, FF_FILTER_TIME_S);

        double ffAlpha =
                1.0 - Math.exp(-dt / ffTime);

        filteredFF += ffAlpha
                * (smoothVelocity - filteredFF);

        if ((smoothTarget >= max
                && filteredFF > 0.0)
                || (smoothTarget <= min
                && filteredFF < 0.0)) {

            filteredFF = 0.0;
        }

        turret.setVelocityFeedforward(filteredFF);
        turret.setTarget(smoothTarget);
        turret.update();
    }
}

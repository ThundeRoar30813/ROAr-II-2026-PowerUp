
package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

public class TurretAimControllerV7 {

    private final DriveSubsystem drive;
    private final TurretSubsystemV4 turret;
    private final TurretVisionV5 vision;

    // Signos que ya funcionaban
    public static double HEADING_COMP_SIGN = 1.0;
    public static double TX_TO_TURRET_SIGN = -1.0;

    // Limelight
    public static double TX_FILTER_ALPHA = 0.45;
    public static double TX_DEADBAND = 0.75;
    public static double VISION_KP = 1.10;
    public static double VISION_RATE_DEG_S = 300.0;
    public static double MIN_VISION_INTERVAL_S = 0.025;
    public static double MAX_VALID_TX = 27.0;
    public static double MAX_VISION_AGE_MS = 150.0;

    // Compensacion de Pedro
    public static double RATE_FILTER_ALPHA = 0.30;
    public static double HEADING_LEAD_TIME_S = 0.025;
    public static double MAX_HEADING_LEAD_DEG = 6.0;
    public static double MAX_HEADING_STEP = 45.0;
    public static double MAX_CHASSIS_RATE = 350.0;

    // Perfil de movimiento
    public static double MAX_TARGET_SPEED = 300.0;
    public static double MAX_TARGET_ACCEL = 1800.0;

    // Reducir correccion visual si el motor va atrasado
    public static double LARGE_MOTOR_ERROR_DEG = 8.0;
    public static double MOVING_VISION_SCALE = 0.85;

    private boolean enabled = false;

    private double lastHeading = 0.0;
    private double baseTarget = 0.0;
    private double visionOffset = 0.0;

    private double smoothTarget = 0.0;
    private double smoothVelocity = 0.0;

    private double filteredTx = Double.NaN;
    private double chassisRate = 0.0;
    private double headingLead = 0.0;
    private double filteredFeedforward = 0.0;

    private double lastVisionTime = 0.0;

    private int selectedTag = -1;
    private int previousTag = -1;

    private long lastNs = 0;

    private String status = "DISABLED";

    public TurretAimControllerV7(
            DriveSubsystem drive,
            TurretSubsystemV4 turret,
            TurretVisionV5 vision) {

        this.drive = drive;
        this.turret = turret;
        this.vision = vision;
    }

    public void setSelectedTag(int id) {
        selectedTag = id;
    }

    public void enable() {

        if (turret.isFault()) {
            enabled = false;
            status = "TURRET_FAULT";
            return;
        }

        turret.enableHold();

        if (!turret.isEnabled()) {
            enabled = false;
            status = "NOT_ENABLED";
            return;
        }

        enabled = true;

        lastHeading = drive.getHeadingDegrees();

        baseTarget = turret.getAngle();
        smoothTarget = baseTarget;
        smoothVelocity = 0.0;

        visionOffset = 0.0;
        filteredTx = Double.NaN;

        chassisRate = 0.0;
        headingLead = 0.0;
        filteredFeedforward = 0.0;

        previousTag = selectedTag;

        lastNs = System.nanoTime();
        lastVisionTime = 0.0;

        turret.setVelocityFeedforward(0.0);
        turret.setTarget(smoothTarget);

        status = "ACTIVE";
    }

    public void disable() {

        enabled = false;

        smoothVelocity = 0.0;
        filteredFeedforward = 0.0;

        turret.setVelocityFeedforward(0.0);
        turret.disable();

        status = "DISABLED";
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

    public double getVisionOffset() {
        return visionOffset;
    }

    public double getChassisAngularVelocity() {
        return chassisRate;
    }

    public double getHeadingLead() {
        return headingLead;
    }

    private double angleDifference(
            double current,
            double previous) {

        double diff = current - previous;

        while (diff > 180.0) diff -= 360.0;
        while (diff < -180.0) diff += 360.0;

        return diff;
    }

    public void update() {

        if (!enabled) {
            return;
        }

        if (turret.isFault()) {
            disable();
            status = "TURRET_FAULT";
            return;
        }

        long now = System.nanoTime();

        double dt = (now - lastNs) / 1e9;
        lastNs = now;

        boolean validDt =
                Double.isFinite(dt)
                        && dt > 0.001
                        && dt <= 0.15;

        if (!validDt) {
            dt = 0.02;
            chassisRate = 0.0;
        }

        // =====================================
        // 1. COMPENSACION DE PEDRO
        // =====================================

        double heading = drive.getHeadingDegrees();

        if (!Double.isFinite(heading)) {
            turret.setVelocityFeedforward(0);
            turret.update();
            status = "INVALID_HEADING";
            return;
        }

        double deltaHeading =
                angleDifference(heading, lastHeading);

        lastHeading = heading;

        boolean validStep =
                validDt
                        && Math.abs(deltaHeading)
                        <= MAX_HEADING_STEP;

        if (validStep) {
            baseTarget +=
                    HEADING_COMP_SIGN * deltaHeading;
        }

        double measuredRate = 0.0;

        if (validStep) {
            measuredRate = Range.clip(
                    deltaHeading / dt,
                    -MAX_CHASSIS_RATE,
                    MAX_CHASSIS_RATE
            );
        }

        chassisRate += RATE_FILTER_ALPHA
                * (measuredRate - chassisRate);

        double turretRate =
                HEADING_COMP_SIGN * chassisRate;

        headingLead = Range.clip(
                turretRate * HEADING_LEAD_TIME_S,
                -MAX_HEADING_LEAD_DEG,
                MAX_HEADING_LEAD_DEG
        );

        // =====================================
        // 2. CAMBIO DE APRILTAG
        // =====================================

        if (selectedTag != previousTag) {

            previousTag = selectedTag;

            filteredTx = Double.NaN;
            lastVisionTime = 0.0;
            visionOffset = 0.0;
        }

        // =====================================
        // 3. CORRECCION VISUAL
        // =====================================

        double currentTime = now / 1e9;

        boolean visionReady =
                currentTime - lastVisionTime
                        >= MIN_VISION_INTERVAL_S;

        double age = vision.getAgeMs();

        boolean fresh =
                Double.isFinite(age)
                        && age >= 0
                        && age <= MAX_VISION_AGE_MS;

        if (vision.hasTarget() && fresh) {

            double rawTx = vision.getTx();

            if (Double.isFinite(rawTx)
                    && Math.abs(rawTx) <= MAX_VALID_TX) {

                if (visionReady) {

                    double visionDt =
                            lastVisionTime == 0.0
                                    ? MIN_VISION_INTERVAL_S
                                    : Range.clip(
                                    currentTime - lastVisionTime,
                                    0.001,
                                    0.10
                            );

                    lastVisionTime = currentTime;

                    if (!Double.isFinite(filteredTx)) {
                        filteredTx = rawTx;
                    } else {
                        filteredTx += TX_FILTER_ALPHA
                                * (rawTx - filteredTx);
                    }

                    double errorTx = 0.0;

                    if (Math.abs(filteredTx) > TX_DEADBAND) {
                        errorTx =
                                Math.signum(filteredTx)
                                        * (Math.abs(filteredTx)
                                        - TX_DEADBAND);
                    }

                    double scale = 1.0;

                    if (Math.abs(turret.getError())
                            > LARGE_MOTOR_ERROR_DEG) {
                        scale = MOVING_VISION_SCALE;
                    }

                    double visualError =
                            TX_TO_TURRET_SIGN
                                    * errorTx
                                    * VISION_KP
                                    * scale;

                    double desiredTarget =
                            turret.getAngle() + visualError;

                    double maxCorrection =
                            VISION_RATE_DEG_S * visionDt;

                    double correction = Range.clip(
                            desiredTarget - baseTarget,
                            -maxCorrection,
                            maxCorrection
                    );

                    baseTarget += correction;

                    status = Math.abs(errorTx) > 0.0
                            ? "VISION_TRACKING"
                            : "VISION_STABLE";
                }

            } else {
                status = "INVALID_TX";
            }

        } else {

            filteredTx = Double.NaN;

            status = vision.hasTarget()
                    ? "STALE_FRAME"
                    : "HEADING_ONLY";
        }

        // =====================================
        // 4. OBJETIVO SOLICITADO
        // =====================================

//        double minAngle =
//                TurretSubsystemV4.MIN_ANGLE_DEG;
//
//        double maxAngle =
//                TurretSubsystemV4.MAX_ANGLE_DEG;

//        double requestedTarget =
//                baseTarget + headingLead;
//
//        double desiredTarget = Range.clip(
//                requestedTarget,
//                minAngle,
//                maxAngle
//        );
//
//        if (desiredTarget != requestedTarget) {
//            baseTarget = desiredTarget - headingLead;
//            status = "ANGLE_LIMIT";
//        }

        // =====================================
        // 5. PERFIL SUAVE DE MOVIMIENTO
        // =====================================

//        double remaining =
//                desiredTarget - smoothTarget;

        double acceleration =
                Math.max(1.0, MAX_TARGET_ACCEL);

        double maxSpeed =
                Math.max(1.0, MAX_TARGET_SPEED);

        // Velocidad que permite frenar
        // antes de llegar al objetivo
//        double brakingSpeed = Math.sqrt(
//                2.0 * acceleration
//                        * Math.abs(remaining)
//        );
//
//        double desiredVelocity =
//                Math.signum(remaining)
//                        * Math.min(maxSpeed, brakingSpeed);

        // Limitar aceleracion por ciclo
        double maxDeltaVelocity =
                acceleration * dt;

//        smoothVelocity += Range.clip(
//                desiredVelocity - smoothVelocity,
//                -maxDeltaVelocity,
//                maxDeltaVelocity
//        );
//
//        double step = smoothVelocity * dt;
//
//        // No rebasar el objetivo cuando
//        // ya estamos frenando
//        if (Math.signum(step)
//                == Math.signum(remaining)
//                && Math.abs(step)
//                >= Math.abs(remaining)) {
//
//            smoothTarget = desiredTarget;
//            smoothVelocity = 0.0;
//
//        } else {
//
//            smoothTarget += step;
//        }
//
//        smoothTarget = Range.clip(
//                smoothTarget,
//                minAngle,
//                maxAngle
//        );
//
//        // No seguir empujando contra los limites
//        if ((smoothTarget >= maxAngle
//                && smoothVelocity > 0)
//                ||
//                (smoothTarget <= minAngle
//                        && smoothVelocity < 0)) {
//
//            smoothVelocity = 0.0;
//        }

        // =====================================
        // 6. FEEDFORWARD SUAVIZADO
        // =====================================

        // Usar la velocidad de la referencia
        // suavizada para evitar que el motor
        // reciba cambios bruscos de potencia
        double requestedFF = smoothVelocity;

        // Filtro temporal para el feedforward
        double ffAlpha =
                1.0 - Math.exp(-dt / 0.035);

        filteredFeedforward += ffAlpha
                * (requestedFF - filteredFeedforward);
//
//        if ((smoothTarget >= maxAngle
//                && filteredFeedforward > 0)
//                ||
//                (smoothTarget <= minAngle
//                        && filteredFeedforward < 0)) {
//
//            filteredFeedforward = 0.0;
//        }
//
//        turret.setVelocityFeedforward(
//                filteredFeedforward
//        );

        // =====================================
        // 7. ENVIAR AL MOTOR
        // =====================================

        turret.setTarget(smoothTarget);
        turret.update();
    }
}

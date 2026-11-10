
package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

public class TurretAimControllerV6 {

    private final DriveSubsystem drive;
    private final TurretSubsystemV4 turret;
    private final TurretVisionV5 vision;

    // Signos ya comprobados en tu robot
    public static double HEADING_COMP_SIGN = 1.0;
    public static double TX_TO_TURRET_SIGN = -1.0;

    // =====================================
    // VISION: REDUCIR OSCILACION
    // =====================================

    public static double TX_DEADBAND = 1.0;
    public static double TX_FILTER_ALPHA = 0.55;

    public static double VISION_KP = 1.05;
    public static double VISION_RATE_DEG_S = 200.0;

    public static double MAX_VISION_OFFSET = 50.0;
    public static double MAX_VALID_TX = 27.0;

    // =====================================
    // CHASSIS ROTATION
    // =====================================

    public static double HEADING_VELOCITY_ALPHA = 0.55;

    // Adelanto inicial moderado.
    // Aumentar solo si no produce sobrepaso.
    public static double HEADING_LEAD_TIME_S = 0.065;
    public static double MAX_HEADING_LEAD_DEG = 10.0;

    public static double MAX_HEADING_STEP = 55.0;
    public static double MAX_CHASSIS_RATE = 450.0;

    private boolean enabled = false;

    private double lastHeading = 0;
    private double baseTarget = 0;
    private double visionOffset = 0;

    private double filteredTx = Double.NaN;
    private double filteredAngularVelocity = 0;

    private int selectedTag = -1;
    private int previousTag = -1;

    private long lastNs = 0;

    private String status = "DISABLED";

    public TurretAimControllerV6(
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
            status = "TURRET_FAULT";
            return;
        }

        turret.enableHold();

        if (!turret.isEnabled()) {
            status = "NOT_ENABLED";
            return;
        }

        enabled = true;

        lastHeading = drive.getHeadingDegrees();
        baseTarget = turret.getAngle();

        visionOffset = 0;
        filteredTx = Double.NaN;
        filteredAngularVelocity = 0;

        previousTag = selectedTag;

        lastNs = System.nanoTime();

        turret.setVelocityFeedforward(0);
        turret.setTarget(baseTarget);

        status = "HYBRID_ACTIVE";
    }

    public void disable() {

        enabled = false;
        turret.setVelocityFeedforward(0);
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
        return filteredAngularVelocity;
    }

    private double angleDifference(
            double current,
            double previous) {

        double difference = current - previous;

        while (difference > 180) {
            difference -= 360;
        }

        while (difference < -180) {
            difference += 360;
        }

        return difference;
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
                Double.isFinite(dt) &&
                        dt > 0.001 &&
                        dt <= 0.15;

        if (!validDt) {
            dt = 0.02;
            filteredAngularVelocity = 0;
        }

        // =====================================
        // 1. PEDRO HEADING
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

        boolean validHeadingStep =
                Math.abs(deltaHeading) <= MAX_HEADING_STEP;

        if (validHeadingStep) {

            baseTarget +=
                    HEADING_COMP_SIGN * deltaHeading;
        }

        // =====================================
        // 2. VELOCIDAD ANGULAR DEL CHASIS
        // =====================================

        double chassisRate = 0;

        if (validDt && validHeadingStep) {

            chassisRate = Range.clip(
                    deltaHeading / dt,
                    -MAX_CHASSIS_RATE,
                    MAX_CHASSIS_RATE
            );
        }

        filteredAngularVelocity +=
                HEADING_VELOCITY_ALPHA *
                        (chassisRate - filteredAngularVelocity);

        double turretDesiredRate =
                HEADING_COMP_SIGN *
                        filteredAngularVelocity;

        // Enviar velocidad deseada directamente
        // al controlador del motor
        turret.setVelocityFeedforward(
                turretDesiredRate
        );

        // =====================================
        // 3. ADELANTO DE HEADING
        // =====================================

        double headingLead = Range.clip(
                turretDesiredRate *
                        HEADING_LEAD_TIME_S,
                -MAX_HEADING_LEAD_DEG,
                MAX_HEADING_LEAD_DEG
        );

        // =====================================
        // 4. CAMBIO DE APRILTAG
        // =====================================

        if (selectedTag != previousTag) {

            previousTag = selectedTag;

            baseTarget += visionOffset;
            visionOffset = 0;

            filteredTx = Double.NaN;
        }

        // =====================================
        // 5. LIMELIGHT
        // =====================================

        if (vision.hasTarget()) {

            double rawTx = vision.getTx();

            if (Double.isFinite(rawTx) &&
                    Math.abs(rawTx) <= MAX_VALID_TX) {

                if (!Double.isFinite(filteredTx)) {

                    filteredTx = rawTx;

                } else {

                    filteredTx +=
                            TX_FILTER_ALPHA *
                                    (rawTx - filteredTx);
                }

                if (Math.abs(filteredTx) >
                        TX_DEADBAND) {

                    // La camara mide el error
                    // desde el angulo ACTUAL.
                    double desiredAbsolute =
                            turret.getAngle()
                                    + TX_TO_TURRET_SIGN
                                    * filteredTx
                                    * VISION_KP;

                    double desiredOffset =
                            Range.clip(
                                    desiredAbsolute - baseTarget,
                                    -MAX_VISION_OFFSET,
                                    MAX_VISION_OFFSET
                            );

                    double maxStep =
                            VISION_RATE_DEG_S * dt;

                    visionOffset += Range.clip(
                            desiredOffset - visionOffset,
                            -maxStep,
                            maxStep
                    );

                    status = "HYBRID_TRACKING";

                } else {

                    // Conservar la referencia
                    // cuando el tag esta centrado
                    status = "VISION_ALIGNED";
                }

            } else {

                status = "INVALID_TX";
            }

        } else {

            status = "HEADING_ONLY";
        }

        // =====================================
        // 6. OBJETIVO FINAL
        // =====================================

        double requestedTarget =
                baseTarget
                        + visionOffset
                        + headingLead;

        double target = Range.clip(
                requestedTarget,
                TurretSubsystemV4.MIN_ANGLE_DEG,
                TurretSubsystemV4.MAX_ANGLE_DEG
        );

        if (target != requestedTarget) {

            // No seguir acumulando objetivos
            // fuera del recorrido mecanico.
            baseTarget =
                    target - visionOffset - headingLead;

            status = "ANGLE_LIMIT";
        }

        // Si ya estamos en el tope, evitar
        // que el feedforward siga empujando
        // en direccion prohibida.
        if ((target >= TurretSubsystemV4.MAX_ANGLE_DEG
                && turretDesiredRate > 0) ||
                (target <= TurretSubsystemV4.MIN_ANGLE_DEG
                        && turretDesiredRate < 0)) {

            turret.setVelocityFeedforward(0);
        }

        turret.setTarget(target);
        turret.update();
    }
}

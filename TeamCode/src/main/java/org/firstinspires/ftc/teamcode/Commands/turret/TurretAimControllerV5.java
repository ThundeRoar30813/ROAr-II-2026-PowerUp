
package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

public class TurretAimControllerV5 {

    private final DriveSubsystem drive;
    private final TurretSubsystemV4 turret;
    private final TurretVisionV5 vision;

    // Sentido de compensacion del heading
    public static double HEADING_COMP_SIGN = 1.0;

    // Sentido visual previamente funcional
    public static double TX_TO_TURRET_SIGN = -1.0;

    public static double TX_DEADBAND = 0.8;
    public static double TX_FILTER_ALPHA = 0.70;

    public static double VISION_KP = 1.70;
    public static double VISION_RATE_DEG_S = 300.0;


    public static double HEADING_LEAD_TIME_S = 0.09;
    public static double MAX_HEADING_LEAD_DEG = 15.0;


    public static double MAX_VISION_OFFSET = 35.0;
    public static double MAX_VALID_TX = 27.0;
    public static double MAX_HEADING_STEP = 45.0;

    private boolean enabled = false;

    private double lastHeading = 0;
    private double baseTarget = 0;
    private double visionOffset = 0;

    private double filteredTx = Double.NaN;

    private int selectedTag = -1;
    private int previousTag = -1;

    private long lastNs = 0;

    private String status = "DISABLED";

    public TurretAimControllerV5(
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

        visionOffset = 0;
        filteredTx = Double.NaN;

        previousTag = selectedTag;

        lastNs = System.nanoTime();

        turret.setTarget(baseTarget);

        status = "HYBRID_ACTIVE";
    }

    public void disable() {
        enabled = false;
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

        if (!Double.isFinite(dt) ||
                dt <= 0 || dt > 0.15) {
            dt = 0.02;
        }

        // ======================================
        // 1. PEDRO HEADING COMPENSATION
        // ======================================

        double heading = drive.getHeadingDegrees();

        if (!Double.isFinite(heading)) {
            status = "INVALID_HEADING";
            turret.update();
            return;
        }

        double deltaHeading =
                angleDifference(heading, lastHeading);

        lastHeading = heading;

        if (Math.abs(deltaHeading) <=
                MAX_HEADING_STEP) {

            baseTarget +=
                    HEADING_COMP_SIGN * deltaHeading;

        } else {

            status = "HEADING_JUMP";
        }


        double chassisAngularVelocity = deltaHeading / dt;

        double headingLead = Range.clip(
                HEADING_COMP_SIGN
                        * chassisAngularVelocity
                        * HEADING_LEAD_TIME_S,
                -MAX_HEADING_LEAD_DEG,
                MAX_HEADING_LEAD_DEG
        );


        // ======================================
        // 2. CAMBIO DE APRILTAG
        // ======================================

        if (selectedTag != previousTag) {

            previousTag = selectedTag;

            // Mantener objetivo continuo
            baseTarget += visionOffset;
            visionOffset = 0;

            filteredTx = Double.NaN;
        }

        // ======================================
        // 3. LIMELIGHT CORRECTION
        // ======================================

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

                double desiredOffset = visionOffset;

                if (Math.abs(filteredTx) >
                        TX_DEADBAND) {

                    // Angulo absoluto solicitado
                    // basado en medicion actual
                    double desiredAbsolute =
                            turret.getAngle()
                                    + TX_TO_TURRET_SIGN
                                    * filteredTx
                                    * VISION_KP;

                    desiredOffset =
                            desiredAbsolute - baseTarget;

                    desiredOffset = Range.clip(
                            desiredOffset,
                            -MAX_VISION_OFFSET,
                            MAX_VISION_OFFSET
                    );

                    status = "HYBRID_TRACKING";

                } else {

                    status = "VISION_ALIGNED";
                }

                double maxStep =
                        VISION_RATE_DEG_S * dt;

                visionOffset += Range.clip(
                        desiredOffset - visionOffset,
                        -maxStep,
                        maxStep
                );

            } else {

                status = "INVALID_TX";
            }

        } else {

            // Sin imagen: conservar el ultimo
            // objetivo relativo al campo y
            // compensar el giro del chasis.
            status = "HEADING_ONLY";
        }

        // ======================================
        // 4. TARGET Y LIMITES
        // ======================================

        double target =
                baseTarget + visionOffset;

//        double clippedTarget = Range.clip(
//                target,
//                TurretSubsystemV4.MIN_ANGLE_DEG,
//                TurretSubsystemV4.MAX_ANGLE_DEG
//        );

//        if (clippedTarget != target) {
//
//            // Evitar acumulacion de comandos
//            // fuera del recorrido permitido
//            baseTarget =
//                    clippedTarget - visionOffset;
//
//            status = "ANGLE_LIMIT";
//        }

//        turret.setTarget(clippedTarget);
        turret.update();
    }
}

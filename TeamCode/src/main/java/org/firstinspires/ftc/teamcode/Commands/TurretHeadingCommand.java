package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

public class TurretHeadingCommand {

    private final DriveSubsystem drive;
    private final TurretSubsystem turret;

    /*
     * Heading que tenía Pedro cuando activamos
     * el tracking.
     */
    private double startingRobotHeadingDeg = 0;

    private boolean enabled = false;

    public TurretHeadingCommand(
            DriveSubsystem drive,
            TurretSubsystem turret
    ) {

        this.drive = drive;
        this.turret = turret;
    }

    // =========================================================
    // ACTIVAR
    // =========================================================

    public void enable() {

        /*
         * Guardamos hacia dónde estaba mirando el robot
         * en el momento de activar el tracking.
         */
        startingRobotHeadingDeg =
                drive.getHeadingDegrees();

        /*
         * Consideramos la posición actual de la torreta
         * como nuestro 0°.
         */
        //turret.resetZero();

        enabled = true;
    }

    // =========================================================
    // DESACTIVAR
    // =========================================================

    public void disable() {

        enabled = false;

        turret.stop();
    }

    // =========================================================
    // UPDATE
    // =========================================================

    public void execute() {

        if (!enabled) {
            return;
        }

        double robotHeading =
                drive.getHeadingDegrees();

        /*
         * ¿Cuánto ha girado el robot desde que
         * activamos el tracking?
         */


        double robotRotation =
                angleDifferenceDegrees(
                        robotHeading,
                        startingRobotHeadingDeg
                );

        /*
         * La torreta hace exactamente lo contrario.
         *
         * Robot +30°
         * Turret -30°
         */
        double turretTarget =
                robotRotation;

        turret.setTargetAngleDeg(
                turretTarget
        );

        turret.update();
    }

    // =========================================================
    // NORMALIZAR ÁNGULO
    // =========================================================

    private double angleDifferenceDegrees(
            double current,
            double reference
    ) {

        double difference =
                current - reference;

        while (difference > 180) {
            difference -= 360;
        }

        while (difference < -180) {
            difference += 360;
        }

        return difference;
    }

    // =========================================================
    // GETTERS
    // =========================================================

    public boolean isEnabled() {

        return enabled;
    }

    public double getStartingRobotHeadingDeg() {

        return startingRobotHeadingDeg;
    }
}
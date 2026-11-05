
package org.firstinspires.ftc.teamcode.Commands.turret;

public class TurretSafetySimulation {

    public static void main(String[] args) {

        TurretSafetyManager safety =
                new TurretSafetyManager(
                        -115.0,
                        120.0,
                        5.0
                );

        double[] requestedAngles = {
                0.0,
                45.0,
                90.0,
                120.0,
                180.0,
                -90.0,
                -180.0,
                -270.0
        };

        double currentAngle = 0.0;

        for (double requested : requestedAngles) {

            double safe = safety.calculateSafeTarget(
                    requested,
                    currentAngle
            );

            System.out.printf(
                    "Requested: %7.1f | Safe: %7.1f | Status: %s%n",
                    requested,
                    safe,
                    safety.getStatus()
            );
        }
    }
}

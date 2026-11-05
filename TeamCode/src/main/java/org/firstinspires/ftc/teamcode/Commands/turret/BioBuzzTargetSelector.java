
package org.firstinspires.ftc.teamcode.Commands.turret;

public class BioBuzzTargetSelector {

    public enum Target {
        TAG_34,
        TAG_37
    }

    private final double centerX;
    private final double hysteresis;

    private Target currentTarget = Target.TAG_37;

    public BioBuzzTargetSelector(
            double centerX,
            double hysteresis
    ) {
        this.centerX = centerX;
        this.hysteresis = Math.max(0, hysteresis);
    }

    public Target update(double robotX) {

        // Convencion provisional:
        // X menor = mitad roja
        // X mayor = mitad azul

        if (currentTarget == Target.TAG_37) {

            if (robotX > centerX + hysteresis) {
                currentTarget = Target.TAG_34;
            }

        } else {

            if (robotX < centerX - hysteresis) {
                currentTarget = Target.TAG_37;
            }
        }

        return currentTarget;
    }

    public Target getCurrentTarget() {
        return currentTarget;
    }

    public void reset(Target initialTarget) {
        currentTarget = initialTarget;
    }
}

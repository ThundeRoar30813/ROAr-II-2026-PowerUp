
package org.firstinspires.ftc.teamcode.Commands.turret;

public class ZoneTargetSelector {

    public enum Alliance {
        RED,
        BLUE
    }

    private Alliance alliance = Alliance.RED;

    private boolean right = false;
    private boolean upper = false;

    public static double CENTER_X = 72.0;
    public static double CENTER_Y = 72.0;
    public static double HYSTERESIS_IN = 3.0;

    public void setAlliance(Alliance value) {
        alliance = value;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void update(double x, double y) {

        if (!Double.isFinite(x) ||
                !Double.isFinite(y)) return;

        if (right && x < CENTER_X - HYSTERESIS_IN) {
            right = false;
        } else if (!right &&
                x > CENTER_X + HYSTERESIS_IN) {
            right = true;
        }

        if (upper && y < CENTER_Y - HYSTERESIS_IN) {
            upper = false;
        } else if (!upper &&
                y > CENTER_Y + HYSTERESIS_IN) {
            upper = true;
        }
    }

    public int getTagId() {

        if (alliance == Alliance.RED) {

            if (upper) {
                return right ? 33 : 30;
            }

            return right ? 34 : 37;
        }

        if (upper) {
            return right ? 45 : 42;
        }

        return right ? 38 : 41;
    }

    public String getZone() {

        return (upper ? "UPPER" : "LOWER")
                + "_"
                + (right ? "RIGHT" : "LEFT");
    }
}

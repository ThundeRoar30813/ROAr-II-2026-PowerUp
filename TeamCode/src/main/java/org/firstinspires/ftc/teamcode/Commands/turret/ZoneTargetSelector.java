
package org.firstinspires.ftc.teamcode.Commands.turret;

public class ZoneTargetSelector {

    public enum Alliance {
        RED,
        BLUE
    }

    public static double CENTER_X = 72.0;
    public static double CENTER_Y = 72.0;
    public static double HYSTERESIS_IN = 3.0;

    // Orientacion provisional para tu campo
    public static boolean INVERT_Y = true;

    private Alliance alliance = Alliance.RED;

    private boolean automatic = true;
    private boolean right = false;
    private boolean upper = false;

    private int manualIndex = 0;

    public void setAlliance(Alliance alliance) {
        if (alliance != null) {
            this.alliance = alliance;
        }
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void setAutomatic(boolean automatic) {
        this.automatic = automatic;
    }

    public boolean isAutomatic() {
        return automatic;
    }

    public String getMode() {
        return automatic ? "AUTO" : "MANUAL";
    }

    // Inferior izquierda
    public void selectLeft() {
        automatic = false;
        manualIndex = 0;
    }

    // Inferior derecha
    public void selectRight() {
        automatic = false;
        manualIndex = 1;
    }

    // Superior izquierda
    public void selectUpperLeft() {
        automatic = false;
        manualIndex = 2;
    }

    // Superior derecha
    public void selectUpperRight() {
        automatic = false;
        manualIndex = 3;
    }

    public void update(double x, double y) {

        if (!Double.isFinite(x) ||
                !Double.isFinite(y)) {
            return;
        }

        if (right) {

            if (x < CENTER_X - HYSTERESIS_IN) {
                right = false;
            }

        } else if (x > CENTER_X + HYSTERESIS_IN) {

            right = true;
        }

        double adjustedY = INVERT_Y ? -y : y;
        double adjustedCenter =
                INVERT_Y ? -CENTER_Y : CENTER_Y;

        if (upper) {

            if (adjustedY <
                    adjustedCenter - HYSTERESIS_IN) {
                upper = false;
            }

        } else if (adjustedY >
                adjustedCenter + HYSTERESIS_IN) {

            upper = true;
        }
    }

    public int getTagId() {

        int index;

        if (automatic) {

            if (upper) {
                index = right ? 3 : 2;
            } else {
                index = right ? 1 : 0;
            }

        } else {

            index = manualIndex;
        }

        if (alliance == Alliance.RED) {

            switch (index) {
                case 0: return 37;
                case 1: return 34;
                case 2: return 30;
                default: return 33;
            }
        }

        switch (index) {
            case 0: return 41;
            case 1: return 38;
            case 2: return 42;
            default: return 45;
        }
    }

    public String getZone() {

        return (upper ? "UPPER" : "LOWER")
                + "_"
                + (right ? "RIGHT" : "LEFT");
    }
}

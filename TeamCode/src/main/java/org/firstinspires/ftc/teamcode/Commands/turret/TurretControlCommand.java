
package org.firstinspires.ftc.teamcode.Commands.turret;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

public class TurretControlCommand {

    // =========================================
    // DIVISION DEL CAMPO - PEDRO PATHING
    // =========================================

    public static double FIELD_MID_X = 72.0;
    public static double FIELD_MID_Y = 72.0;

    public static double SPLIT_HYSTERESIS_IN = 1.5;

    // ZONA INFERIOR
    public static int LOWER_LEFT_TAG_ID = 37;
    public static int LOWER_RIGHT_TAG_ID = 35;

    // ZONA SUPERIOR
    public static int UPPER_LEFT_TAG_ID = 30;
    public static int UPPER_RIGHT_TAG_ID = 33;

    public static double TEST_POWER = 1.0;

    private final DriveSubsystem drive;
    private final TurretSubsystemV4 turret;
    private final TurretVisionV5 vision;
    private final TurretAimControllerV77 aim;

    private int activeTagId = LOWER_LEFT_TAG_ID;

    private boolean tracking = false;
    private boolean home = false;

    private boolean upperHalf = false;
    private boolean rightHalf = false;

    private boolean lastB = false;
    private boolean lastStart = false;
    private boolean lastBack = false;

    public TurretControlCommand(
            DriveSubsystem drive,
            TurretSubsystemV4 turret,
            TurretVisionV5 vision,
            TurretAimControllerV77 aim) {

        this.drive = drive;
        this.turret = turret;
        this.vision = vision;
        this.aim = aim;

        turret.maxPower = TEST_POWER;

        aim.setMode(
                TurretAimControllerV77.Mode.COMBINED
        );
    }

    public void start() {
        vision.start();
        updateAutoTag();
        vision.setSelectedTag(activeTagId);
    }

    // =========================================
    // SELECCION AUTOMATICA DE APRILTAGS
    // =========================================

    private void updateAutoTag() {

        double x = drive.getX();
        double y = drive.getY();

        if (!Double.isFinite(x) ||
                !Double.isFinite(y)) {
            return;
        }

        // DIVISION IZQUIERDA / DERECHA
        if (x < FIELD_MID_X - SPLIT_HYSTERESIS_IN) {

            rightHalf = false;

        } else if (x > FIELD_MID_X + SPLIT_HYSTERESIS_IN) {

            rightHalf = true;
        }

        // DIVISION INFERIOR / SUPERIOR
        if (y < FIELD_MID_Y - SPLIT_HYSTERESIS_IN) {

            upperHalf = false;

        } else if (y > FIELD_MID_Y + SPLIT_HYSTERESIS_IN) {

            upperHalf = true;
        }

        // SELECCIONAR APRILTAG
        if (upperHalf) {

            activeTagId = rightHalf
                    ? UPPER_RIGHT_TAG_ID
                    : UPPER_LEFT_TAG_ID;

        } else {

            activeTagId = rightHalf
                    ? LOWER_RIGHT_TAG_ID
                    : LOWER_LEFT_TAG_ID;
        }
    }

    // =========================================
    // EJECUCION PRINCIPAL
    // =========================================

    public void execute(
            boolean trackingButton,
            boolean homeButton,
            boolean resetFaultButton) {

        // 1. ACTUALIZAR ZONA Y APRILTAG
        updateAutoTag();

        vision.setSelectedTag(activeTagId);
        vision.update();

        // 2. TRACKING ON / OFF
        if (trackingButton && !lastB) {

            if (tracking) {

                aim.disable();
                tracking = false;

            } else {

                home = false;

                aim.setMode(
                        TurretAimControllerV77.Mode.COMBINED
                );

                aim.enable();
                tracking = aim.isEnabled();
            }
        }

        lastB = trackingButton;

        // 3. HOME
        if (homeButton && !lastStart) {

            aim.disable();
            tracking = false;

            turret.enableHold();
            turret.setTarget(0);

            home = turret.isEnabled();
        }

        lastStart = homeButton;

        // 4. RESET FAULT
        if (resetFaultButton && !lastBack) {

            aim.disable();

            tracking = false;
            home = false;

            turret.clearFault();
        }

        lastBack = resetFaultButton;

        // 5. ACTUALIZAR TORRETA
        if (tracking) {

            aim.update();

            if (!aim.isEnabled() || turret.isFault()) {
                tracking = false;
            }

        } else if (home) {

            turret.setVelocityFeedforward(0);
            turret.update();

            if (turret.isFault()) {

                home = false;

            } else if (
                    Math.abs(turret.getError()) <= 1.0 &&
                            Math.abs(turret.getVelocity()) < 8.0) {

                turret.disable();
                home = false;
            }

        } else {

            turret.update();
        }
    }

    // =========================================
    // GETTERS
    // =========================================

    public int getActiveTagId() {
        return activeTagId;
    }

    public boolean isTracking() {
        return tracking;
    }

    public boolean isHome() {
        return home;
    }

    public String getFieldSide() {

        return rightHalf
                ? "DERECHA"
                : "IZQUIERDA";
    }

    public String getFieldZone() {

        if (upperHalf) {

            return rightHalf
                    ? "SUPERIOR DERECHA"
                    : "SUPERIOR IZQUIERDA";

        } else {

            return rightHalf
                    ? "INFERIOR DERECHA"
                    : "INFERIOR IZQUIERDA";
        }
    }

    public void stop() {

        aim.disable();
        turret.stop();
        vision.stop();

        tracking = false;
        home = false;
    }
}


package org.firstinspires.ftc.teamcode.Commands.turret;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

public class TurretControlCommand {

    // CONFIGURACION DE ZONAS
    public static double FIELD_MID_X = 72.0;
    public static double SPLIT_HYSTERESIS_IN = 1.5;

    public static int LEFT_SIDE_TAG_ID = 37;
    public static int RIGHT_SIDE_TAG_ID = 35;

    public static double TEST_POWER = 1.0;

    private final DriveSubsystem drive;
    private final TurretSubsystemV4 turret;
    private final TurretVisionV5 vision;
    private final TurretAimControllerV77 aim;

    private int activeTagId = LEFT_SIDE_TAG_ID;

    private boolean tracking = false;
    private boolean home = false;

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

        // COMBINED POR DEFECTO
        aim.setMode(
                TurretAimControllerV77.Mode.COMBINED
        );
    }

    public void start() {
        vision.start();
    }

    // =========================================
    // EJECUCION
    // =========================================
    public void execute(
            boolean trackingButton,
            boolean homeButton,
            boolean resetFaultButton) {

        // 1. AUTOTAG SIEMPRE ACTIVO
        double robotX = drive.getX();

        if (Double.isFinite(robotX)) {

            if (robotX <
                    FIELD_MID_X - SPLIT_HYSTERESIS_IN) {

                activeTagId = LEFT_SIDE_TAG_ID;

            } else if (robotX >
                    FIELD_MID_X + SPLIT_HYSTERESIS_IN) {

                activeTagId = RIGHT_SIDE_TAG_ID;
            }
        }

        vision.setSelectedTag(activeTagId);
        vision.update();

        // 2. TRACKING ON/OFF CON B
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

        // 3. HOME CON START
        if (homeButton && !lastStart) {

            aim.disable();
            tracking = false;

            turret.enableHold();
            turret.setTarget(0);

            home = turret.isEnabled();
        }

        lastStart = homeButton;

        // 4. RESET FAULT CON BACK
        if (resetFaultButton && !lastBack) {

            aim.disable();

            tracking = false;
            home = false;

            turret.clearFault();
        }

        lastBack = resetFaultButton;

        // 5. ACTUALIZAR TURRET
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

        double x = drive.getX();

        if (!Double.isFinite(x)) {
            return "DESCONOCIDA";
        }

        return x < FIELD_MID_X
                ? "IZQUIERDA"
                : "DERECHA";
    }

    public void stop() {

        aim.disable();
        turret.stop();
        vision.stop();

        tracking = false;
        home = false;
    }
}


package org.firstinspires.ftc.teamcode.Commands.shooter;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class ShooterTuningCommand {

    public static double TPS_STEP = 5.0;
    public static double TRIGGER_DEADBAND = 0.15;

    public static double P_STEP = 9.00001;
    public static double I_STEP = 0.000001;
    public static double D_STEP = 0.000001;
    public static double F_STEP = 12.00001;

    private final ShooterSubsystem shooter;

    private int selected = 0; // 0=P, 1=I, 2=D, 3=F

    private boolean lastX = false;
    private boolean lastRT = false;
    private boolean lastLT = false;
    private boolean lastIncrease = false;
    private boolean lastDecrease = false;

    public ShooterTuningCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }

    public void execute(
            boolean x,
            double rightTrigger,
            double leftTrigger,
            boolean selectP,
            boolean selectI,
            boolean selectD,
            boolean selectF,
            boolean increasePIDF,
            boolean decreasePIDF) {

        // SHOOTER ON/OFF
        if (x && !lastX) {
            shooter.toggle();
        }
        lastX = x;

        // CADA FLECHA SELECCIONA UN PARAMETRO
        if (selectP) {
            selected = 0;
        } else if (selectI) {
            selected = 1;
        } else if (selectD) {
            selected = 2;
        } else if (selectF) {
            selected = 3;
        }

        // AUMENTAR / DISMINUIR TPS
        boolean rt = rightTrigger > TRIGGER_DEADBAND;
        boolean lt = leftTrigger > TRIGGER_DEADBAND;

        if (rt && !lastRT) {
            shooter.setTargetTPS(
                    shooter.getTargetTPS() + TPS_STEP
            );
        }

        if (lt && !lastLT) {
            shooter.setTargetTPS(
                    shooter.getTargetTPS() - TPS_STEP
            );
        }

        lastRT = rt;
        lastLT = lt;

        // AJUSTAR EL PIDF SELECCIONADO
        if (increasePIDF && !lastIncrease) {
            adjustPIDF(1);
        }

        if (decreasePIDF && !lastDecrease) {
            adjustPIDF(-1);
        }

        lastIncrease = increasePIDF;
        lastDecrease = decreasePIDF;

        shooter.update();
    }

    private void adjustPIDF(int direction) {

        switch (selected) {

            case 0:
                shooter.kP = Math.max(
                        0, shooter.kP + direction * P_STEP
                );
                break;

            case 1:
                shooter.kI = Math.max(
                        0, shooter.kI + direction * I_STEP
                );
                break;

            case 2:
                shooter.kD = Math.max(
                        0, shooter.kD + direction * D_STEP
                );
                break;

            case 3:
                shooter.kF = Math.max(
                        0, shooter.kF + direction * F_STEP
                );
                break;
        }
    }

    public String getSelectedParameter() {
        switch (selected) {
            case 0: return "P";
            case 1: return "I";
            case 2: return "D";
            default: return "F";
        }
    }

    public void stop() {
        shooter.stop();
    }
}

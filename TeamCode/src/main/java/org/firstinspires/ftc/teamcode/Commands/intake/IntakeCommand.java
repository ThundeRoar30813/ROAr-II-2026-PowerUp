
package org.firstinspires.ftc.teamcode.Commands.intake;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

public class IntakeCommand {

    private final IntakeSubsystem intake;

    private boolean lastAuto = false;
    private boolean lastReset = false;

    public IntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
    }

    public void execute(
            boolean autoButton,
            boolean feedButton,
            boolean manualIntakeButton,
            boolean resetButton) {

        // ACTIVAR/DESACTIVAR AUTO INTAKE
        if (autoButton && !lastAuto) {
            intake.toggleAutoIntake();
        }

        lastAuto = autoButton;

        // REINICIAR CONTADOR
        if (resetButton && !lastReset) {
            intake.resetCount();
        }

        lastReset = resetButton;

        // ACTUALIZAR MOTORES, SENSORES Y GATE
        intake.update(
                autoButton,
                manualIntakeButton,
                feedButton,
                resetButton
        );
    }

    public void stop() {
        intake.stop();
    }
}

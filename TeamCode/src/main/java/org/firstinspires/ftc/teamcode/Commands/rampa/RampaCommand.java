
package org.firstinspires.ftc.teamcode.Commands.rampa;

import org.firstinspires.ftc.teamcode.Subsystems.RampaSubsystem;

public class RampaCommand {

    private final RampaSubsystem rampa;

    public RampaCommand(RampaSubsystem rampa) {
        this.rampa = rampa;
    }

    public void subir() {
        rampa.subir();
    }

    public void bajar() {
        rampa.bajar();
    }

    public void execute() {
        rampa.update();
    }

    public boolean terminado() {
        return rampa.isReady();
    }

    public boolean fallo() {
        return rampa.hasFault();
    }

    public void stop() {
        rampa.stop();
    }
}

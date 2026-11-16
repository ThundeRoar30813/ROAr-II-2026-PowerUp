
package org.firstinspires.ftc.teamcode.Commands.rampa;

import org.firstinspires.ftc.teamcode.Subsystems.RampaSubsystem;

public class RampaCommand {

    private final RampaSubsystem rampa;

    // ==========================================
    // CONSTRUCTOR
    // ==========================================

    public RampaCommand(RampaSubsystem rampa) {
        this.rampa = rampa;
    }

    // ==========================================
    // SUBIR
    // ==========================================

    public void subir() {
        rampa.subir();
    }

    // ==========================================
    // BAJAR
    // ==========================================

    public void bajar() {
        rampa.bajar();
    }

    // ==========================================
    // UPDATE
    // ==========================================

    public void execute() {
        rampa.update();
    }

    // ==========================================
    // ESTADO
    // ==========================================

    public boolean isReady() {
        return rampa.isReady();
    }

    public boolean hasFault() {
        return rampa.hasFault();
    }

    public String getStatus() {
        return rampa.getStatus();
    }

    // ==========================================
    // RESET
    // ==========================================

    public void clearFault() {
        rampa.clearFault();
    }

    // ==========================================
    // STOP
    // ==========================================

    public void stop() {
        rampa.stop();
    }
}

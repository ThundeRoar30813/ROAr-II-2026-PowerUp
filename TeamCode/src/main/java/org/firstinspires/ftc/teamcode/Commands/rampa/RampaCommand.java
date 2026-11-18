package org.firstinspires.ftc.teamcode.Commands.rampa;

import org.firstinspires.ftc.teamcode.Subsystems.RampaSubsystem;

public class RampaCommand {

    private final RampaSubsystem rampa;

    private boolean lastBtnA = false;
    private boolean lastBtnB = false;
    private boolean lastBtnY = false;

    public RampaCommand(RampaSubsystem rampa) {
        this.rampa = rampa;
    }

    public void subir() { rampa.irArriba(); }
    public void bajar() { rampa.irAbajo(); }
    public void execute() { rampa.update(); }
    public void stop() { rampa.stop(); }

    public void executeTeleOp(
            boolean btnArriba,
            boolean btnBajar,
            boolean btnZero,
            boolean manualBajar,
            boolean manualSubir) {

        if (btnArriba && !lastBtnA) rampa.irArriba();
        lastBtnA = btnArriba;

        if (btnBajar && !lastBtnB) rampa.irAbajo();
        lastBtnB = btnBajar;

        if (btnZero && !lastBtnY) rampa.zeroHere();
        lastBtnY = btnZero;

        if (manualBajar) {
            rampa.setManualPower(0.35);
        } else if (manualSubir) {
            rampa.setManualPower(-0.35);
        }

        rampa.update();
    }
}
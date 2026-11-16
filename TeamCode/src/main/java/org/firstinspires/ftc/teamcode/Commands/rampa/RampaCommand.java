package org.firstinspires.ftc.teamcode.Commands.rampa;

import org.firstinspires.ftc.teamcode.Subsystems.RampaSubsystem;

public class RampaCommand {

    private final RampaSubsystem rampa;

    public enum EstadoRampa {
        ARRIBA,
        OBJETIVO,
        STOP
    }

    private EstadoRampa estadoActual = EstadoRampa.STOP;

    public RampaCommand(RampaSubsystem rampa) {
        this.rampa = rampa;
    }

    public void subir() {
        estadoActual = EstadoRampa.ARRIBA;
        rampa.setTargetTicks(RampaSubsystem.POSICION_ARRIBA);
    }

    public void bajar() {
        estadoActual = EstadoRampa.OBJETIVO;
        rampa.setTargetTicks(RampaSubsystem.POSICION_OBJETIVO);
    }

    public void stop() {
        estadoActual = EstadoRampa.STOP;
        rampa.stop();
    }

    /** Se ejecuta continuamente en el loop del OpMode o Autónomo */
    public void execute() {
        if (rampa.hasFault()) {
            rampa.stop();
            return;
        }

        if (estadoActual == EstadoRampa.STOP) {
            rampa.stop();
            return;
        }

        // Ejecuta tu lógica de control proporcional probada
        rampa.aplicarControl();
    }
}
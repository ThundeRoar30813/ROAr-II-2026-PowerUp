
package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

import java.util.concurrent.TimeUnit;

public class TurretAimControllerV4 {

    private final TurretSubsystemV4 turret;
    private final TurretVisionV5 vision;

    // Verificar signo con shooter apagado.
    public static double TX_TO_TURRET_SIGN = -1.0;

    public static double TX_DEADBAND = 1.0;
    public static double MAX_CORRECTION_PER_FRAME = 2.0;
    public static double MAX_TX = 20.0;
    public static double MAX_RATE_DEG_S = 120.0;

    private boolean enabled;
    private int lastTag = -1;

    private double filteredTx = Double.NaN;
    private double previousTx = Double.NaN;
    private double correction;

    private long lastNs;
    private String status = "OFF";

    public TurretAimControllerV4(
            TurretSubsystemV4 turret,
            TurretVisionV5 vision) {

        this.turret = turret;
        this.vision = vision;
    }

    public void enable() {

        if (turret.isFault()) {
            status = "FAULT";
            return;
        }

        turret.enableHold();
        enabled = true;

        lastNs = System.nanoTime();

        filteredTx = Double.NaN;
        previousTx = Double.NaN;

        status = "WAITING_FOR_SELECTED_TAG";
    }

    public void disable() {

        enabled = false;
        turret.disable();

        status = "OFF";
    }

    public void update() {

        if (!enabled) return;

        if (turret.isFault()) {
            disable();
            status = "FAULT";
            return;
        }

        long now = System.nanoTime();

        double dt = Range.clip(
                (now - lastNs) * 1e-9,
                0.005,
                0.10
        );

        lastNs = now;

        // Cambio automatico de AprilTag.
        if (lastTag != vision.getSelectedTag()) {

            lastTag = vision.getSelectedTag();

            filteredTx = Double.NaN;
            previousTx = Double.NaN;

            status = "TAG_CHANGED_WAITING";
        }

        // Si pierde el tag, mantener posicion.
        if (!vision.hasTarget()) {

            filteredTx = Double.NaN;
            previousTx = Double.NaN;

            status = "NO_TAG_HOLDING";

            turret.update();
            return;
        }

        double raw = vision.getTx();

        // Rechazar cambios bruscos de medicion.
        if (Double.isFinite(previousTx) &&
                Math.abs(raw - previousTx) > 8.0) {

            previousTx = raw;
            status = "VISION_SPIKE_REJECTED";

            turret.update();
            return;
        }

        previousTx = raw;

        // Filtro exponencial.
        filteredTx = Double.isFinite(filteredTx)
                ? 0.65 * filteredTx + 0.35 * raw
                : raw;

        if (Math.abs(filteredTx) <= TX_DEADBAND) {

            status = "VISION_ALIGNED";

            turret.update();
            return;
        }

        if (Math.abs(turret.getError()) > 5.0){
            status = "WAITING:_FOR_MOTOR";

            turret.update();
            return;

        }

        double requested = Range.clip(
                TX_TO_TURRET_SIGN * filteredTx * 0.20,
                -MAX_CORRECTION_PER_FRAME,
                MAX_CORRECTION_PER_FRAME
        );

        if (Math.abs(filteredTx) < 3.0 &&
        Math.abs(turret.getVelocity()) > 25.0) {
            status = "SETTLING";
            turret.update();

            return;

        }

        correction = Range.clip(
                requested,
                -MAX_RATE_DEG_S * dt,
                MAX_RATE_DEG_S * dt
        );

        double newTarget =
                turret.getTarget() + correction;

//        if (newTarget < TurretSubsystemV4.MIN_ANGLE_DEG ||
//                newTarget > TurretSubsystemV4.MAX_ANGLE_DEG) {
//
//            status = "CABLE_LIMIT_HOLD";
//
//        } else {
//
//            turret.setTarget(newTarget);
//            status = "VISION_TRACKING";
//        }

        turret.update();
    }

    public boolean isEnabled() {
        return enabled;
    }

    public String getStatus() {
        return status;
    }

    public double getFilteredTx() {
        return filteredTx;
    }

    public double getCorrection() {
        return correction;
    }
}

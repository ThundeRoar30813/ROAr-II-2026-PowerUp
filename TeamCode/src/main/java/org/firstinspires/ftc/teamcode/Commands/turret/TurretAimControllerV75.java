
package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

public class TurretAimControllerV75 {

    public enum Mode {
        VISION_ONLY,
        PEDRO_ONLY,
        COMBINED
    }

    private final DriveSubsystem drive;
    private final TurretSubsystemV4 turret;
    private final TurretVisionV5 vision;

    // Signos confirmados
    public static double HEADING_SIGN = 1.0;
    public static double TX_SIGN = -1.0;

    // Correccion angular por fotograma
    public static double VISION_GAIN_FAR = 0.85;
    public static double VISION_GAIN_NEAR = 0.08;

    public static double MAX_CORRECTION_PER_FRAME = 8.0;

    // Zona central sin histeresis
    public static double TX_DEADBAND = 0.80;
    public static double TX_NEAR_DEG = 4.0;

    // Filtro aplicado solo a nuevas mediciones
    public static double TX_ALPHA_FAR = 0.75;
    public static double TX_ALPHA_NEAR = 0.30;

    // Rechazo de mediciones
    public static double MAX_TX = 27.0;
    public static double MAX_AGE_MS = 150.0;
    public static double LOST_HOLD_S = 0.20;

    // Perfil de movimiento
    // Usa tus ultimos valores estables si difieren
    public static double MAX_SPEED = 395.0;
    public static double MAX_ACCEL = 2600.0;
    public static double MAX_MOTOR_LAG = 16.0;

    // Pedro
    public static double RATE_FILTER_S = 0.04;
    public static double MAX_HEADING_STEP = 45.0;

    // Feedforward
    public static double FF_FILTER_S = 0.04;

    // Seguridad: tiempo maximo entre frames
    // para aceptar una nueva correccion
    public static double MAX_FRAME_GAP_S = 0.35;

    private Mode mode = Mode.COMBINED;
    private boolean enabled = false;

    private double baseTarget;
    private double smoothTarget;
    private double smoothVelocity;

    private double filteredTx = Double.NaN;
    private double visionRate = 0.0;
    private double chassisRate = 0.0;
    private double filteredFF = 0.0;

    private double lastHeading;

    private long lastNs;
    private long lastValidVisionNs;
    private long lastProcessedFrame = -1;
    private long lastFrameNs = 0;

    private String status = "DISABLED";

    public TurretAimControllerV75(
            DriveSubsystem drive,
            TurretSubsystemV4 turret,
            TurretVisionV5 vision) {

        this.drive = drive;
        this.turret = turret;
        this.vision = vision;
    }

    public void setMode(Mode next) {
        if (next == null || next == mode) return;

        mode = next;

        if (enabled) reset();
    }

    public Mode getMode() {
        return mode;
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

    public double getVisionRate() {
        return visionRate;
    }

    public double getChassisRate() {
        return chassisRate;
    }

    public double getBaseTarget() {
        return baseTarget;
    }

    public double getSmoothTarget() {
        return smoothTarget;
    }

    public double getSmoothVelocity() {
        return smoothVelocity;
    }

    public double getFilteredFF() {
        return filteredFF;
    }

    public void enable() {

        if (turret.isFault()) {
            status = "FAULT";
            return;
        }

        turret.enableHold();

        if (!turret.isEnabled()) {
            status = "NOT_ENABLED";
            return;
        }

        enabled = true;
        reset();
    }

    private void reset() {

        baseTarget = turret.getAngle();
        smoothTarget = baseTarget;
        smoothVelocity = 0.0;

        filteredTx = Double.NaN;
        visionRate = 0.0;
        chassisRate = 0.0;
        filteredFF = 0.0;

        lastHeading = drive.getHeadingDegrees();

        lastNs = System.nanoTime();
        lastValidVisionNs = 0;
        lastFrameNs = 0;

        // Evita reutilizar un frame anterior
        lastProcessedFrame = vision.getFrameSequence();

        turret.setVelocityFeedforward(0.0);
        turret.setTarget(smoothTarget);

        status = "READY";
    }

    public void disable() {

        enabled = false;
        smoothVelocity = 0.0;
        visionRate = 0.0;
        filteredFF = 0.0;

        turret.setVelocityFeedforward(0.0);
        turret.disable();

        status = "DISABLED";
    }

    private double angleDelta(
            double current,
            double previous) {

        double d = current - previous;

        while (d > 180.0) d -= 360.0;
        while (d < -180.0) d += 360.0;

        return d;
    }

    private double getVisionGain(double absError) {

        if (absError <= 2.0) {
            return VISION_GAIN_NEAR;
        }

        if (absError >= 6.0) {
            return VISION_GAIN_FAR;
        }

        double t = (absError - 2.0) / 4.0;

        return VISION_GAIN_NEAR
                + t * (VISION_GAIN_FAR - VISION_GAIN_NEAR);
    }

    public void update() {

        if (!enabled) return;

        if (turret.isFault()) {
            disable();
            status = "FAULT";
            return;
        }

        long now = System.nanoTime();

        double rawDt = (now - lastNs) * 1e-9;
        lastNs = now;

        boolean validDt =
                Double.isFinite(rawDt)
                        && rawDt > 0.001
                        && rawDt <= 0.15;

        double dt = validDt ? rawDt : 0.02;

        // =====================================
        // 1. PEDRO
        // =====================================

        double heading = drive.getHeadingDegrees();

        if (!Double.isFinite(heading)) {
            filteredFF = 0.0;
            turret.setVelocityFeedforward(0.0);
            turret.update();
            status = "INVALID_HEADING";
            return;
        }

        double headingStep =
                angleDelta(heading, lastHeading);

        lastHeading = heading;

        boolean usePedro =
                mode == Mode.PEDRO_ONLY
                        || mode == Mode.COMBINED;

        boolean validHeadingStep =
                validDt
                        && Math.abs(headingStep)
                        <= MAX_HEADING_STEP;

        double measuredChassisRate = 0.0;

        if (usePedro && validHeadingStep) {

            double correction =
                    HEADING_SIGN * headingStep;

            baseTarget += correction;

            measuredChassisRate = Range.clip(
                    correction / dt,
                    -400.0,
                    400.0
            );
        }

        double rateAlpha =
                1.0 - Math.exp(
                        -dt / Math.max(0.005, RATE_FILTER_S)
                );

        chassisRate += rateAlpha
                * (measuredChassisRate - chassisRate);

        if (!usePedro) {
            chassisRate = 0.0;
        }

        // =====================================
        // 2. VISION POR FOTOGRAMA
        // =====================================

        boolean useVision =
                mode == Mode.VISION_ONLY
                        || mode == Mode.COMBINED;

        visionRate = 0.0;

        if (useVision) {

            long frameSequence =
                    vision.getFrameSequence();

            boolean freshFrame =
                    vision.isNewFrame()
                            && frameSequence != lastProcessedFrame;

            if (freshFrame) {

                lastProcessedFrame = frameSequence;

                double frameDt =
                        lastFrameNs == 0
                                ? 0.033
                                : (now - lastFrameNs) * 1e-9;

                lastFrameNs = now;

                double tx = vision.getTx();
                double age = vision.getAgeMs();

                boolean validVision =
                        vision.hasTarget()
                                && Double.isFinite(tx)
                                && Double.isFinite(age)
                                && age >= 0.0
                                && age <= MAX_AGE_MS
                                && Math.abs(tx) <= MAX_TX;

                if (validVision) {

                    lastValidVisionNs = now;

                    double alpha =
                            Math.abs(tx) > TX_NEAR_DEG
                                    ? TX_ALPHA_FAR
                                    : TX_ALPHA_NEAR;

                    if (!Double.isFinite(filteredTx)) {
                        filteredTx = tx;
                    } else {
                        filteredTx += alpha
                                * (tx - filteredTx);
                    }

                    double absTx =
                            Math.abs(filteredTx);

                    double error = 0.0;

                    if (absTx > TX_DEADBAND) {

                        error =
                                Math.signum(filteredTx)
                                        * (absTx - TX_DEADBAND);
                    }

                    // =================================
                    // 3. CORRECCION ANGULAR
                    // =================================

                    double gain =
                            getVisionGain(Math.abs(error));

                    double correction =
                            TX_SIGN * gain * error;

                    correction = Range.clip(
                            correction,
                            -MAX_CORRECTION_PER_FRAME,
                            MAX_CORRECTION_PER_FRAME
                    );

                    // =================================
                    // 4. CONTROL DE MOTOR LAG
                    // =================================

                    double motorLag =
                            baseTarget - turret.getAngle();

                    if (Math.abs(motorLag)
                            > MAX_MOTOR_LAG
                            && Math.signum(motorLag)
                            == Math.signum(correction)) {

                        correction = 0.0;
                        status = "VISION_LAG_LIMIT";

                    } else if (Math.abs(error) < 0.001) {

                        status = "CENTERED";

                    } else {

                        status = "VISION_CORRECTING";
                    }

                    // =================================
                    // 5. APLICAR SOLO UNA VEZ
                    // =================================

                    if (frameDt > 0.0
                            && frameDt <= MAX_FRAME_GAP_S) {

                        baseTarget += correction;

                        // Telemetria en grados/segundo
                        visionRate = correction / frameDt;

                    } else {

                        status = "FRAME_GAP";
                    }

                } else {

                    filteredTx = Double.NaN;
                    status = "NO_VALID_TAG";
                }
            }

            if (lastValidVisionNs == 0) {

                if (!freshFrame) {
                    status = "WAITING_FOR_FRAME";
                }

            } else {

                double lostSeconds =
                        (now - lastValidVisionNs) * 1e-9;

                if (lostSeconds > LOST_HOLD_S) {

                    filteredTx = Double.NaN;
                    status = "NO_TAG";
                }
            }

        } else {

            filteredTx = Double.NaN;
            status = "PEDRO_ONLY";
        }

        // =====================================
        // 6. LIMITES MECANICOS
        // =====================================

//        double min =
//                TurretSubsystemV4.MIN_ANGLE_DEG;
//
//        double max =
//                TurretSubsystemV4.MAX_ANGLE_DEG;

//        baseTarget = Range.clip(
//                baseTarget,
//                min,
//                max
//        );

        // =====================================
        // 7. PERFIL DE MOVIMIENTO
        // =====================================

        double remaining =
                baseTarget - smoothTarget;

        double maxSpeed =
                Math.max(1.0, MAX_SPEED);

        double accel =
                Math.max(1.0, MAX_ACCEL);

        double brakingSpeed =
                Math.sqrt(
                        2.0 * accel * Math.abs(remaining)
                );

        double requestedVelocity =
                Math.signum(remaining)
                        * Math.min(maxSpeed, brakingSpeed);

        double maxVelocityChange =
                accel * dt;

        smoothVelocity += Range.clip(
                requestedVelocity - smoothVelocity,
                -maxVelocityChange,
                maxVelocityChange
        );

        double movement = smoothVelocity * dt;

        if (Math.abs(movement) >= Math.abs(remaining)
                && Math.signum(movement)
                == Math.signum(remaining)) {

            smoothTarget = baseTarget;
            smoothVelocity = 0.0;

        } else {

            smoothTarget += movement;
        }

//        smoothTarget = Range.clip(
//                smoothTarget,
//                min,
//                max
//        );
//
//        if ((smoothTarget >= max
//                && smoothVelocity > 0.0)
//                || (smoothTarget <= min
//                && smoothVelocity < 0.0)) {
//
//            smoothVelocity = 0.0;
//        }

        // =====================================
        // 8. FEEDFORWARD
        // =====================================

        double ffAlpha =
                1.0 - Math.exp(
                        -dt / Math.max(0.005, FF_FILTER_S)
                );

        filteredFF += ffAlpha
                * (smoothVelocity - filteredFF);
//
//        if ((smoothTarget >= max
//                && filteredFF > 0.0)
//                || (smoothTarget <= min
//                && filteredFF < 0.0)) {
//
//            filteredFF = 0.0;
//        }

        // =====================================
        // 9. ENVIAR AL SUBSYSTEM
        // =====================================

        turret.setVelocityFeedforward(filteredFF);
        turret.setTarget(smoothTarget);
        turret.update();
    }
}


package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

public class TurretAimControllerV77 {

    public enum Mode {
        VISION_ONLY,
        PEDRO_ONLY,
        COMBINED
    }

    private final DriveSubsystem drive;
    private final TurretSubsystemV4 turret;
    private final TurretVisionV5 vision;

    public static double HEADING_SIGN = 1.0;
    public static double TX_SIGN = -1.0;

    public static double VISION_GAIN_FAR = 0.85;
    public static double VISION_GAIN_NEAR = 0.07;

    public static double MAX_CORRECTION_PER_FRAME = 8.0;

    public static double TX_DEADBAND = 1.0;
    public static double TX_NEAR_DEG = 4.0;

    public static double TX_ALPHA_FAR = 0.85;
    public static double TX_ALPHA_NEAR = 0.35;

    public static double MAX_TX = 27.0;
    public static double MAX_AGE_MS = 150.0;
    public static double LOST_HOLD_S = 0.20;

    public static double MAX_SPEED = 450.0;
    public static double MAX_ACCEL = 3200.0;
    public static double MAX_MOTOR_LAG = 18.0;

    public static double RATE_FILTER_S = 0.04;
    public static double MAX_HEADING_STEP = 45.0;
    public static double FF_FILTER_S = 0.04;

    public static double MAX_FRAME_GAP_S = 0.30;

    // Compensacion por movimiento X/Y.
    // Ajustar usando las coordenadas reales
    // del campo y los AprilTags.
    public static double POSITION_GAIN = 0.35;
    public static double MAX_POSITION_STEP = 2.0;
    public static double MIN_TARGET_DISTANCE = 8.0;

    public static double TAG34_X = 51.0;
    public static double TAG34_Y = 57.0;

    public static double TAG37_X = 62.0;
    public static double TAG37_Y = 57.0;

    private Mode mode = Mode.COMBINED;
    private boolean enabled = false;

    private double baseTarget;
    private double smoothTarget;
    private double smoothVelocity;

    private double filteredTx = Double.NaN;
    private double visionRate = 0;
    private double chassisRate = 0;
    private double filteredFF = 0;

    private double lastHeading;
    private double lastBearing = Double.NaN;

    private int lastTag = -1;

    private long lastNs;
    private long lastValidVisionNs;
    private long lastProcessedFrame = -1;
    private long lastFrameNs;

    private String status = "DISABLED";

    public TurretAimControllerV77(
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

        if (enabled) {
            reset();
        }
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

        // CORRECCION V77:
        // Inicializar con el angulo real,
        // no con una variable sin declarar.
        smoothTarget = turret.getAngle();

        smoothVelocity = 0;

        filteredTx = Double.NaN;
        visionRate = 0;
        chassisRate = 0;
        filteredFF = 0;

        lastHeading = drive.getHeadingDegrees();

        lastTag = vision.getSelectedTag();
        lastBearing = currentBearing(lastTag);

        lastNs = System.nanoTime();
        lastValidVisionNs = 0;
        lastFrameNs = 0;

        lastProcessedFrame =
                vision.getFrameSequence();

        turret.setVelocityFeedforward(0);
        turret.setTarget(smoothTarget);

        status = "READY";
    }

    public void disable() {

        enabled = false;
        smoothVelocity = 0;
        visionRate = 0;
        filteredFF = 0;

        turret.setVelocityFeedforward(0);
        turret.disable();

        status = "DISABLED";
    }

    private double angleDelta(
            double current,
            double previous) {

        double delta = current - previous;

        while (delta > 180) {
            delta -= 360;
        }

        while (delta < -180) {
            delta += 360;
        }

        return delta;
    }

    private double[] targetFor(int id) {

        if (id == 34) {
            return new double[]{
                    TAG34_X,
                    TAG34_Y
            };
        }

        if (id == 37) {
            return new double[]{
                    TAG37_X,
                    TAG37_Y
            };
        }

        return null;
    }

    private double currentBearing(int id) {

        double[] target = targetFor(id);

        if (target == null) {
            return Double.NaN;
        }

        double dx = target[0] - drive.getX();
        double dy = target[1] - drive.getY();

        if (!Double.isFinite(dx)
                || !Double.isFinite(dy)
                || Math.hypot(dx, dy)
                < MIN_TARGET_DISTANCE) {

            return Double.NaN;
        }

        return Math.toDegrees(
                Math.atan2(dy, dx)
        );
    }

    private double visionGain(double error) {

        if (error <= 2) {
            return VISION_GAIN_NEAR;
        }

        if (error >= 6) {
            return VISION_GAIN_FAR;
        }

        double t = (error - 2) / 4;

        return VISION_GAIN_NEAR
                + t * (
                VISION_GAIN_FAR
                        - VISION_GAIN_NEAR
        );
    }

    // ==========================================
    // SELECCION DE ANGULO EQUIVALENTE
    // ==========================================

    private double choosePhysicalTarget(
            double virtualTarget) {

        double min = TurretSubsystemV4.MIN_ANGLE_DEG;
        double max = TurretSubsystemV4.MAX_ANGLE_DEG;

        double actual = turret.getAngle();

        double bestTarget = Double.NaN;
        double bestDistance = Double.POSITIVE_INFINITY;

        // Buscar representaciones equivalentes:
        // theta, theta +/- 360, etc.
        for (int k = -3; k <= 3; k++) {

            double candidate =
                    virtualTarget + 360.0 * k;

            if (candidate < min || candidate > max) {
                continue;
            }

            double distance =
                    Math.abs(candidate - actual);

            if (distance < bestDistance) {

                bestDistance = distance;
                bestTarget = candidate;
            }
        }

        if (Double.isFinite(bestTarget)) {
            return bestTarget;
        }

        // No existe equivalente dentro del
        // recorrido mecanico.
        // Elegir el limite con menor error
        // angular respecto al objetivo.
        double bestMiss = Double.POSITIVE_INFINITY;
        double bestBound = actual;

        for (int k = -3; k <= 3; k++) {

            double candidate =
                    virtualTarget + 360.0 * k;

            double bound = Range.clip(
                    candidate,
                    min,
                    max
            );

            double miss =
                    Math.abs(candidate - bound);

            if (miss < bestMiss) {

                bestMiss = miss;
                bestBound = bound;
            }
        }

        status = "TARGET_OUTSIDE_ARC";

        return bestBound;
    }

    // ==========================================
    // UPDATE
    // ==========================================

    public void update() {

        if (!enabled) return;

        if (turret.isFault()) {

            disable();
            status = "FAULT";
            return;
        }

        long now = System.nanoTime();

        double rawDt =
                (now - lastNs) * 1e-9;

        lastNs = now;

        boolean validDt =
                Double.isFinite(rawDt)
                        && rawDt > 0.001
                        && rawDt <= 0.15;

        double dt = validDt ? rawDt : 0.02;

        // ======================================
        // 1. PEDRO HEADING
        // ======================================

        double heading = drive.getHeadingDegrees();

        if (!Double.isFinite(heading)) {

            turret.setVelocityFeedforward(0);
            turret.update();

            status = "INVALID_HEADING";
            return;
        }

        double headingStep =
                angleDelta(
                        heading,
                        lastHeading
                );

        lastHeading = heading;

        boolean usePedro =
                mode == Mode.PEDRO_ONLY
                        || mode == Mode.COMBINED;

        boolean validHeadingStep =
                validDt
                        && Math.abs(headingStep)
                        <= MAX_HEADING_STEP;

        double measuredChassisRate = 0;

        if (usePedro && validHeadingStep) {

            double correction =
                    HEADING_SIGN * headingStep;

            baseTarget += correction;

            measuredChassisRate = Range.clip(
                    correction / dt,
                    -400,
                    400
            );
        }

        double rateAlpha =
                1 - Math.exp(
                        -dt / Math.max(
                                0.005,
                                RATE_FILTER_S
                        )
                );

        chassisRate += rateAlpha
                * (
                measuredChassisRate
                        - chassisRate
        );

        if (!usePedro) {
            chassisRate = 0;
        }

        // ======================================
        // 2. PEDRO POSITION
        // ======================================

        int tag = vision.getSelectedTag();

        if (tag != lastTag) {

            lastTag = tag;
            lastBearing = currentBearing(tag);

            filteredTx = Double.NaN;

            lastProcessedFrame =
                    vision.getFrameSequence();

            lastFrameNs = 0;

        } else {

            double bearing =
                    currentBearing(tag);

            if (usePedro
                    && validDt
                    && Double.isFinite(bearing)
                    && Double.isFinite(lastBearing)) {

                double step =
                        angleDelta(
                                bearing,
                                lastBearing
                        );

                if (Math.abs(step)
                        <= MAX_HEADING_STEP) {

                    baseTarget += Range.clip(
                            -step * POSITION_GAIN,
                            -MAX_POSITION_STEP,
                            MAX_POSITION_STEP
                    );
                }
            }

            lastBearing = bearing;
        }

        // ======================================
        // 3. LIMELIGHT
        // ======================================

        boolean useVision =
                mode == Mode.VISION_ONLY
                        || mode == Mode.COMBINED;

        visionRate = 0;

        if (useVision) {

            long frame =
                    vision.getFrameSequence();

            boolean freshFrame =
                    vision.isNewFrame()
                            && frame != lastProcessedFrame;

            if (freshFrame) {

                lastProcessedFrame = frame;

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
                                && age >= 0
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

                    double error = 0;

                    if (absTx > TX_DEADBAND) {

                        error =
                                Math.signum(filteredTx)
                                        * (
                                        absTx - TX_DEADBAND
                                );
                    }

                    double gain =
                            visionGain(
                                    Math.abs(error)
                            );

                    double correction =
                            TX_SIGN * gain * error;

                    correction = Range.clip(
                            correction,
                            -MAX_CORRECTION_PER_FRAME,
                            MAX_CORRECTION_PER_FRAME
                    );

                    double motorLag =
                            baseTarget - turret.getAngle();

                    if (Math.abs(motorLag)
                            > MAX_MOTOR_LAG
                            && Math.signum(motorLag)
                            == Math.signum(correction)) {

                        correction = 0;
                        status = "VISION_LAG_LIMIT";

                    } else if (Math.abs(error)
                            < 0.001) {

                        status = "CENTERED";

                    } else {

                        status = "VISION_CORRECTING";
                    }

                    if (frameDt > 0
                            && frameDt <= MAX_FRAME_GAP_S) {

                        baseTarget += correction;

                        visionRate =
                                correction / frameDt;

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
                        (now - lastValidVisionNs)
                                * 1e-9;

                if (lostSeconds > LOST_HOLD_S) {

                    filteredTx = Double.NaN;
                    status = "NO_TAG";
                }
            }

        } else {

            filteredTx = Double.NaN;
            status = "PEDRO_ONLY";
        }

        // ======================================
        // 4. ANGULO FISICO SEGURO
        // ======================================

        double min = TurretSubsystemV4.MIN_ANGLE_DEG;
        double max = TurretSubsystemV4.MAX_ANGLE_DEG;

        double physicalTarget =
                choosePhysicalTarget(baseTarget);

        // Mantener el angulo virtual acotado
        // numericamente, sin perder direccion.
        while (baseTarget - physicalTarget > 540) {
            baseTarget -= 360;
        }

        while (baseTarget - physicalTarget < -540) {
            baseTarget += 360;
        }

        // ======================================
        // 5. PERFIL DE MOVIMIENTO
        // ======================================

        double remaining =
                physicalTarget - smoothTarget;

        double maxSpeed =
                Math.max(1, MAX_SPEED);

        double accel =
                Math.max(1, MAX_ACCEL);

        double brakingSpeed =
                Math.sqrt(
                        2 * accel
                                * Math.abs(remaining)
                );

        double requestedVelocity =
                Math.signum(remaining)
                        * Math.min(
                        maxSpeed,
                        brakingSpeed
                );

        double maxChange = accel * dt;

        smoothVelocity += Range.clip(
                requestedVelocity - smoothVelocity,
                -maxChange,
                maxChange
        );

        double movement =
                smoothVelocity * dt;

        if (Math.abs(movement)
                >= Math.abs(remaining)
                && Math.signum(movement)
                == Math.signum(remaining)) {

            smoothTarget = physicalTarget;
            smoothVelocity = 0;

        } else {

            smoothTarget += movement;
        }

        smoothTarget = Range.clip(
                smoothTarget,
                min,
                max
        );

        if ((smoothTarget >= max
                && smoothVelocity > 0)
                || (smoothTarget <= min
                && smoothVelocity < 0)) {

            smoothVelocity = 0;
        }

        // ======================================
        // 6. FEEDFORWARD
        // ======================================

        double ffAlpha =
                1 - Math.exp(
                        -dt / Math.max(
                                0.005,
                                FF_FILTER_S
                        )
                );

        filteredFF += ffAlpha
                * (
                smoothVelocity
                        - filteredFF
        );

        if ((smoothTarget >= max
                && filteredFF > 0)
                || (smoothTarget <= min
                && filteredFF < 0)) {

            filteredFF = 0;
        }

        // ======================================
        // 7. MOTOR
        // ======================================

        turret.setVelocityFeedforward(filteredFF);
        turret.setTarget(smoothTarget);
        turret.update();
    }
}

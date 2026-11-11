
package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

public class TurretAimControllerV76 {

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

    public static double VISION_GAIN_FAR = 0.50;
    public static double VISION_GAIN_NEAR = 0.05;
    public static double MAX_CORRECTION_PER_FRAME = 3.0;

    public static double TX_DEADBAND = 1.00;
    public static double TX_NEAR_DEG = 4.0;
    public static double TX_ALPHA_FAR = 0.95;
    public static double TX_ALPHA_NEAR = 0.45;

    public static double MAX_TX = 27.0;
    public static double MAX_AGE_MS = 150.0;
    public static double LOST_HOLD_S = 0.20;

    public static double MAX_SPEED = 480.0;
    public static double MAX_ACCEL = 3400.0;
    public static double MAX_MOTOR_LAG = 18.0;

    public static double RATE_FILTER_S = 0.04;
    public static double MAX_HEADING_STEP = 45.0;
    public static double FF_FILTER_S = 0.05;
    public static double MAX_FRAME_GAP_S = 0.30;

    // Compensacion de strafe
    public static double POSITION_GAIN = 1.00;
    public static double MAX_POSITION_STEP = 4.0;
    public static double MIN_TARGET_DISTANCE = 8.0;

    // Coordenadas Pedro Pathing
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
    private double visionRate = 0.0;
    private double chassisRate = 0.0;
    private double filteredFF = 0.0;

    private double lastHeading;
    private double lastBearing = Double.NaN;
    private int lastTag = -1;

    private long lastNs;
    private long lastValidVisionNs;
    private long lastProcessedFrame = -1;
    private long lastFrameNs = 0;

    private String status = "DISABLED";

    public TurretAimControllerV76(
            DriveSubsystem drive,
            TurretSubsystemV4 turret,
            TurretVisionV5 vision) {
        this.drive = drive;
        this.turret = turret;
        this.vision = vision;
    }

    private double[] targetFor(int id) {
        if (id == 34)
            return new double[]{TAG34_X, TAG34_Y};
        if (id == 37)
            return new double[]{TAG37_X, TAG37_Y};
        return null;
    }

    private double currentBearing(int id) {
        double[] t = targetFor(id);
        if (t == null) return Double.NaN;

        double dx = t[0] - drive.getX();
        double dy = t[1] - drive.getY();

        if (!Double.isFinite(dx)
                || !Double.isFinite(dy)
                || Math.hypot(dx, dy) < MIN_TARGET_DISTANCE) {
            return Double.NaN;
        }

        return Math.toDegrees(Math.atan2(dy, dx));
    }

    private double angleDelta(double current, double previous) {
        double d = current - previous;
        while (d > 180.0) d -= 360.0;
        while (d < -180.0) d += 360.0;
        return d;
    }

    private double getVisionGain(double absError) {
        if (absError <= 2.0) return VISION_GAIN_NEAR;
        if (absError >= 6.0) return VISION_GAIN_FAR;

        double t = (absError - 2.0) / 4.0;
        return VISION_GAIN_NEAR
                + t * (VISION_GAIN_FAR - VISION_GAIN_NEAR);
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
        lastTag = vision.getSelectedTag();
        lastBearing = currentBearing(lastTag);

        lastNs = System.nanoTime();
        lastValidVisionNs = 0;
        lastFrameNs = 0;
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

        boolean validDt = Double.isFinite(rawDt)
                && rawDt > 0.001
                && rawDt <= 0.15;

        double dt = validDt ? rawDt : 0.02;

        // ====================================
        // 1. COMPENSACION DE GIRO CON PEDRO
        // ====================================

        double heading = drive.getHeadingDegrees();

        if (!Double.isFinite(heading)) {
            filteredFF = 0.0;
            turret.setVelocityFeedforward(0.0);
            turret.update();
            status = "INVALID_HEADING";
            return;
        }

        double headingStep = angleDelta(heading, lastHeading);
        lastHeading = heading;

        boolean usePedro = mode == Mode.PEDRO_ONLY
                || mode == Mode.COMBINED;

        double measuredChassisRate = 0.0;

        if (usePedro && validDt
                && Math.abs(headingStep) <= MAX_HEADING_STEP) {

            double correction = HEADING_SIGN * headingStep;
            baseTarget += correction;

            measuredChassisRate = Range.clip(
                    correction / dt, -400.0, 400.0);
        }

        double rateAlpha = 1.0 - Math.exp(
                -dt / Math.max(0.005, RATE_FILTER_S));

        chassisRate += rateAlpha
                * (measuredChassisRate - chassisRate);

        if (!usePedro) chassisRate = 0.0;

        // ====================================
        // 2. COMPENSACION LATERAL CON PEDRO
        // ====================================

        int tag = vision.getSelectedTag();

        if (tag != lastTag) {
            lastTag = tag;
            lastBearing = currentBearing(tag);
            filteredTx = Double.NaN;
            lastProcessedFrame = vision.getFrameSequence();
            lastFrameNs = 0;
        } else {
            double bearing = currentBearing(tag);

            if (usePedro && validDt
                    && Double.isFinite(bearing)
                    && Double.isFinite(lastBearing)) {

                double step = angleDelta(bearing, lastBearing);

                if (Math.abs(step) <= MAX_HEADING_STEP) {
                    baseTarget += Range.clip(
                            -step * POSITION_GAIN,
                            -MAX_POSITION_STEP,
                            MAX_POSITION_STEP);
                }
            }

            lastBearing = bearing;
        }

        // ====================================
        // 3. LIMELIGHT - FOTOGRAMAS NUEVOS
        // ====================================

        boolean useVision = mode == Mode.VISION_ONLY
                || mode == Mode.COMBINED;

        visionRate = 0.0;

        if (useVision) {
            long frameSequence = vision.getFrameSequence();

            boolean freshFrame = vision.isNewFrame()
                    && frameSequence != lastProcessedFrame;

            if (freshFrame) {
                lastProcessedFrame = frameSequence;

                double frameDt = lastFrameNs == 0
                        ? 0.033
                        : (now - lastFrameNs) * 1e-9;

                lastFrameNs = now;

                double tx = vision.getTx();
                double age = vision.getAgeMs();

                boolean validVision = vision.hasTarget()
                        && Double.isFinite(tx)
                        && Double.isFinite(age)
                        && age >= 0.0
                        && age <= MAX_AGE_MS
                        && Math.abs(tx) <= MAX_TX;

                if (validVision) {
                    lastValidVisionNs = now;

                    double alpha = Math.abs(tx) > TX_NEAR_DEG
                            ? TX_ALPHA_FAR
                            : TX_ALPHA_NEAR;

                    if (!Double.isFinite(filteredTx)) {
                        filteredTx = tx;
                    } else {
                        filteredTx += alpha * (tx - filteredTx);
                    }

                    double absTx = Math.abs(filteredTx);
                    double error = 0.0;

                    if (absTx > TX_DEADBAND) {
                        error = Math.signum(filteredTx)
                                * (absTx - TX_DEADBAND);
                    }

                    double gain = getVisionGain(Math.abs(error));
                    double correction = TX_SIGN * gain * error;

                    correction = Range.clip(
                            correction,
                            -MAX_CORRECTION_PER_FRAME,
                            MAX_CORRECTION_PER_FRAME);

                    // Evitar acumular correcciones
                    // cuando el motor ya va atrasado
                    double motorLag = baseTarget - turret.getAngle();

                    if (Math.abs(motorLag) > MAX_MOTOR_LAG
                            && Math.signum(motorLag)
                            == Math.signum(correction)) {

                        correction = 0.0;
                        status = "VISION_LAG_LIMIT";

                    } else if (Math.abs(error) < 0.001) {
                        status = "CENTERED";
                    } else {
                        status = "VISION_CORRECTING";
                    }

                    if (frameDt > 0.0
                            && frameDt <= MAX_FRAME_GAP_S) {

                        baseTarget += correction;
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

        // ====================================
        // 4. LIMITES MECANICOS
        // ====================================

        double min = TurretSubsystemV4.MIN_ANGLE_DEG;
        double max = TurretSubsystemV4.MAX_ANGLE_DEG;

        baseTarget = Range.clip(baseTarget, min, max);

        // ====================================
        // 5. PERFIL DE VELOCIDAD
        // ====================================

        double remaining = baseTarget - smoothTarget;

        double maxSpeed = Math.max(1.0, MAX_SPEED);
        double accel = Math.max(1.0, MAX_ACCEL);

        double brakingSpeed = Math.sqrt(
                2.0 * accel * Math.abs(remaining));

        double requestedVelocity =
                Math.signum(remaining)
                        * Math.min(maxSpeed, brakingSpeed);

        double maxVelocityChange = accel * dt;

        smoothVelocity += Range.clip(
                requestedVelocity - smoothVelocity,
                -maxVelocityChange,
                maxVelocityChange);

        double movement = smoothVelocity * dt;

        if (Math.abs(movement) >= Math.abs(remaining)
                && Math.signum(movement)
                == Math.signum(remaining)) {

            smoothTarget = baseTarget;
            smoothVelocity = 0.0;

        } else {
            smoothTarget += movement;
        }

        smoothTarget = Range.clip(smoothTarget, min, max);

        if ((smoothTarget >= max && smoothVelocity > 0.0)
                || (smoothTarget <= min && smoothVelocity < 0.0)) {
            smoothVelocity = 0.0;
        }

        // ====================================
        // 6. FEEDFORWARD
        // ====================================

        double ffAlpha = 1.0 - Math.exp(
                -dt / Math.max(0.005, FF_FILTER_S));

        filteredFF += ffAlpha
                * (smoothVelocity - filteredFF);

        if ((smoothTarget >= max && filteredFF > 0.0)
                || (smoothTarget <= min && filteredFF < 0.0)) {
            filteredFF = 0.0;
        }

        // ====================================
        // 7. ACTUALIZAR MOTOR
        // ====================================

        turret.setVelocityFeedforward(filteredFF);
        turret.setTarget(smoothTarget);
        turret.update();
    }
}

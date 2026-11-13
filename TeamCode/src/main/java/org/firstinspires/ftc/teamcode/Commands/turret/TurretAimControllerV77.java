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

    // Coordenadas Pedro Pathing en pulgadas (ajusta X/Y exactos midiendo en tu campo real)
    public static double TAG34_X = 51.0;
    public static double TAG34_Y = 57.0;

    public static double TAG35_X = 55.0;
    public static double TAG35_Y = 57.0;

    public static double TAG37_X = 62.0;
    public static double TAG37_Y = 57.0;

    // Signos de dirección
    public static double TURRET_DIRECTION_SIGN = -1.0; // Invierte el sentido relativo respecto a Pedro Pathing
    public static double TX_SIGN = -1.0;               // Invierte el sentido de corrección de la Limelight si empuja al lado contrario

    // Ajustes de Limelight (Offset Suave anti-vibración)
    public static double VISION_OFFSET_SPEED = 1.5;    // Grados por segundo que la cámara puede corregir
    public static double MAX_VISION_OFFSET = 12.0;     // Máximo error (en grados) que puede corregir la cámara
    public static double TX_DEADBAND = 1.5;            // Zona muerta en grados para ignorar micro-vibraciones

    private Mode mode = Mode.COMBINED;
    private boolean enabled = false;
    private String status = "DISABLED";

    private double baseTarget = 0.0;
    private double visionOffset = 0.0;
    private int selectedTag = -1;

    private double lastRobotHeading = Double.NaN;
    private double filteredChassisAngVel = 0.0;

    private long lastNs;

    public TurretAimControllerV77(
            DriveSubsystem drive,
            TurretSubsystemV4 turret,
            TurretVisionV5 vision) {
        this.drive = drive;
        this.turret = turret;
        this.vision = vision;
    }

    private double[] targetFor(int id) {
        if (id == 34) return new double[]{TAG34_X, TAG34_Y};
        if (id == 35) return new double[]{TAG35_X, TAG35_Y};
        if (id == 37) return new double[]{TAG37_X, TAG37_Y};
        return null;
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

    public double getBaseTarget() {
        return baseTarget;
    }

    public double getVisionOffset() {
        return visionOffset;
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
        visionOffset = 0.0;
        selectedTag = vision.getSelectedTag();
        lastRobotHeading = Double.NaN;
        filteredChassisAngVel = 0.0;
        lastNs = System.nanoTime();
        status = "READY";
    }

    public void disable() {
        enabled = false;
        turret.setVelocityFeedforward(0.0);
        turret.disable();
        status = "DISABLED";
    }

    private double normalizeAngleToCurrent(double target, double current) {
        double diff = (target - current) % 360.0;
        if (diff > 180.0) diff -= 360.0;
        if (diff < -180.0) diff += 360.0;

        double candidate = current + diff;

        if (candidate > TurretSubsystemV4.MAX_ANGLE_DEG) {
            double unwrapped = candidate - 360.0;
            if (unwrapped >= TurretSubsystemV4.MIN_ANGLE_DEG) {
                return unwrapped;
            }
        }
        if (candidate < TurretSubsystemV4.MIN_ANGLE_DEG) {
            double unwrapped = candidate + 360.0;
            if (unwrapped <= TurretSubsystemV4.MAX_ANGLE_DEG) {
                return unwrapped;
            }
        }

        return candidate;
    }

    public void update() {
        if (!enabled) return;

        if (turret.isFault()) {
            disable();
            status = "FAULT";
            return;
        }

        long now = System.nanoTime();
        double dt = (now - lastNs) * 1e-9;
        lastNs = now;
        if (!Double.isFinite(dt) || dt <= 0.001 || dt > 0.15) {
            dt = 0.02;
        }

        int currentTag = vision.getSelectedTag();
        if (currentTag != selectedTag) {
            selectedTag = currentTag;
            visionOffset = 0.0;
        }

        double[] targetCoords = targetFor(selectedTag);

        // ====================================
        // 1. CÁLCULO ABSOLUTO (PEDRO PATHING) + FEEDFORWARD
        // ====================================
        double pedroTargetAngle = turret.getAngle();
        double desiredFeedforwardDegS = 0.0;

        if (targetCoords != null && (mode == Mode.COMBINED || mode == Mode.PEDRO_ONLY)) {
            double robotX = drive.getX();
            double robotY = drive.getY();
            double robotHeading = drive.getHeadingDegrees();

            if (Double.isFinite(robotX) && Double.isFinite(robotY) && Double.isFinite(robotHeading)) {
                double absoluteFieldAngle = Math.toDegrees(
                        Math.atan2(targetCoords[1] - robotY, targetCoords[0] - robotX)
                );

                double relativeTarget = TURRET_DIRECTION_SIGN * (absoluteFieldAngle - robotHeading);
                pedroTargetAngle = normalizeAngleToCurrent(relativeTarget, turret.getAngle());

                // Velocidad angular del chasis (grados/seg)
                if (Double.isFinite(lastRobotHeading)) {
                    double deltaHeading = robotHeading - lastRobotHeading;
                    if (deltaHeading > 180.0) deltaHeading -= 360.0;
                    if (deltaHeading < -180.0) deltaHeading += 360.0;

                    double rawChassisVel = deltaHeading / dt;
                    filteredChassisAngVel = (0.7 * filteredChassisAngVel) + (0.3 * rawChassisVel);
                }
                lastRobotHeading = robotHeading;

                // Compensar el giro del chasis en sentido opuesto
                desiredFeedforwardDegS = -TURRET_DIRECTION_SIGN * filteredChassisAngVel;
            }
        } else {
            lastRobotHeading = Double.NaN;
            filteredChassisAngVel = 0.0;
        }

        // ====================================
        // 2. OFFSET SUAVE DE VISIÓN (LIMELIGHT)
        // ====================================
        if (mode == Mode.COMBINED) {
            if (vision.isResultValid() && vision.hasTarget()) {
                double tx = vision.getTx();

                if (Double.isFinite(tx) && Math.abs(tx) > TX_DEADBAND) {
                    double correctionStep = Math.signum(tx) * VISION_OFFSET_SPEED * dt;
                    visionOffset += (TX_SIGN * correctionStep);
                    visionOffset = Range.clip(visionOffset, -MAX_VISION_OFFSET, MAX_VISION_OFFSET);
                    status = "TRACKING_FUSED";
                } else {
                    status = "LOCKED_CENTER";
                }
            } else {
                status = "PEDRO_COASTING";
            }

            baseTarget = pedroTargetAngle + visionOffset;

        } else if (mode == Mode.PEDRO_ONLY) {
            visionOffset = 0.0;
            baseTarget = pedroTargetAngle;
            status = "PEDRO_ONLY";

        } else if (mode == Mode.VISION_ONLY) {
            visionOffset = 0.0;
            if (vision.isResultValid() && vision.hasTarget()) {
                double tx = vision.getTx();
                if (Double.isFinite(tx) && Math.abs(tx) > TX_DEADBAND) {
                    baseTarget = turret.getAngle() + (TX_SIGN * tx * 0.6);
                    status = "VISION_TRACKING";
                } else {
                    baseTarget = turret.getAngle();
                    status = "VISION_CENTERED";
                }
            } else {
                baseTarget = turret.getAngle();
                status = "VISION_NO_TAG";
            }
        }

        // ====================================
        // 3. LÍMITES MECÁNICOS (PROTECCIÓN DE CABLES)
        // ====================================
        if (baseTarget < TurretSubsystemV4.MIN_ANGLE_DEG || baseTarget > TurretSubsystemV4.MAX_ANGLE_DEG) {
            status = "OUT_OF_RANGE_LIMIT";
            desiredFeedforwardDegS = 0.0;
        }

        baseTarget = Range.clip(
                baseTarget,
                TurretSubsystemV4.MIN_ANGLE_DEG,
                TurretSubsystemV4.MAX_ANGLE_DEG
        );

        // ====================================
        // 4. ACTUALIZAR MOTOR
        // ====================================
        turret.setVelocityFeedforward(desiredFeedforwardDegS);
        turret.setTarget(baseTarget);
        turret.update();
    }
}
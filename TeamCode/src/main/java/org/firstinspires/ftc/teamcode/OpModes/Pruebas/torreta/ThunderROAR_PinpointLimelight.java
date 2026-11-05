package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

@TeleOp(name = "ThunderROAR - PINPOINT + LIMELIGHT")
public class ThunderROAR_PinpointLimelight extends OpMode {

    private DriveSubsystem drive;
    private TurretSubsystem turret;
    private MecanumDriveCmd driveCmd;
    private Limelight3A limelight;

    private boolean trackingEnabled = false;
    private boolean visionEnabled = false;

    private boolean previousX = false;
    private boolean previousB = false;

    private double startingHeading;
    private double startingTurretAngle;

    private double previousRawHeading;
    private double accumulatedRobotRotation;

    private double targetAngle;
    private double previousTarget;
    private double targetVelocity;

    private long previousTime;

    // Este signo ya funciono en tu robot.
    private static final double TRACKING_SIGN = -1.0;

    private static final double MAX_TARGET_VELOCITY = 500.0;

    // Correccion visual inicial conservadora.
    private static final double VISION_KP = 0.12;

    // Se calibra segun el sentido real del TX.
    private static final double VISION_SIGN = 1.0;

    private static final double TX_DEADBAND = 0.7;
    private static final double MAX_VISION_RATE = 10.0;
    private static final double MAX_VISION_OFFSET = 15.0;

    // Rechazar resultados demasiado antiguos.
    private static final double MAX_STALENESS_MS = 150.0;

    private double visionOffsetDeg = 0.0;
    private double tx = 0.0;
    private double visionStep = 0.0;

    private boolean tagVisible = false;
    private int detectedTag = -1;

    @Override
    public void init() {

        drive = new DriveSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        driveCmd = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> gamepad1.left_stick_x,
                () -> gamepad1.right_stick_x
        );

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(50);

        targetAngle = turret.getCurrentAngleDeg();
        previousTarget = targetAngle;
        previousTime = System.nanoTime();

        telemetry.addLine("THUNDERROAR READY");
        telemetry.addLine("X = Pinpoint tracking ON/OFF");
        telemetry.addLine("B = Vision correction ON/OFF");
        telemetry.addLine("Tags permitidos: 34 y 36");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        // =====================================
        // 1. CHASIS Y PINPOINT
        // =====================================

        drive.update();
        driveCmd.execute();

        double rawHeading = drive.getHeadingDegrees();

        // =====================================
        // 2. BOTONES
        // =====================================

        boolean currentX = gamepad1.x;
        boolean currentB = gamepad1.b;

        if (currentX && !previousX) {

            trackingEnabled = !trackingEnabled;

            if (trackingEnabled) {

                startingHeading = rawHeading;
                previousRawHeading = rawHeading;
                accumulatedRobotRotation = 0.0;

                startingTurretAngle =
                        turret.getCurrentAngleDeg();

                targetAngle = startingTurretAngle;
                previousTarget = targetAngle;
                targetVelocity = 0.0;

                visionOffsetDeg = 0.0;
                visionStep = 0.0;

                previousTime = System.nanoTime();

            } else {

                visionEnabled = false;
                visionOffsetDeg = 0.0;
                targetVelocity = 0.0;

                turret.stop();
            }
        }

        if (currentB && !previousB && trackingEnabled) {

            visionEnabled = !visionEnabled;

            // Evitar correcciones antiguas.
            visionOffsetDeg = 0.0;
            visionStep = 0.0;
        }

        previousX = currentX;
        previousB = currentB;

        // =====================================
        // 3. LEER APRILTAGS
        // =====================================

        tagVisible = false;
        detectedTag = -1;
        tx = 0.0;
        visionStep = 0.0;

        LLResult result = limelight.getLatestResult();

        if (result != null
                && result.isValid()
                && result.getStaleness() <= MAX_STALENESS_MS
                && result.getFiducialResults() != null) {

            for (LLResultTypes.FiducialResult fiducial
                    : result.getFiducialResults()) {

                int id = fiducial.getFiducialId();

                if (id == 34 || id == 36) {

                    detectedTag = id;
                    tx = fiducial.getTargetXDegrees();
                    tagVisible = true;

                    break;
                }
            }
        }

        // =====================================
        // 4. TRACKING PINPOINT
        // =====================================

        if (trackingEnabled) {

            long now = System.nanoTime();

            double dt =
                    (now - previousTime) / 1_000_000_000.0;

            previousTime = now;

            if (dt <= 0.0 || dt > 0.1) {
                dt = 0.02;
            }

            // Acumular cambios de heading para
            // evitar saltos de 360 grados.
            double headingDelta = angleDifference(
                    rawHeading,
                    previousRawHeading
            );

            accumulatedRobotRotation += headingDelta;
            previousRawHeading = rawHeading;

            // =================================
            // 5. CORRECCION VISUAL
            // =================================

            if (visionEnabled && tagVisible) {

                if (Math.abs(tx) > TX_DEADBAND) {

                    // Movimiento maximo por segundo.
                    double requestedStep =
                            VISION_SIGN * VISION_KP * tx;

                    double maxStep = MAX_VISION_RATE * dt;

                    visionStep = clamp(
                            requestedStep,
                            -maxStep,
                            maxStep
                    );

                    visionOffsetDeg = clamp(
                            visionOffsetDeg + visionStep,
                            -MAX_VISION_OFFSET,
                            MAX_VISION_OFFSET
                    );
                }
            }

            // =================================
            // 6. OBJETIVO FINAL
            // =================================

            double pinpointTarget =
                    startingTurretAngle
                            - TRACKING_SIGN
                            * accumulatedRobotRotation;

            targetAngle = clamp(
                    pinpointTarget + visionOffsetDeg,
                    TurretSubsystem.MIN_ANGLE_DEG,
                    TurretSubsystem.MAX_ANGLE_DEG
            );

            // =================================
            // 7. VELOCIDAD PARA FEEDFORWARD
            // =================================

            targetVelocity = clamp(
                    (targetAngle - previousTarget) / dt,
                    -MAX_TARGET_VELOCITY,
                    MAX_TARGET_VELOCITY
            );

            // No empujar contra los limites.
            if (targetAngle <= TurretSubsystem.MIN_ANGLE_DEG
                    && targetVelocity < 0.0) {
                targetVelocity = 0.0;
            }

            if (targetAngle >= TurretSubsystem.MAX_ANGLE_DEG
                    && targetVelocity > 0.0) {
                targetVelocity = 0.0;
            }

            previousTarget = targetAngle;

            turret.setTargetAngleDeg(targetAngle);
            turret.setTargetVelocityDegPerSec(
                    targetVelocity
            );

            turret.update();
        }

        // =====================================
        // 8. TELEMETRIA
        // =====================================

        telemetry.addLine("===== PINPOINT =====");

        telemetry.addData(
                "Heading", "%.2f", rawHeading
        );

        telemetry.addData(
                "Robot Rotation",
                "%.2f",
                accumulatedRobotRotation
        );

        telemetry.addData(
                "Tracking",
                trackingEnabled ? "ON" : "OFF"
        );

        telemetry.addLine("===== LIMELIGHT =====");

        telemetry.addData(
                "Vision",
                visionEnabled ? "ON" : "OFF"
        );

        telemetry.addData(
                "Tag Visible",
                tagVisible ? "YES" : "NO"
        );

        telemetry.addData(
                "Tag ID",
                detectedTag
        );

        telemetry.addData(
                "TX",
                "%.2f deg",
                tx
        );

        telemetry.addData(
                "Vision Offset",
                "%.2f deg",
                visionOffsetDeg
        );

        telemetry.addData(
                "Vision Step",
                "%.3f deg",
                visionStep
        );

        telemetry.addLine("===== TURRET =====");

        telemetry.addData(
                "Target",
                "%.2f deg",
                turret.getTargetAngleDeg()
        );

        telemetry.addData(
                "Actual",
                "%.2f deg",
                turret.getCurrentAngleDeg()
        );

        telemetry.addData(
                "Error",
                "%.2f deg",
                turret.getErrorDeg()
        );

        telemetry.addData(
                "Target Velocity",
                "%.2f deg/s",
                targetVelocity
        );

        telemetry.addData(
                "Motor Power",
                "%.3f",
                turret.getMotorPower()
        );

        telemetry.update();
    }

    private double angleDifference(
            double current,
            double reference
    ) {

        double difference = current - reference;

        while (difference > 180.0) {
            difference -= 360.0;
        }

        while (difference < -180.0) {
            difference += 360.0;
        }

        return difference;
    }

    private double clamp(
            double value,
            double min,
            double max
    ) {

        return Math.max(
                min,
                Math.min(max, value)
        );
    }

    @Override
    public void stop() {

        if (limelight != null) {
            limelight.stop();
        }

        if (turret != null) {
            turret.stop();
        }

        if (driveCmd != null) {
            driveCmd.end();
        } else if (drive != null) {
            drive.stop();
        }
    }
}
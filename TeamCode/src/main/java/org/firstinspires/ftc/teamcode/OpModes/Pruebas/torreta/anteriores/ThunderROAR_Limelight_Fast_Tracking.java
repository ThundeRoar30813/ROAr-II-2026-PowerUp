package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta.anteriores;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

import java.util.List;

@TeleOp(name = "ThunderROAR - ANTI OSCILLATION")
public class ThunderROAR_Limelight_Fast_Tracking extends OpMode {

    // =====================================================
    // CONFIGURACION
    // =====================================================

    private static final int PIPELINE = 0;

    // Confirmar los IDs con la telemetria.
    private static final int TAG_1 = 34;
    private static final int TAG_2 = 36;

    // Signo del TX respecto al angulo de la torreta.
    // X permite invertirlo durante la prueba.
    private double visionSign = -1.0;

    // Ganancia visual conservadora.
    private static final double VISION_KP = 0.25;

    // Zona muerta visual.
    private static final double DEADBAND_DEG = 0.8;

    // Filtro de la lectura visual.
    private static final double TX_FILTER_ALPHA = 0.45;

    // Cambio maximo de objetivo por segundo.
    private static final double MAX_TARGET_RATE = 240.0;

    // Cambio maximo de la referencia visual por imagen.
    private static final double MAX_VISION_STEP = 1.0;

    // Rechazar lecturas antiguas.
    private static final long MAX_AGE_MS = 150;

    // Limites mecanicos.
    private static final double MIN_TURRET_ANGLE = -120.0;
    private static final double MAX_TURRET_ANGLE = 125.0;

    // =====================================================
    // HARDWARE
    // =====================================================

    private TurretSubsystem turret;
    private DriveSubsystem drive;
    private Limelight3A limelight;

    // =====================================================
    // ESTADO
    // =====================================================

    private boolean trackingEnabled = false;

    private int selectedTag = TAG_1;

    private double offsetTag1 = 0.0;
    private double offsetTag2 = 0.0;

    private double tx = Double.NaN;
    private double filteredTx = Double.NaN;
    private double visualError = Double.NaN;

    private boolean tagVisible = false;
    private boolean newFrame = false;

    private String visibleIds = "NINGUNO";
    private String status = "ESPERANDO";

    // Heading continuo del robot.
    private boolean headingInitialized = false;
    private double previousRawHeading = 0.0;
    private double continuousHeading = 0.0;

    // Direccion global de la torreta.
    // Convencion: fieldBearing = heading - turretAngle.
    private double fieldBearing = 0.0;

    // Referencia visual filtrada en coordenadas globales.
    private double visualBearing = 0.0;
    private boolean visualBearingInitialized = false;

    // Target final y objetivo deseado.
    private double targetAngle = 0.0;
    private double desiredAngle = 0.0;

    // Timestamp de la ultima imagen procesada.
    private long lastFrameTimestamp = Long.MIN_VALUE;

    private long lastLoopTime = 0L;

    private double lastVisualCorrection = 0.0;
    private double lastTargetStep = 0.0;

    // =====================================================
    // BOTONES
    // =====================================================

    private boolean oldB = false;
    private boolean oldLB = false;
    private boolean oldRB = false;
    private boolean oldX = false;
    private boolean oldUp = false;
    private boolean oldDown = false;

    // =====================================================
    // INIT
    // =====================================================

    @Override
    public void init() {

        turret = new TurretSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap);

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.pipelineSwitch(PIPELINE);
        limelight.setPollRateHz(100);

        targetAngle = turret.getCurrentAngleDeg();

        turret.setTargetAngleDeg(targetAngle);
        turret.setTargetVelocityDegPerSec(0.0);

        telemetry.addLine("THUNDERROAR - ANTI OSCILLATION");
        telemetry.addLine("B = TRACKING ON/OFF");
        telemetry.addLine("LB = TAG 34");
        telemetry.addLine("RB = TAG 36");
        telemetry.addLine("X = INVERTIR SIGNO");
        telemetry.addLine("DPAD UP/DOWN = OFFSET");
        telemetry.update();
    }

    // =====================================================
    // START
    // =====================================================

    @Override
    public void start() {

        limelight.start();

        drive.update();

        double heading = drive.getHeadingDegrees();

        previousRawHeading = heading;
        continuousHeading = heading;
        headingInitialized = true;

        targetAngle = turret.getCurrentAngleDeg();

        fieldBearing = continuousHeading - targetAngle;

        trackingEnabled = false;

        resetVision();

        lastLoopTime = System.nanoTime();
    }

    // =====================================================
    // LOOP
    // =====================================================

    @Override
    public void loop() {

        long now = System.nanoTime();

        double dt = (now - lastLoopTime) / 1e9;
        lastLoopTime = now;

        dt = clamp(dt, 0.001, 0.05);

        // ================================================
        // 1. ACTUALIZAR ODOMETRIA
        // ================================================

        drive.update();

        double rawHeading = drive.getHeadingDegrees();

        if (!headingInitialized) {

            previousRawHeading = rawHeading;
            continuousHeading = rawHeading;
            headingInitialized = true;

        } else {

            double headingDelta = wrap180(
                    rawHeading - previousRawHeading
            );

            continuousHeading += headingDelta;
            previousRawHeading = rawHeading;
        }

        double actualAngle = turret.getCurrentAngleDeg();

        // ================================================
        // 2. ACTIVAR/DESACTIVAR TRACKING
        // ================================================

        if (gamepad1.b && !oldB) {

            trackingEnabled = !trackingEnabled;

            if (trackingEnabled) {

                targetAngle = actualAngle;

                fieldBearing =
                        continuousHeading - actualAngle;

                resetVision();

                status = "TRACKING ACTIVADO";

            } else {

                turret.stop();
                status = "TRACKING OFF";
            }
        }

        oldB = gamepad1.b;

        // ================================================
        // 3. SELECCIONAR TAG
        // ================================================

        if (gamepad1.left_bumper && !oldLB) {

            selectTag(TAG_1, actualAngle);
        }

        if (gamepad1.right_bumper && !oldRB) {

            selectTag(TAG_2, actualAngle);
        }

        oldLB = gamepad1.left_bumper;
        oldRB = gamepad1.right_bumper;

        // ================================================
        // 4. INVERTIR SIGNO VISUAL
        // ================================================

        if (gamepad1.x && !oldX) {

            visionSign *= -1.0;

            resetVision();

            status = "SIGNO INVERTIDO";
        }

        oldX = gamepad1.x;

        // ================================================
        // 5. OFFSETS
        // ================================================

        if (gamepad1.dpad_up && !oldUp) {

            if (selectedTag == TAG_1) {
                offsetTag1 += 0.25;
            } else {
                offsetTag2 += 0.25;
            }

            resetVision();
        }

        if (gamepad1.dpad_down && !oldDown) {

            if (selectedTag == TAG_1) {
                offsetTag1 -= 0.25;
            } else {
                offsetTag2 -= 0.25;
            }

            resetVision();
        }

        oldUp = gamepad1.dpad_up;
        oldDown = gamepad1.dpad_down;

        // ================================================
        // 6. LEER LIMELIGHT
        // ================================================

        tagVisible = false;
        newFrame = false;

        tx = Double.NaN;
        visualError = Double.NaN;
        lastVisualCorrection = 0.0;

        visibleIds = "NINGUNO";

        LLResult result = limelight.getLatestResult();

        if (result != null
                && result.isValid()
                && result.getPipelineIndex() == PIPELINE
                && result.getStaleness() >= 0
                && result.getStaleness() <= MAX_AGE_MS) {

            List<LLResultTypes.FiducialResult> tags =
                    result.getFiducialResults();

            if (tags != null) {

                StringBuilder ids = new StringBuilder();

                for (LLResultTypes.FiducialResult tag : tags) {

                    if (ids.length() > 0) {
                        ids.append(", ");
                    }

                    ids.append(tag.getFiducialId());

                    if (tag.getFiducialId() == selectedTag) {

                        tagVisible = true;
                        tx = tag.getTargetXDegrees();
                    }
                }

                if (ids.length() > 0) {
                    visibleIds = ids.toString();
                }
            }

            if (tagVisible && Double.isFinite(tx)) {

                long frameTimestamp =
                        result.getControlHubTimeStamp();

                newFrame =
                        frameTimestamp != lastFrameTimestamp;

                if (newFrame) {

                    lastFrameTimestamp = frameTimestamp;

                    if (!Double.isFinite(filteredTx)) {

                        filteredTx = tx;

                    } else {

                        filteredTx =
                                TX_FILTER_ALPHA * tx
                                        + (1.0 - TX_FILTER_ALPHA)
                                        * filteredTx;
                    }
                }
            }
        }

        // ================================================
        // 7. CONTROL HIBRIDO
        // ================================================

        if (trackingEnabled) {

            double offset =
                    selectedTag == TAG_1
                            ? offsetTag1
                            : offsetTag2;

            if (tagVisible && newFrame
                    && Double.isFinite(filteredTx)) {

                visualError = filteredTx - offset;

                // Direccion global que observa la camara.
                // Usa el ANGULO REAL de la torreta.
                double measuredBearing =
                        continuousHeading - actualAngle;

                if (!visualBearingInitialized) {

                    visualBearing = measuredBearing;
                    visualBearingInitialized = true;
                }

                if (Math.abs(visualError) > DEADBAND_DEG) {

                    // Corregir la direccion observada.
                    double correction =
                            visionSign
                                    * VISION_KP
                                    * visualError;

                    correction = clamp(
                            correction,
                            -MAX_VISION_STEP,
                            MAX_VISION_STEP
                    );

                    // El bearing observado incorpora el
                    // movimiento REAL de la torreta.
                    double observedTargetBearing =
                            measuredBearing - correction;

                    // Llevar la observacion al angulo
                    // equivalente mas cercano.
                    observedTargetBearing = nearestEquivalent(
                            observedTargetBearing,
                            visualBearing
                    );

                    // Limitar el cambio de referencia.
                    double bearingDifference =
                            observedTargetBearing - visualBearing;

                    bearingDifference = clamp(
                            bearingDifference,
                            -MAX_VISION_STEP,
                            MAX_VISION_STEP
                    );

                    visualBearing += bearingDifference;

                    fieldBearing = visualBearing;

                    lastVisualCorrection = correction;

                } else {

                    // No cambiar la referencia por
                    // ruido pequeno alrededor de TX=0.
                    fieldBearing = visualBearing;
                }

                status = "LIMELIGHT + PINPOINT";

            } else {

                status = "PINPOINT MANTENIENDO";
            }

            // ============================================
            // 8. CALCULAR OBJETIVO SIN SALTOS DE 360
            // ============================================

            desiredAngle =
                    continuousHeading - fieldBearing;

            // Elegir representacion mas cercana
            // al angulo actual, no simplemente
            // normalizar a [-180,180].
            desiredAngle = nearestEquivalent(
                    desiredAngle,
                    actualAngle
            );

            // Nunca ordenar una vuelta completa
            // para alcanzar un angulo imposible.
            desiredAngle = clamp(
                    desiredAngle,
                    MIN_TURRET_ANGLE,
                    MAX_TURRET_ANGLE
            );

            // ============================================
            // 9. LIMITAR VELOCIDAD DEL TARGET
            // ============================================

            double maxStep = MAX_TARGET_RATE * dt;

            double targetDifference =
                    desiredAngle - targetAngle;

            lastTargetStep = clamp(
                    targetDifference,
                    -maxStep,
                    maxStep
            );

            targetAngle += lastTargetStep;

            targetAngle = clamp(
                    targetAngle,
                    MIN_TURRET_ANGLE,
                    MAX_TURRET_ANGLE
            );

            // ============================================
            // 10. MOVER TORRETA
            // ============================================

            turret.setTargetAngleDeg(targetAngle);
            turret.setTargetVelocityDegPerSec(0.0);
            turret.update();

        } else {

            turret.stop();
            status = "TRACKING OFF";
        }

        // ================================================
        // 11. TELEMETRIA
        // ================================================

        telemetry.addLine("====== LIMELIGHT ======");

        telemetry.addData("Estado", status);
        telemetry.addData("Tracking", trackingEnabled);
        telemetry.addData("Tag seleccionado", selectedTag);
        telemetry.addData("Tags detectados", visibleIds);
        telemetry.addData("Tag visible", tagVisible);
        telemetry.addData("Frame nuevo", newFrame);

        telemetry.addData("TX original", "%.2f", tx);
        telemetry.addData("TX filtrado", "%.2f", filteredTx);
        telemetry.addData("Error visual", "%.2f", visualError);

        telemetry.addData(
                "Correccion visual",
                "%.3f",
                lastVisualCorrection
        );

        telemetry.addData(
                "Vision Sign",
                "%.0f",
                visionSign
        );

        telemetry.addLine("====== PINPOINT ======");

        telemetry.addData(
                "Heading raw",
                "%.2f",
                rawHeading
        );

        telemetry.addData(
                "Heading continuo",
                "%.2f",
                continuousHeading
        );

        telemetry.addData(
                "Field Bearing",
                "%.2f",
                fieldBearing
        );

        telemetry.addLine("====== TORRETA ======");

        telemetry.addData(
                "Turret Actual",
                "%.2f",
                actualAngle
        );

        telemetry.addData(
                "Turret Desired",
                "%.2f",
                desiredAngle
        );

        telemetry.addData(
                "Turret Target",
                "%.2f",
                targetAngle
        );

        telemetry.addData(
                "Target Step",
                "%.3f",
                lastTargetStep
        );

        telemetry.addData(
                "Turret Error",
                "%.2f",
                turret.getErrorDeg()
        );

        telemetry.addData(
                "Motor Power",
                "%.3f",
                turret.getMotorPower()
        );

        telemetry.update();
    }

    // =====================================================
    // SELECCIONAR TAG
    // =====================================================

    private void selectTag(
            int tagId,
            double actualAngle
    ) {

        if (selectedTag == tagId) {
            return;
        }

        selectedTag = tagId;

        fieldBearing =
                continuousHeading - actualAngle;

        targetAngle = actualAngle;

        resetVision();

        status = "CAMBIO DE TAG";
    }

    // =====================================================
    // REINICIAR VISION
    // =====================================================

    private void resetVision() {

        filteredTx = Double.NaN;
        visualError = Double.NaN;

        lastFrameTimestamp = Long.MIN_VALUE;

        visualBearingInitialized = false;

        lastVisualCorrection = 0.0;
    }

    // =====================================================
    // ANGULO MAS CERCANO
    // =====================================================

    private static double nearestEquivalent(
            double angle,
            double reference
    ) {

        return reference + wrap180(angle - reference);
    }

    // =====================================================
    // NORMALIZAR DIFERENCIA ANGULAR
    // =====================================================

    private static double wrap180(double angle) {

        while (angle > 180.0) {
            angle -= 360.0;
        }

        while (angle < -180.0) {
            angle += 360.0;
        }

        return angle;
    }

    // =====================================================
    // CLAMP
    // =====================================================

    private static double clamp(
            double value,
            double min,
            double max
    ) {

        return Math.max(
                min,
                Math.min(max, value)
        );
    }

    // =====================================================
    // STOP
    // =====================================================

    @Override
    public void stop() {

        trackingEnabled = false;

        if (limelight != null) {
            limelight.stop();
        }

        if (turret != null) {
            turret.stop();
        }
    }
}

package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

import java.util.List;
@Disabled
@TeleOp(name = "ThunderROAR - LIMELIGHT STABLE")
public class ThunderROAR_Limelight_Stable_Test extends OpMode {

    // =====================================================
    // CONFIGURACION
    // =====================================================

    private static final int PIPELINE = 0;

    // IDs de tu configuracion anterior.
    // Confirma que sean los que detecta Limelight.
    private static final int TAG_1 = 34;
    private static final int TAG_2 = 36;

    // Si TX aumenta al corregir, invertir el signo.
    private double visionSign = -1.0;

    // Control visual conservador.
    private static final double VISION_KP = 0.30;

    // Zona muerta para evitar vibraciones.
    private static final double DEADBAND_DEG = 1.0;

    // Suavizado de TX.
    private static final double TX_FILTER_ALPHA = 0.25;

    // Maximo cambio del objetivo visual
    // por cada frame nuevo.
    private static final double MAX_CORRECTION_PER_FRAME = 1.0;

    // Limite de correccion por segundo.
    private static final double MAX_CORRECTION_RATE = 45.0;

    // Solo aceptar resultados recientes.
    private static final double MAX_AGE_MS = 150.0;

    // Tiempo maximo entre iteraciones.
    private static final double MAX_DT = 0.05;

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
    private boolean hasFieldLock = false;

    private boolean tagVisible = false;
    private boolean freshFrame = false;

    private int selectedTag = TAG_1;

    private double tx = Double.NaN;
    private double filteredTx = Double.NaN;
    private double visualError = Double.NaN;

    private double offsetTag1 = 0.0;
    private double offsetTag2 = 0.0;

    // Referencia global:
    // fieldBearing = robotHeading - turretAngle
    private double lockedFieldBearingDeg = 0.0;

    private double commandedAngleDeg = 0.0;

    private double lastCorrection = 0.0;

    private long previousLoopTime = 0;

    // Identificador del ultimo frame procesado.
    private double lastCaptureTimestamp = Double.NaN;

    private String visibleIds = "NINGUNO";
    private String status = "ESPERANDO";

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

        commandedAngleDeg = turret.getCurrentAngleDeg();

        turret.setTargetAngleDeg(commandedAngleDeg);
        turret.setTargetVelocityDegPerSec(0.0);

        telemetry.addLine("THUNDERROAR LIMELIGHT STABLE");
        telemetry.addLine("B = TRACKING ON/OFF");
        telemetry.addLine("LB = TAG 34");
        telemetry.addLine("RB = TAG 36");
        telemetry.addLine("X = INVERTIR SIGNO");
        telemetry.addLine("DPAD UP/DOWN = OFFSET TAG");
        telemetry.addLine("Sin shooter ni intake");
        telemetry.update();
    }

    // =====================================================
    // START
    // =====================================================

    @Override
    public void start() {

        limelight.start();

        trackingEnabled = false;
        hasFieldLock = false;

        previousLoopTime = System.nanoTime();

        status = "PRESIONA B";
    }

    // =====================================================
    // LOOP
    // =====================================================

    @Override
    public void loop() {

        // ================================================
        // 1. ACTUALIZAR PINPOINT
        // ================================================

        drive.update();

        double robotHeading = drive.getHeadingDegrees();
        double turretAngle = turret.getCurrentAngleDeg();

        // ================================================
        // 2. TIEMPO DEL LOOP
        // ================================================

        long now = System.nanoTime();

        double dt = previousLoopTime == 0
                ? 0.02
                : (now - previousLoopTime) / 1e9;

        previousLoopTime = now;

        dt = clamp(dt, 0.001, MAX_DT);

        // ================================================
        // 3. BOTON B - TRACKING
        // ================================================

        if (gamepad1.b && !oldB) {

            trackingEnabled = !trackingEnabled;

            if (trackingEnabled) {

                lockedFieldBearingDeg = normalize(
                        robotHeading - turretAngle
                );

                commandedAngleDeg = turretAngle;

                hasFieldLock = false;

                resetVisionFilter();

                status = "BUSCANDO APRILTAG";

            } else {

                status = "TRACKING OFF";
            }
        }

        oldB = gamepad1.b;

        // ================================================
        // 4. CAMBIAR TAG
        // ================================================

        if (gamepad1.left_bumper && !oldLB) {

            selectedTag = TAG_1;

            resetVisionFilter();

            hasFieldLock = false;

            lockedFieldBearingDeg = normalize(
                    robotHeading - turretAngle
            );

            status = "TAG 34 SELECCIONADO";
        }

        if (gamepad1.right_bumper && !oldRB) {

            selectedTag = TAG_2;

            resetVisionFilter();

            hasFieldLock = false;

            lockedFieldBearingDeg = normalize(
                    robotHeading - turretAngle
            );

            status = "TAG 36 SELECCIONADO";
        }

        oldLB = gamepad1.left_bumper;
        oldRB = gamepad1.right_bumper;

        // ================================================
        // 5. INVERTIR SIGNO
        // ================================================

        if (gamepad1.x && !oldX) {

            visionSign *= -1.0;

            resetVisionFilter();

            status = "SIGNO INVERTIDO";
        }

        oldX = gamepad1.x;

        // ================================================
        // 6. AJUSTAR OFFSET
        // ================================================

        if (gamepad1.dpad_up && !oldUp) {

            if (selectedTag == TAG_1) {
                offsetTag1 += 0.25;
            } else {
                offsetTag2 += 0.25;
            }
        }

        if (gamepad1.dpad_down && !oldDown) {

            if (selectedTag == TAG_1) {
                offsetTag1 -= 0.25;
            } else {
                offsetTag2 -= 0.25;
            }
        }

        oldUp = gamepad1.dpad_up;
        oldDown = gamepad1.dpad_down;

        // ================================================
        // 7. LEER LIMELIGHT
        // ================================================

        tagVisible = false;
        freshFrame = false;

        tx = Double.NaN;
        visualError = Double.NaN;
        lastCorrection = 0.0;

        visibleIds = "NINGUNO";

        LLResult result = limelight.getLatestResult();

        double captureTimestamp = Double.NaN;

        if (result != null && result.isValid()) {

            double ageMs = result.getStaleness();

            freshFrame =
                    ageMs >= 0.0
                            && ageMs <= MAX_AGE_MS;

            captureTimestamp =
                    result.getCaptureLatency()
                            + result.getTargetingLatency();

            List<LLResultTypes.FiducialResult> tags =
                    result.getFiducialResults();

            if (tags != null) {

                StringBuilder ids = new StringBuilder();

                for (LLResultTypes.FiducialResult tag : tags) {

                    if (ids.length() > 0) {
                        ids.append(", ");
                    }

                    ids.append(tag.getFiducialId());

                    if (freshFrame
                            && tag.getFiducialId() == selectedTag) {

                        tagVisible = true;

                        tx = tag.getTargetXDegrees();
                    }
                }

                if (ids.length() > 0) {
                    visibleIds = ids.toString();
                }
            }
        }

        // ================================================
        // 8. PROCESAR VISION
        // ================================================

        double offset =
                selectedTag == TAG_1
                        ? offsetTag1
                        : offsetTag2;

        boolean newFrame =
                tagVisible
                        && !Double.isNaN(captureTimestamp)
                        && captureTimestamp != lastCaptureTimestamp;

        if (newFrame) {

            lastCaptureTimestamp = captureTimestamp;

            // Filtro exponencial de TX.
            if (Double.isNaN(filteredTx)) {

                filteredTx = tx;

            } else {

                filteredTx =
                        TX_FILTER_ALPHA * tx
                                + (1.0 - TX_FILTER_ALPHA) * filteredTx;
            }
        }

        if (!Double.isNaN(filteredTx) && tagVisible) {

            visualError = filteredTx - offset;
        }

        // ================================================
        // 9. TRACKING HIBRIDO
        // ================================================

        if (trackingEnabled) {

            // Primero calcular el objetivo con Pinpoint.
            commandedAngleDeg = normalize(
                    robotHeading - lockedFieldBearingDeg
            );

            commandedAngleDeg = clamp(
                    commandedAngleDeg,
                    TurretSubsystem.MIN_ANGLE_DEG,
                    TurretSubsystem.MAX_ANGLE_DEG
            );

            if (newFrame && tagVisible) {

                // ========================================
                // LIMELIGHT CORRIGE LA REFERENCIA GLOBAL
                // ========================================

                if (Math.abs(visualError) > DEADBAND_DEG) {

                    double correction =
                            visionSign
                                    * VISION_KP
                                    * visualError;

                    correction = clamp(
                            correction,
                            -MAX_CORRECTION_PER_FRAME,
                            MAX_CORRECTION_PER_FRAME
                    );

                    // Limite adicional por tiempo.
                    double maxCorrection =
                            MAX_CORRECTION_RATE * dt;

                    correction = clamp(
                            correction,
                            -maxCorrection,
                            maxCorrection
                    );

                    // Actualizar direccion global.
                    lockedFieldBearingDeg = normalize(
                            lockedFieldBearingDeg - correction
                    );

                    lastCorrection = correction;
                }

                hasFieldLock = true;

                status = "LIMELIGHT CORRIGIENDO";

            } else if (hasFieldLock) {

                status = "PINPOINT MANTENIENDO";

            } else {

                status = "BUSCANDO APRILTAG";
            }

            // Recalcular despues de la correccion.
            commandedAngleDeg = normalize(
                    robotHeading - lockedFieldBearingDeg
            );

            commandedAngleDeg = clamp(
                    commandedAngleDeg,
                    TurretSubsystem.MIN_ANGLE_DEG,
                    TurretSubsystem.MAX_ANGLE_DEG
            );

            turret.setTargetAngleDeg(commandedAngleDeg);

            turret.setTargetVelocityDegPerSec(0.0);

            turret.update();

        } else {

            turret.stop();

            status = "TRACKING OFF";
        }

        // ================================================
        // 10. TELEMETRIA
        // ================================================

        telemetry.addLine(
                "========== LIMELIGHT =========="
        );

        telemetry.addData(
                "Tracking",
                trackingEnabled ? "ON" : "OFF"
        );

        telemetry.addData(
                "Estado",
                status
        );

        telemetry.addData(
                "Tag seleccionado",
                selectedTag
        );

        telemetry.addData(
                "Tags detectados",
                visibleIds
        );

        telemetry.addData(
                "Tag visible",
                tagVisible
        );

        telemetry.addData(
                "Frame reciente",
                freshFrame
        );

        telemetry.addData(
                "Frame nuevo",
                newFrame
        );

        telemetry.addData(
                "TX original",
                "%.2f",
                tx
        );

        telemetry.addData(
                "TX filtrado",
                "%.2f",
                filteredTx
        );

        telemetry.addData(
                "Error visual",
                "%.2f",
                visualError
        );

        telemetry.addData(
                "Ultima correccion",
                "%.3f",
                lastCorrection
        );

        telemetry.addData(
                "Vision Sign",
                "%.0f",
                visionSign
        );

        telemetry.addLine(
                "========== PINPOINT =========="
        );

        telemetry.addData(
                "Robot X",
                "%.2f",
                drive.getX()
        );

        telemetry.addData(
                "Robot Y",
                "%.2f",
                drive.getY()
        );

        telemetry.addData(
                "Heading",
                "%.2f",
                robotHeading
        );

        telemetry.addData(
                "Field Bearing",
                "%.2f",
                lockedFieldBearingDeg
        );

        telemetry.addData(
                "Field Lock",
                hasFieldLock
        );

        telemetry.addLine(
                "========== TORRETA =========="
        );

        telemetry.addData(
                "Turret Actual",
                "%.2f",
                turretAngle
        );

        telemetry.addData(
                "Turret Target",
                "%.2f",
                commandedAngleDeg
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
    // REINICIAR FILTRO VISUAL
    // =====================================================

    private void resetVisionFilter() {

        filteredTx = Double.NaN;

        lastCaptureTimestamp = Double.NaN;

        lastCorrection = 0.0;
    }

    // =====================================================
    // NORMALIZAR ANGULO
    // =====================================================

    private static double normalize(double angle) {

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

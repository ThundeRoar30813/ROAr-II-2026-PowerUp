
package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class TurretVisionV5 {

    // ==========================================
    // CONFIGURACION
    // ==========================================

    public static int PIPELINE = 0;

    // Antiguedad maxima del resultado
    public static double MAX_FRAME_AGE_MS = 150.0;

    // Angulo horizontal maximo permitido
    public static double MAX_TX_DEG = 27.0;

    // ==========================================
    // HARDWARE
    // ==========================================

    private final Limelight3A limelight;

    // ==========================================
    // APRILTAGS
    // ==========================================

    private int selectedTag = 37;
    private int detectedTag = -1;

    private int detectedTagCount = 0;

    // ==========================================
    // VISION
    // ==========================================

    private boolean targetVisible = false;
    private boolean anyTagVisible = false;
    private boolean resultValid = false;

    private double tx = Double.NaN;
    private double ty = Double.NaN;

    private double ageMs = -1;

    private String status = "NOT_STARTED";

    // ==========================================
    // CONSTRUCTOR
    // ==========================================

    public TurretVisionV5(HardwareMap hardwareMap) {

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.pipelineSwitch(PIPELINE);
    }

    // ==========================================
    // START / STOP
    // ==========================================

    public void start() {

        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        status = "STARTED";
    }

    public void stop() {

        limelight.stop();

        targetVisible = false;
        anyTagVisible = false;

        status = "STOPPED";
    }

    // ==========================================
    // SELECTED TAG
    // ==========================================

    public void setSelectedTag(int id) {

        if (id <= 0) {
            return;
        }

        selectedTag = id;
    }

    public int getSelectedTag() {
        return selectedTag;
    }

    // ==========================================
    // GETTERS
    // ==========================================

    public boolean hasTarget() {
        return targetVisible;
    }

    public boolean hasAnyTag() {
        return anyTagVisible;
    }

    public boolean isResultValid() {
        return resultValid;
    }

    public int getDetectedTag() {
        return detectedTag;
    }

    public int getDetectedTagCount() {
        return detectedTagCount;
    }

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public double getAgeMs() {
        return ageMs;
    }

    public String getStatus() {
        return status;
    }

    // ==========================================
    // UPDATE
    // ==========================================

    public void update() {

        // Reiniciar resultados del ciclo
        targetVisible = false;
        anyTagVisible = false;
        resultValid = false;

        detectedTag = -1;
        detectedTagCount = 0;

        tx = Double.NaN;
        ty = Double.NaN;
        ageMs = -1;

        // ======================================
        // 1. LEER LIMELIGHT
        // ======================================

        LLResult result = limelight.getLatestResult();

        if (result == null) {

            status = "NO_RESULT";
            return;
        }

        // ======================================
        // 2. VALIDAR RESULTADO
        // ======================================

        resultValid = result.isValid();

        if (!resultValid) {

            status = "INVALID_RESULT";
            return;
        }

        // ======================================
        // 3. COMPROBAR ANTIGUEDAD
        // ======================================

        ageMs = result.getStaleness();

        if (!Double.isFinite(ageMs) ||
                ageMs < 0 ||
                ageMs > MAX_FRAME_AGE_MS) {

            status = "STALE_RESULT";
            return;
        }

        // ======================================
        // 4. OBTENER APRILTAGS
        // ======================================

        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();

        if (fiducials == null ||
                fiducials.isEmpty()) {

            status = "NO_APRILTAGS";
            return;
        }

        detectedTagCount = fiducials.size();

        // ======================================
        // 5. RECORRER APRILTAGS
        // ======================================

        for (LLResultTypes.FiducialResult fiducial
                : fiducials) {

            int id = fiducial.getFiducialId();

            // Guardar el primer tag detectado
            if (!anyTagVisible) {

                detectedTag = id;
                anyTagVisible = true;
            }

            // Buscar solamente el tag elegido
            if (id != selectedTag) {
                continue;
            }

            // ==================================
            // 6. LEER TX / TY
            // ==================================

            double measuredTx =
                    fiducial.getTargetXDegrees();

            double measuredTy =
                    fiducial.getTargetYDegrees();

            if (!Double.isFinite(measuredTx) ||
                    !Double.isFinite(measuredTy)) {

                status = "INVALID_TX_TY";
                continue;
            }

            if (Math.abs(measuredTx) > MAX_TX_DEG) {

                status = "TX_OUT_OF_RANGE";
                continue;
            }

            // ==================================
            // 7. OBJETIVO VALIDO
            // ==================================

            tx = measuredTx;
            ty = measuredTy;

            detectedTag = id;
            targetVisible = true;

            status = "TARGET_LOCKED";

            break;
        }

        // ======================================
        // 8. NO SE ENCONTRO EL TAG ELEGIDO
        // ======================================

        if (!targetVisible &&
                !status.equals("INVALID_TX_TY") &&
                !status.equals("TX_OUT_OF_RANGE")) {

            status = "SELECTED_TAG_NOT_FOUND";
        }
    }
}

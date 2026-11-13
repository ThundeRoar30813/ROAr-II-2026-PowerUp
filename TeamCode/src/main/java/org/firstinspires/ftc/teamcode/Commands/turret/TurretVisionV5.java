
package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class TurretVisionV5 {

    public static int PIPELINE = 0;
    public static double MAX_FRAME_AGE_MS = 150.0;
    public static double MAX_TX_DEG = 27.0;

    // Deteccion aproximada de fotograma nuevo
    public static double FRAME_RESET_THRESHOLD_MS = 5.0;

    private final Limelight3A limelight;


    private double distanceToTagMeters = Double.NaN;


    private int selectedTag = 37;
    private int detectedTag = -1;
    private int detectedTagCount = 0;

    private boolean targetVisible = false;
    private boolean anyTagVisible = false;
    private boolean resultValid = false;
    private boolean newFrame = false;

    private long frameSequence = 0;

    private double previousAgeMs = Double.NaN;
    private double tx = Double.NaN;
    private double ty = Double.NaN;
    private double ageMs = -1;

    private String status = "NOT_STARTED";

    public TurretVisionV5(HardwareMap hardwareMap) {

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.pipelineSwitch(PIPELINE);
    }

    public void start() {

        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        previousAgeMs = Double.NaN;
        newFrame = false;

        status = "STARTED";
    }

    public void stop() {

        limelight.stop();

        targetVisible = false;
        anyTagVisible = false;
        newFrame = false;

        status = "STOPPED";
    }

    public void setSelectedTag(int id) {

        if (id <= 0) return;

        if (selectedTag != id) {
            selectedTag = id;
            targetVisible = false;
            previousAgeMs = Double.NaN;
        }
    }

    public int getSelectedTag() {
        return selectedTag;
    }

    public boolean hasTarget() {
        return targetVisible;
    }

    public boolean hasAnyTag() {
        return anyTagVisible;
    }

    public boolean isResultValid() {
        return resultValid;
    }

    public boolean isNewFrame() {
        return newFrame;
    }

    public long getFrameSequence() {
        return frameSequence;
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


    public double getDistanceToTagMeters() {
        return distanceToTagMeters;
    }


    public void update() {


        distanceToTagMeters = Double.NaN;


        newFrame = false;
        targetVisible = false;
        anyTagVisible = false;
        resultValid = false;

        detectedTag = -1;
        detectedTagCount = 0;

        tx = Double.NaN;
        ty = Double.NaN;
        ageMs = -1;

        LLResult result = limelight.getLatestResult();

        if (result == null) {
            status = "NO_RESULT";
            return;
        }

        resultValid = result.isValid();

        if (!resultValid) {
            previousAgeMs = Double.NaN;
            status = "INVALID_RESULT";
            return;
        }

        ageMs = result.getStaleness();

        if (!Double.isFinite(ageMs)
                || ageMs < 0
                || ageMs > MAX_FRAME_AGE_MS) {

            status = "STALE_RESULT";
            previousAgeMs = Double.NaN;
            return;
        }

        // Un fotograma nuevo suele hacer que
        // disminuya la antiguedad del resultado.
        if (!Double.isFinite(previousAgeMs)) {

            newFrame = true;

        } else if (
                previousAgeMs - ageMs
                        >= FRAME_RESET_THRESHOLD_MS) {

            newFrame = true;
        }

        previousAgeMs = ageMs;

        if (newFrame) {
            frameSequence++;
        }

        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {

            status = "NO_APRILTAGS";
            return;
        }

        detectedTagCount = fiducials.size();

        for (LLResultTypes.FiducialResult fiducial
                : fiducials) {

            int id = fiducial.getFiducialId();


            if (!anyTagVisible) {

                detectedTag = id;
                anyTagVisible = true;
            }

            if (id != selectedTag) {
                continue;
            }

            double measuredTx =
                    fiducial.getTargetXDegrees();

            double measuredTy =
                    fiducial.getTargetYDegrees();

            if (!Double.isFinite(measuredTx)
                    || !Double.isFinite(measuredTy)) {

                status = "INVALID_TX_TY";
                continue;
            }

            if (Math.abs(measuredTx) > MAX_TX_DEG) {

                status = "TX_OUT_OF_RANGE";
                continue;
            }

            tx = measuredTx;
            ty = measuredTy;


            if (fiducial.getTargetPoseCameraSpace() != null &&
                    fiducial.getTargetPoseCameraSpace().getPosition() != null) {

                double x = fiducial.getTargetPoseCameraSpace()
                        .getPosition().x;
                double y = fiducial.getTargetPoseCameraSpace()
                        .getPosition().y;
                double z = fiducial.getTargetPoseCameraSpace()
                        .getPosition().z;

                if (Double.isFinite(x) &&
                        Double.isFinite(y) &&
                        Double.isFinite(z)) {

                    // Pose 3D expresada en metros.
                    distanceToTagMeters = Math.sqrt(
                            x * x + y * y + z * z
                    );
                }
            }


            detectedTag = id;
            targetVisible = true;

            status = "TARGET_LOCKED";
            break;
        }

        if (!targetVisible
                && !status.equals("INVALID_TX_TY")
                && !status.equals("TX_OUT_OF_RANGE")) {

            status = "SELECTED_TAG_NOT_FOUND";
        }
    }
}

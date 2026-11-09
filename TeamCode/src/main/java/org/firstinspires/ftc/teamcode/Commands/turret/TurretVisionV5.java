
package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class TurretVisionV5 {

    private final Limelight3A limelight;

    private int selectedTag = 37;
    private boolean visible;

    private double tx = Double.NaN;
    private double ty = Double.NaN;
    private double ageMs = Double.NaN;

    public static final double MAX_AGE_MS = 150.0;

    public TurretVisionV5(HardwareMap map) {

        limelight = map.get(
                Limelight3A.class, "limelight"
        );

        // Pipeline 0 debe detectar AprilTags.
        limelight.pipelineSwitch(0);
    }

    public void start() {
        limelight.start();
    }

    public void setSelectedTag(int id) {
        selectedTag = id;
    }

    public void update() {

        visible = false;

        tx = Double.NaN;
        ty = Double.NaN;
        ageMs = Double.NaN;

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return;
        }

        ageMs = result.getStaleness();

        if (!Double.isFinite(ageMs) ||
                ageMs > MAX_AGE_MS) {
            return;
        }

        List<LLResultTypes.FiducialResult> tags =
                result.getFiducialResults();

        if (tags == null) return;

        for (LLResultTypes.FiducialResult tag : tags) {

            if (tag.getFiducialId() != selectedTag) {
                continue;
            }

            double x = tag.getTargetXDegrees();
            double y = tag.getTargetYDegrees();

            if (!Double.isFinite(x) ||
                    !Double.isFinite(y) ||
                    Math.abs(x) > 27) {
                return;
            }

            tx = x;
            ty = y;
            visible = true;
            return;
        }
    }

    public int getSelectedTag() {
        return selectedTag;
    }

    public boolean hasTarget() {
        return visible;
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

    public void stop() {
        limelight.stop();
    }
}

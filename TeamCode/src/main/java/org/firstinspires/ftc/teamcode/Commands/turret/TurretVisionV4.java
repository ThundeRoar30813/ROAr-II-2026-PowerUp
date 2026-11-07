
package org.firstinspires.ftc.teamcode.Commands.turret;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretVisionV4 {

    public static final int TAG_ID = 36;
    public static final int PIPELINE = 0;

    private final Limelight3A camera;

    private boolean visible = false;
    private double tx = 0.0;
    private double ty = 0.0;
    private long ageMs = -1;

    public TurretVisionV4(HardwareMap map) {
        camera = map.get(Limelight3A.class, "limelight");
    }

    public void start() {
        camera.pipelineSwitch(PIPELINE);
        camera.start();
    }

    public void update() {

        visible = false;
        ageMs = -1;

        LLResult result = camera.getLatestResult();

        if (result == null || !result.isValid()) {
            return;
        }

        ageMs = result.getStaleness();

        if (ageMs < 0 || ageMs > 150) {
            return;
        }

        if (result.getFiducialResults() == null) {
            return;
        }

        for (LLResultTypes.FiducialResult tag :
                result.getFiducialResults()) {

            if (tag.getFiducialId() != TAG_ID) {
                continue;
            }

            double x = tag.getTargetXDegrees();
            double y = tag.getTargetYDegrees();

            if (!Double.isFinite(x)
                    || !Double.isFinite(y)) {
                return;
            }

            tx = x;
            ty = y;
            visible = true;
            return;
        }
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

    public long getAgeMs() {
        return ageMs;
    }

    public void stop() {
        camera.stop();
    }
}

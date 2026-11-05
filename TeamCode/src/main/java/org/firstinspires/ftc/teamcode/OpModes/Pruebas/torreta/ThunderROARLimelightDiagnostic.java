
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ThunderROAR - Limelight Diagnostic")
public class ThunderROARLimelightDiagnostic extends OpMode {

    private Limelight3A limelight;

    private static final int PIPELINE = 0;
    private static final int RED_TARGET_1 = 34;
    private static final int RED_TARGET_2 = 37;

    private static final double MAX_AGE_MS = 150.0;

    @Override
    public void init() {

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        telemetry.addLine("Limelight Diagnostic READY");
        telemetry.update();
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        boolean found34 = false;
        boolean found37 = false;

        double tx34 = Double.NaN;
        double tx37 = Double.NaN;

        if (result != null) {

            telemetry.addData(
                    "Result Valid",
                    result.isValid()
            );

            telemetry.addData(
                    "Result Age (ms)",
                    "%.1f",
                    result.getStaleness()
            );

            telemetry.addData(
                    "General TX",
                    "%.2f",
                    result.getTx()
            );

            boolean fresh =
                    result.getStaleness() <= MAX_AGE_MS;

            if (result.isValid() && fresh) {

                for (LLResultTypes.FiducialResult tag :
                        result.getFiducialResults()) {

                    int id = tag.getFiducialId();

                    if (id == RED_TARGET_1) {
                        found34 = true;
                        tx34 = tag.getTargetXDegrees();
                    }

                    if (id == RED_TARGET_2) {
                        found37 = true;
                        tx37 = tag.getTargetXDegrees();
                    }
                }
            }

        } else {

            telemetry.addLine("Sin resultado de Limelight");
        }

        telemetry.addLine("===== BIO BUZZ RED =====");

        telemetry.addData("ID 34 Visible", found34);
        telemetry.addData("ID 34 TX", "%.2f", tx34);

        telemetry.addData("ID 37 Visible", found37);
        telemetry.addData("ID 37 TX", "%.2f", tx37);

        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}

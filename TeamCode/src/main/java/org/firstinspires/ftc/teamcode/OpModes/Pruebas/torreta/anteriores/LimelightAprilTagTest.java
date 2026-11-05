package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta.anteriores;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;
@Disabled
@TeleOp(name = "ThunderROAR - Limelight AprilTag Test")
public class LimelightAprilTagTest extends OpMode {

    private Limelight3A limelight;

    // Pipeline de AprilTags configurado en Limelight
    private static final int APRILTAG_PIPELINE = 0;

    @Override
    public void init() {

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.pipelineSwitch(APRILTAG_PIPELINE);

        telemetry.addLine("===== LIMELIGHT APRILTAG TEST =====");
        telemetry.addLine("Pipeline: 0");
        telemetry.addLine("Esperando START...");
        telemetry.update();
    }

    @Override
    public void start() {

        limelight.start();
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        telemetry.addLine("===== LIMELIGHT 3A =====");

        if (result == null || !result.isValid()) {

            telemetry.addLine("Sin objetivo valido");
            telemetry.update();
            return;
        }

        telemetry.addData("TX", "%.2f grados", result.getTx());
        telemetry.addData("TY", "%.2f grados", result.getTy());
        telemetry.addData("TA", "%.2f", result.getTa());

        List<LLResultTypes.FiducialResult> tags =
                result.getFiducialResults();

        if (tags == null || tags.isEmpty()) {

            telemetry.addLine("No se detectaron AprilTags");
            telemetry.update();
            return;
        }

        telemetry.addData("AprilTags detectados", tags.size());

        for (LLResultTypes.FiducialResult tag : tags) {

            int id = tag.getFiducialId();

            telemetry.addLine("-------------------------");

            telemetry.addData("AprilTag ID", id);

            telemetry.addData(
                    "Objetivo",
                    id == 34 ? "ESQUINA 34" :
                            id == 36 ? "ESQUINA 36" :
                            "OTRO TAG"
            );

            /*
             * Posición de la cámara respecto al AprilTag.
             *
             * El método devuelve una Pose3D.
             * La traslación está expresada en metros.
             */

            double x = tag.getCameraPoseTargetSpace()
                    .getPosition().x;

            double y = tag.getCameraPoseTargetSpace()
                    .getPosition().y;

            double z = tag.getCameraPoseTargetSpace()
                    .getPosition().z;

            // Distancia 3D en metros
            double distanceMeters =
                    Math.sqrt(x * x + y * y + z * z);

            // Convertir a pulgadas
            double distanceInches =
                    distanceMeters * 39.3701;

            telemetry.addData(
                    "Distancia",
                    "%.2f pulgadas",
                    distanceInches
            );

            telemetry.addData(
                    "Distancia metros",
                    "%.3f m",
                    distanceMeters
            );

            telemetry.addData("X", "%.3f m", x);
            telemetry.addData("Y", "%.3f m", y);
            telemetry.addData("Z", "%.3f m", z);
        }

        telemetry.update();
    }

    @Override
    public void stop() {

        if (limelight != null) {
            limelight.stop();
        }
    }
}
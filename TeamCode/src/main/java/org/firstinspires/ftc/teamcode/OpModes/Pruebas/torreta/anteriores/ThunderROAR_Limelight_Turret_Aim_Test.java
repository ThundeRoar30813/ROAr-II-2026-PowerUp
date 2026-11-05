
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta.anteriores;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

import java.util.List;
@Disabled
@TeleOp(name = "ThunderROAR - Limelight Turret AIM TEST")
public class ThunderROAR_Limelight_Turret_Aim_Test extends OpMode {

    private Limelight3A limelight;
    private TurretSubsystem turret;

    // ==========================================
    // LIMELIGHT
    // ==========================================

    private static final int PIPELINE = 0;

    private static final int TAG_LEFT = 34;
    private static final int TAG_RIGHT = 36;

    private int selectedId = TAG_LEFT;

    // ==========================================
    // OFFSET HACIA CENTRO DE ABERTURA
    // ==========================================

    private double offset34Deg = 0.0;
    private double offset36Deg = 0.0;

    private static final double OFFSET_STEP_DEG = 0.25;

    // ==========================================
    // CONTROL DE TORRETA
    // ==========================================

    // Si gira alejandose del objetivo, cambiar a -1.0
    private static final double TX_SIGN = 1.0;

    private static final double CORRECTION_GAIN = 0.30;

    private static final double DEAD_BAND_DEG = 0.8;

    private static final double MAX_STEP_DEG = 1.5;

    // AJUSTAR al limite mecanico seguro del robot
    private static final double MAX_ABS_TURRET_DEG = 160.0;

    private static final double MAX_RESULT_AGE_MS = 150.0;

    // ==========================================
    // ESTADOS
    // ==========================================

    private boolean aiming = false;

    private boolean prevA = false;
    private boolean prevLeft = false;
    private boolean prevRight = false;
    private boolean prevUp = false;
    private boolean prevDown = false;

    private double lastTx = Double.NaN;
    private double lastError = Double.NaN;

    private boolean tagVisible = false;
    private boolean frameFresh = false;

    private String status = "ESPERANDO";

    // ==========================================
    // INIT
    // ==========================================

    @Override
    public void init() {

        turret = new TurretSubsystem(hardwareMap);

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.pipelineSwitch(PIPELINE);

        telemetry.addLine("===== LIMELIGHT TURRET TEST =====");
        telemetry.addLine("A = ACTIVAR / DESACTIVAR");
        telemetry.addLine("DPAD LEFT = APRILTAG 34");
        telemetry.addLine("DPAD RIGHT = APRILTAG 36");
        telemetry.addLine("DPAD UP = OFFSET +0.25");
        telemetry.addLine("DPAD DOWN = OFFSET -0.25");
        telemetry.addLine("");
        telemetry.addLine("PROBAR SIN PELOTAS");
        telemetry.update();
    }

    // ==========================================
    // START
    // ==========================================

    @Override
    public void start() {

        limelight.start();

        turret.setTargetAngleDeg(
                turret.getCurrentAngleDeg()
        );
    }

    // ==========================================
    // LOOP
    // ==========================================

    @Override
    public void loop() {

        // ======================================
        // BOTONES
        // ======================================

        boolean a = gamepad1.a;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        // ACTIVAR / DESACTIVAR
        if (a && !prevA) {

            aiming = !aiming;

            if (!aiming) {

                turret.setTargetAngleDeg(
                        turret.getCurrentAngleDeg()
                );
            }
        }

        // APRILTAG 34
        if (left && !prevLeft) {
            selectedId = TAG_LEFT;
        }

        // APRILTAG 36
        if (right && !prevRight) {
            selectedId = TAG_RIGHT;
        }

        // OFFSET +
        if (up && !prevUp) {

            if (selectedId == TAG_LEFT) {
                offset34Deg += OFFSET_STEP_DEG;
            } else {
                offset36Deg += OFFSET_STEP_DEG;
            }
        }

        // OFFSET -
        if (down && !prevDown) {

            if (selectedId == TAG_LEFT) {
                offset34Deg -= OFFSET_STEP_DEG;
            } else {
                offset36Deg -= OFFSET_STEP_DEG;
            }
        }

        prevA = a;
        prevLeft = left;
        prevRight = right;
        prevUp = up;
        prevDown = down;

        // ======================================
        // LEER LIMELIGHT
        // ======================================

        tagVisible = false;
        frameFresh = false;

        lastTx = Double.NaN;
        lastError = Double.NaN;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double ageMs = result.getStaleness();

            frameFresh =
                    ageMs >= 0 &&
                            ageMs <= MAX_RESULT_AGE_MS;

            List<LLResultTypes.FiducialResult> tags =
                    result.getFiducialResults();

            if (tags != null) {

                for (LLResultTypes.FiducialResult tag : tags) {

                    if (tag.getFiducialId() == selectedId) {

                        tagVisible = true;

                        lastTx = tag.getTargetXDegrees();

                        break;
                    }
                }
            }
        }

        // ======================================
        // CONTROL DE APUNTADO
        // ======================================

        if (!aiming) {

            turret.setTargetAngleDeg(
                    turret.getCurrentAngleDeg()
            );

            status = "APUNTADO DESACTIVADO";

        } else if (!frameFresh) {

            status = "RESULTADO ANTIGUO / NO VALIDO";

        } else if (!tagVisible) {

            status = "TAG SELECCIONADO NO VISIBLE";

        } else {

            double offset =
                    selectedId == TAG_LEFT
                            ? offset34Deg
                            : offset36Deg;

            // Error respecto al centro de abertura
            lastError = lastTx - offset;

            if (Math.abs(lastError) <= DEAD_BAND_DEG) {

                status = "CENTRADO";

            } else {

                double correction =
                        TX_SIGN *
                                CORRECTION_GAIN *
                                lastError;

                // Limitar correccion por ciclo
                correction = Math.max(
                        -MAX_STEP_DEG,
                        Math.min(MAX_STEP_DEG, correction)
                );

                double desired =
                        turret.getCurrentAngleDeg() +
                                correction;

                // Limites de seguridad
                desired = Math.max(
                        -MAX_ABS_TURRET_DEG,
                        Math.min(MAX_ABS_TURRET_DEG, desired)
                );

                turret.setTargetAngleDeg(desired);

                status = "CORRIGIENDO";
            }
        }

        // ======================================
        // ACTUALIZAR PID DE TORRETA
        // ======================================

        turret.update();

        // ======================================
        // TELEMETRIA
        // ======================================

        telemetry.addLine("===== LIMELIGHT + TURRET =====");

        telemetry.addData(
                "Apuntado",
                aiming ? "ON" : "OFF"
        );

        telemetry.addData(
                "Tag seleccionado",
                selectedId
        );

        telemetry.addData(
                "Tag visible",
                tagVisible
        );

        telemetry.addData(
                "Frame reciente",
                frameFresh
        );

        telemetry.addData(
                "TX del tag",
                "%.2f deg",
                lastTx
        );

        telemetry.addData(
                "Error abertura",
                "%.2f deg",
                lastError
        );

        telemetry.addData(
                "Offset 34",
                "%.2f deg",
                offset34Deg
        );

        telemetry.addData(
                "Offset 36",
                "%.2f deg",
                offset36Deg
        );

        telemetry.addData(
                "Turret actual",
                "%.2f deg",
                turret.getCurrentAngleDeg()
        );

        telemetry.addData(
                "Turret objetivo",
                "%.2f deg",
                turret.getTargetAngleDeg()
        );

        telemetry.addData(
                "Estado",
                status
        );

        telemetry.addLine("");
        telemetry.addLine(
                "A ON/OFF | IZQ 34 | DER 36"
        );

        telemetry.addLine(
                "ARRIBA/ABAJO = OFFSET"
        );

        telemetry.update();
    }

    // ==========================================
    // STOP
    // ==========================================

    @Override
    public void stop() {

        aiming = false;

        if (limelight != null) {
            limelight.stop();
        }

        if (turret != null) {
            turret.stop();
        }
    }
}


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
@TeleOp(name = "ThunderROAR - LIMELIGHT + PINPOINT TEST")
public class ThunderROAR_Limelight_Pinpoint_Test extends OpMode {

    // ==============================================
    // CONFIGURACION
    // ==============================================

    private static final int PIPELINE = 0;

    // IDs anteriores. Confirmar con telemetria.
    private static final int TAG_1 = 34;
    private static final int TAG_2 = 36;

    // Si la torreta se aleja del AprilTag,
    // cambiar este signo o pulsar X.
    private double visionSign = 1.0;

    // Ganancia de correccion visual.
    private static final double VISION_GAIN = 0.35;

    // Error horizontal permitido.
    private static final double DEADBAND_DEG = 0.8;

    // Maxima variacion de objetivo por ciclo.
    private static final double MAX_STEP_DEG = 2.0;

    // Evita datos demasiado antiguos.
    private static final double MAX_AGE_MS = 150.0;

    // ==============================================
    // HARDWARE
    // ==============================================

    private TurretSubsystem turret;
    private DriveSubsystem drive;
    private Limelight3A limelight;

    // ==============================================
    // ESTADO
    // ==============================================

    private int selectedTag = TAG_1;

    private boolean trackingEnabled = false;
    private boolean hasFieldLock = false;

    private boolean tagVisible = false;
    private boolean freshFrame = false;

    private double tx = Double.NaN;
    private double errorDeg = Double.NaN;

    // Direccion global almacenada.
    // Convencion usada por nuestro tracking:
    // fieldBearing = heading - turretAngle
    private double lockedFieldBearingDeg = 0.0;

    // Offset entre centro del tag y punto de disparo.
    private double offsetTag1Deg = 0.0;
    private double offsetTag2Deg = 0.0;

    private double commandedAngleDeg = 0.0;

    private String status = "ESPERANDO";

    private String visibleIds = "NINGUNO";

    // ==============================================
    // BOTONES
    // ==============================================

    private boolean oldB = false;
    private boolean oldLB = false;
    private boolean oldRB = false;
    private boolean oldX = false;
    private boolean oldUp = false;
    private boolean oldDown = false;

    // ==============================================
    // INIT
    // ==============================================

    @Override
    public void init() {

        turret = new TurretSubsystem(hardwareMap);

        drive = new DriveSubsystem(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(PIPELINE);

        commandedAngleDeg = turret.getCurrentAngleDeg();

        turret.setTargetAngleDeg(commandedAngleDeg);
        turret.setTargetVelocityDegPerSec(0);

        telemetry.addLine("LIMELIGHT + PINPOINT TEST");
        telemetry.addLine("B = TRACKING ON/OFF");
        telemetry.addLine("LB = TAG 34");
        telemetry.addLine("RB = TAG 36");
        telemetry.addLine("X = INVERTIR SIGNO VISION");
        telemetry.addLine("DPAD UP/DOWN = OFFSET DEL TAG");
        telemetry.addLine("Sin shooter ni intake");
        telemetry.update();
    }

    // ==============================================
    // START
    // ==============================================

    @Override
    public void start() {

        limelight.start();

        trackingEnabled = false;
        hasFieldLock = false;

        status = "PRESIONA B";
    }

    // ==============================================
    // LOOP
    // ==============================================

    @Override
    public void loop() {

        // ------------------------------------------
        // 1. ACTUALIZAR PINPOINT
        // ------------------------------------------

        drive.update();

        double robotHeading =
                drive.getHeadingDegrees();

        double turretAngle =
                turret.getCurrentAngleDeg();

        // ------------------------------------------
        // 2. BOTON B
        // ------------------------------------------

        if (gamepad1.b && !oldB) {

            trackingEnabled = !trackingEnabled;

            if (trackingEnabled) {

                // Al activar, conservar la direccion
                // actual hasta encontrar un AprilTag.
                lockedFieldBearingDeg = normalize(
                        robotHeading - turretAngle
                );

                hasFieldLock = false;

                commandedAngleDeg = turretAngle;

                status = "BUSCANDO APRILTAG";

            } else {

                commandedAngleDeg = turretAngle;

                status = "TRACKING OFF";
            }
        }

        oldB = gamepad1.b;

        // ------------------------------------------
        // 3. SELECCIONAR TAG
        // ------------------------------------------

        if (gamepad1.left_bumper && !oldLB) {

            selectedTag = TAG_1;

            hasFieldLock = false;

            status = "BUSCANDO TAG 34";
        }

        if (gamepad1.right_bumper && !oldRB) {

            selectedTag = TAG_2;

            hasFieldLock = false;

            status = "BUSCANDO TAG 36";
        }

        oldLB = gamepad1.left_bumper;
        oldRB = gamepad1.right_bumper;

        // ------------------------------------------
        // 4. INVERTIR SIGNO VISUAL
        // ------------------------------------------

        if (gamepad1.x && !oldX) {

            visionSign *= -1.0;

            status = "SIGNO VISION CAMBIADO";
        }

        oldX = gamepad1.x;

        // ------------------------------------------
        // 5. AJUSTAR OFFSET
        // ------------------------------------------

        if (gamepad1.dpad_up && !oldUp) {

            if (selectedTag == TAG_1) {
                offsetTag1Deg += 0.25;
            } else {
                offsetTag2Deg += 0.25;
            }
        }

        if (gamepad1.dpad_down && !oldDown) {

            if (selectedTag == TAG_1) {
                offsetTag1Deg -= 0.25;
            } else {
                offsetTag2Deg -= 0.25;
            }
        }

        oldUp = gamepad1.dpad_up;
        oldDown = gamepad1.dpad_down;

        // ------------------------------------------
        // 6. LEER LIMELIGHT
        // ------------------------------------------

        tagVisible = false;
        freshFrame = false;

        tx = Double.NaN;
        errorDeg = Double.NaN;

        visibleIds = "NINGUNO";

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double ageMs = result.getStaleness();

            freshFrame =
                    ageMs >= 0.0
                            && ageMs <= MAX_AGE_MS;

            List<LLResultTypes.FiducialResult> tags =
                    result.getFiducialResults();

            if (tags != null) {

                StringBuilder ids = new StringBuilder();

                for (LLResultTypes.FiducialResult tag : tags) {

                    int id = tag.getFiducialId();

                    if (ids.length() > 0) {
                        ids.append(", ");
                    }

                    ids.append(id);

                    if (freshFrame && id == selectedTag) {

                        tagVisible = true;

                        tx = tag.getTargetXDegrees();
                    }
                }

                if (ids.length() > 0) {
                    visibleIds = ids.toString();
                }
            }
        }

        // ------------------------------------------
        // 7. CALCULAR ERROR VISUAL
        // ------------------------------------------

        double offset =
                selectedTag == TAG_1
                        ? offsetTag1Deg
                        : offsetTag2Deg;

        if (tagVisible) {

            errorDeg = tx - offset;
        }

        // ------------------------------------------
        // 8. TRACKING
        // ------------------------------------------

        if (trackingEnabled) {

            if (tagVisible) {

                // ==================================
                // MODO LIMELIGHT
                // ==================================

                double correction = 0.0;

                if (Math.abs(errorDeg) > DEADBAND_DEG) {

                    correction =
                            visionSign
                                    * VISION_GAIN
                                    * errorDeg;

                    correction = clamp(
                            correction,
                            -MAX_STEP_DEG,
                            MAX_STEP_DEG
                    );
                }

                // Corregir desde el objetivo actual
                // de tracking, no desde cero.
                commandedAngleDeg = clamp(
                        commandedAngleDeg + correction,
                        TurretSubsystem.MIN_ANGLE_DEG,
                        TurretSubsystem.MAX_ANGLE_DEG
                );

                // Si la vision ya centro el tag,
                // mantener la posicion actual.
                if (Math.abs(errorDeg) <= DEADBAND_DEG) {

                    commandedAngleDeg = turretAngle;
                }

                // Guardar direccion global
                // para utilizarla cuando se pierda
                // el AprilTag.
                lockedFieldBearingDeg = normalize(
                        robotHeading - commandedAngleDeg
                );

                hasFieldLock = true;

                status = "LIMELIGHT APUNTANDO";

            } else if (hasFieldLock) {

                // ==================================
                // MODO PINPOINT
                // ==================================

                // Mantener la direccion global
                // aunque el chasis gire.
                commandedAngleDeg = normalize(
                        robotHeading
                                - lockedFieldBearingDeg
                );

                commandedAngleDeg = clamp(
                        commandedAngleDeg,
                        TurretSubsystem.MIN_ANGLE_DEG,
                        TurretSubsystem.MAX_ANGLE_DEG
                );

                status = "PINPOINT - TAG PERDIDO";

            } else {

                // Todavia no se encontro ningun tag.
                // Mantener direccion global inicial.
                commandedAngleDeg = normalize(
                        robotHeading
                                - lockedFieldBearingDeg
                );

                commandedAngleDeg = clamp(
                        commandedAngleDeg,
                        TurretSubsystem.MIN_ANGLE_DEG,
                        TurretSubsystem.MAX_ANGLE_DEG
                );

                status = "BUSCANDO TAG - SIN LOCK";
            }

            turret.setTargetAngleDeg(commandedAngleDeg);

            turret.setTargetVelocityDegPerSec(0.0);

            turret.update();

        } else {

            turret.stop();

            status = "TRACKING OFF";
        }

        // ------------------------------------------
        // 9. TELEMETRIA
        // ------------------------------------------

        telemetry.addLine(
                "========== LIMELIGHT =========="
        );

        telemetry.addData(
                "Tracking",
                trackingEnabled ? "ON" : "OFF"
        );

        telemetry.addData(
                "Modo",
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
                "Frame reciente",
                freshFrame
        );

        telemetry.addData(
                "Tag visible",
                tagVisible
        );

        telemetry.addData(
                "TX",
                "%.2f",
                tx
        );

        telemetry.addData(
                "Error visual",
                "%.2f",
                errorDeg
        );

        telemetry.addData(
                "Vision Sign",
                "%.0f",
                visionSign
        );

        telemetry.addData(
                "Offset tag",
                "%.2f",
                offset
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
                "Robot heading",
                "%.2f",
                robotHeading
        );

        telemetry.addData(
                "Field lock",
                hasFieldLock
        );

        telemetry.addData(
                "Field bearing",
                "%.2f",
                lockedFieldBearingDeg
        );

        telemetry.addLine(
                "========== TORRETA =========="
        );

        telemetry.addData(
                "Turret actual",
                "%.2f",
                turretAngle
        );

        telemetry.addData(
                "Turret objetivo",
                "%.2f",
                commandedAngleDeg
        );

        telemetry.addData(
                "Turret error",
                "%.2f",
                turret.getErrorDeg()
        );

        telemetry.addData(
                "Motor power",
                "%.3f",
                turret.getMotorPower()
        );

        telemetry.update();
    }

    // ==============================================
    // NORMALIZAR ANGULO
    // ==============================================

    private static double normalize(double angle) {

        while (angle > 180.0) {
            angle -= 360.0;
        }

        while (angle < -180.0) {
            angle += 360.0;
        }

        return angle;
    }

    // ==============================================
    // CLAMP
    // ==============================================

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

    // ==============================================
    // STOP
    // ==============================================

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


package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.TurretFieldTrackingCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;
@Disabled
@TeleOp(name = "ThunderROAR - Pinpoint + Limelight PI")
public class ThunderROAR_Pinpoint_Limelight_Aim extends OpMode {

    // =====================================================
    // SUBSYSTEMS
    // =====================================================

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCommand;

    private TurretSubsystem turret;
    private TurretFieldTrackingCommand tracking;

    private Limelight3A limelight;

    // =====================================================
    // COORDENADAS DEL OBJETIVO
    // =====================================================

    // Valores de las pruebas anteriores.
    // Verificar contra las coordenadas reales del campo.
    private static final double TARGET_X = 57.0;
    private static final double TARGET_Y = 59.0;

    // =====================================================
    // LIMELIGHT
    // =====================================================

    private static final int PIPELINE = 0;

    private static final int TAG_34 = 34;
    private static final int TAG_36 = 36;

    private int selectedTag = TAG_36;

    private static final double MAX_RESULT_AGE_MS = 150.0;

    // =====================================================
    // CONTROLADOR PI VISUAL
    // =====================================================

    // Signo utilizado en las pruebas anteriores.
    private static final double VISION_SIGN = -1.0;

    // Ganancias PI.
    private static final double VISION_KP = 0.35;
    private static final double VISION_KI = 0.45;

    // Zona muerta.
    private static final double VISION_DEADBAND_DEG = 0.8;

    // Limite de correccion visual.
    private static final double MAX_VISION_OFFSET_DEG = 12.0;

    // Velocidad maxima del cambio del offset.
    private static final double MAX_OFFSET_RATE = 8.0;

    // =====================================================
    // OFFSETS DE LOS APRILTAGS
    // =====================================================

    // Pendientes de calibracion al HIGH GOAL.
    private double offset34Deg = 0.0;
    private double offset36Deg = 0.0;

    // =====================================================
    // VARIABLES DEL CONTROLADOR
    // =====================================================

    // Componente integral.
    private double accumulatedVisionCorrection = 0.0;

    // Componente proporcional.
    private double proportionalVisionCorrection = 0.0;

    // Offset calculado por PI.
    private double requestedVisionCorrection = 0.0;

    // Offset aplicado al tracking.
    private double appliedVisionCorrection = 0.0;

    private long previousVisionTime = 0;

    // =====================================================
    // ESTADOS
    // =====================================================

    private boolean visionEnabled = false;

    private boolean tagVisible = false;
    private boolean frameFresh = false;

    private double tx = Double.NaN;
    private double visualErrorDeg = Double.NaN;

    private String visionStatus = "OFF";

    // =====================================================
    // BOTONES
    // =====================================================

    private boolean previousA = false;
    private boolean previousB = false;

    private boolean previousLeft = false;
    private boolean previousRight = false;

    // =====================================================
    // INIT
    // =====================================================

    @Override
    public void init() {

        // CHASIS
        drive = new DriveSubsystem(hardwareMap);

        driveCommand = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );

        // TORRETA
        turret = new TurretSubsystem(hardwareMap);

        tracking = new TurretFieldTrackingCommand(
                drive,
                turret,
                TARGET_X,
                TARGET_Y
        );

        // LIMELIGHT
        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.pipelineSwitch(PIPELINE);

        telemetry.addLine("THUNDERROAR AUTO AIM PI");
        telemetry.addLine("");
        telemetry.addLine("B = PINPOINT ON/OFF");
        telemetry.addLine("A = LIMELIGHT PI ON/OFF");
        telemetry.addLine("DPAD LEFT = TAG 34");
        telemetry.addLine("DPAD RIGHT = TAG 36");
        telemetry.addLine("");
        telemetry.addLine("COLOCAR TORRETA AL FRENTE ANTES DE INIT");
        telemetry.addLine("PRIMERA PRUEBA SIN PELOTAS");
        telemetry.update();
    }

    // =====================================================
    // START
    // =====================================================

    @Override
    public void start() {

        limelight.start();

        previousVisionTime = System.nanoTime();

        resetVisionCorrection();
    }

    // =====================================================
    // LOOP
    // =====================================================

    @Override
    public void loop() {

        // =================================================
        // 1. ACTUALIZAR CHASIS
        // =====================================================

        driveCommand.execute();
        drive.update();

        // =================================================
        // 2. LEER BOTONES
        // =====================================================

        boolean currentA = gamepad1.a;
        boolean currentB = gamepad1.b;

        boolean currentLeft = gamepad1.dpad_left;
        boolean currentRight = gamepad1.dpad_right;

        // ACTIVAR O DESACTIVAR PINPOINT
        if (currentB && !previousB) {

            if (tracking.isEnabled()) {

                tracking.disable();

                visionEnabled = false;

                resetVisionCorrection();

            } else {

                resetVisionCorrection();

                tracking.enable();
            }
        }

        // ACTIVAR O DESACTIVAR PI VISUAL
        if (currentA && !previousA) {

            if (tracking.isEnabled()) {

                visionEnabled = !visionEnabled;

                resetVisionCorrection();
            }
        }

        // SELECCIONAR TAG 34
        if (currentLeft && !previousLeft) {

            selectedTag = TAG_34;

            resetVisionCorrection();
        }

        // SELECCIONAR TAG 36
        if (currentRight && !previousRight) {

            selectedTag = TAG_36;

            resetVisionCorrection();
        }

        previousA = currentA;
        previousB = currentB;

        previousLeft = currentLeft;
        previousRight = currentRight;

        // =====================================================
        // 3. LEER LIMELIGHT
        // =====================================================

        tagVisible = false;
        frameFresh = false;

        tx = Double.NaN;
        visualErrorDeg = Double.NaN;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double ageMs = result.getStaleness();

            frameFresh =
                    ageMs >= 0.0 &&
                            ageMs <= MAX_RESULT_AGE_MS;

            if (frameFresh
                    && result.getFiducialResults() != null) {

                for (LLResultTypes.FiducialResult tag :
                        result.getFiducialResults()) {

                    if (tag.getFiducialId() == selectedTag) {

                        tagVisible = true;

                        tx = tag.getTargetXDegrees();

                        break;
                    }
                }
            }
        }

        // =====================================================
        // 4. CALCULAR ERROR VISUAL
        // =====================================================

        if (tagVisible) {

            double targetOffset =
                    selectedTag == TAG_34
                            ? offset34Deg
                            : offset36Deg;

            visualErrorDeg = tx - targetOffset;
        }

        // =====================================================
        // 5. CALCULAR DT
        // =====================================================

        long now = System.nanoTime();

        double dt =
                previousVisionTime == 0
                        ? 0.02
                        : (now - previousVisionTime)
                          / 1_000_000_000.0;

        previousVisionTime = now;

        dt = clamp(dt, 0.001, 0.05);

        // =====================================================
        // 6. CONTROLADOR PI VISUAL
        // =====================================================

        if (!tracking.isEnabled()) {

            visionStatus = "PINPOINT OFF";

            resetVisionCorrection();

        } else if (!visionEnabled) {

            visionStatus = "VISION OFF";

            resetVisionCorrection();

        } else if (!frameFresh) {

            visionStatus = "FRAME NO RECIENTE";

            // No acumular datos antiguos.
            // Conservar el ultimo offset aplicado.
            proportionalVisionCorrection = 0.0;

            requestedVisionCorrection =
                    appliedVisionCorrection;

        } else if (!tagVisible) {

            visionStatus = "TAG NO VISIBLE";

            // Mantener la ultima correccion
            // mientras Pinpoint sigue funcionando.
            proportionalVisionCorrection = 0.0;

            requestedVisionCorrection =
                    appliedVisionCorrection;

        } else {

            double error = visualErrorDeg;

            // ZONA MUERTA
            if (Math.abs(error) <= VISION_DEADBAND_DEG) {

                proportionalVisionCorrection = 0.0;

                // Mantener el offset que ya centro la torreta.
                requestedVisionCorrection =
                        appliedVisionCorrection;

                visionStatus = "CENTRADO";

            } else {

                // =========================================
                // COMPONENTE P
                // =========================================

                proportionalVisionCorrection =
                        VISION_SIGN
                                * VISION_KP
                                * error;

                // =========================================
                // COMPONENTE I
                // =========================================

                double integralChange =
                        VISION_SIGN
                                * VISION_KI
                                * error
                                * dt;

                double proposedIntegral =
                        accumulatedVisionCorrection
                                + integralChange;

                double proposedOutput =
                        proposedIntegral
                                + proportionalVisionCorrection;

                // =========================================
                // ANTI-WINDUP
                // =========================================

                boolean pushingUpper =
                        proposedOutput > MAX_VISION_OFFSET_DEG
                                && integralChange > 0.0;

                boolean pushingLower =
                        proposedOutput < -MAX_VISION_OFFSET_DEG
                                && integralChange < 0.0;

                if (!pushingUpper && !pushingLower) {

                    accumulatedVisionCorrection = clamp(
                            proposedIntegral,
                            -MAX_VISION_OFFSET_DEG,
                            MAX_VISION_OFFSET_DEG
                    );
                }

                // =========================================
                // SUMAR P + I
                // =========================================

                requestedVisionCorrection =
                        accumulatedVisionCorrection
                                + proportionalVisionCorrection;

                requestedVisionCorrection = clamp(
                        requestedVisionCorrection,
                        -MAX_VISION_OFFSET_DEG,
                        MAX_VISION_OFFSET_DEG
                );

                if (Math.abs(requestedVisionCorrection)
                        >= MAX_VISION_OFFSET_DEG - 0.01) {

                    visionStatus = "OFFSET LIMITADO";

                } else {

                    visionStatus = "PI CORRIGIENDO";
                }
            }
        }

        // =====================================================
        // 7. LIMITAR VELOCIDAD DEL OFFSET
        // =====================================================

        if (visionEnabled && tracking.isEnabled()) {

            double maxChange =
                    MAX_OFFSET_RATE * dt;

            double correctionDifference =
                    requestedVisionCorrection
                            - appliedVisionCorrection;

            double limitedChange = clamp(
                    correctionDifference,
                    -maxChange,
                    maxChange
            );

            // =============================================
            // CAMBIO PRINCIPAL DE ESTA VERSION
            // =============================================
            //
            // ANTES:
            //
            // appliedVisionCorrection +=
            //         limitedChange * 0.20;
            //
            // AHORA:
            //
            // Se aplica todo el cambio permitido.
            // El limite de 8 deg/s sigue activo.
            // =============================================

            appliedVisionCorrection += limitedChange;

            appliedVisionCorrection = clamp(
                    appliedVisionCorrection,
                    -MAX_VISION_OFFSET_DEG,
                    MAX_VISION_OFFSET_DEG
            );

        } else {

            appliedVisionCorrection = 0.0;
        }

        // =====================================================
        // 8. ENVIAR OFFSET AL TRACKING
        // =====================================================

//        tracking.setVisionCorrectionDeg(
//                appliedVisionCorrection
//        );

        // =====================================================
        // 9. EJECUTAR TRACKING
        // =====================================================

        if (tracking.isEnabled()) {

            tracking.execute();

        } else {

            turret.stop();
        }

        // =====================================================
        // 10. TELEMETRIA
        // =====================================================

        telemetry.addLine("========== AUTO AIM PI ==========");

        telemetry.addData(
                "Pinpoint",
                tracking.isEnabled() ? "ON" : "OFF"
        );

        telemetry.addData(
                "Limelight PI",
                visionEnabled ? "ON" : "OFF"
        );

        telemetry.addData(
                "Tag",
                selectedTag
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
                "TX",
                "%.2f deg",
                tx
        );

        telemetry.addData(
                "Error visual",
                "%.2f deg",
                visualErrorDeg
        );

        telemetry.addLine("========== CONTROL PI ==========");

        telemetry.addData(
                "P",
                "%.2f deg",
                proportionalVisionCorrection
        );

        telemetry.addData(
                "I",
                "%.2f deg",
                accumulatedVisionCorrection
        );

        telemetry.addData(
                "Offset solicitado",
                "%.2f deg",
                requestedVisionCorrection
        );

        telemetry.addData(
                "Offset aplicado",
                "%.2f deg",
                appliedVisionCorrection
        );

        telemetry.addData(
                "Vision Status",
                visionStatus
        );

        telemetry.addLine("========== TORRETA ==========");

        telemetry.addData(
                "Actual",
                "%.2f deg",
                turret.getCurrentAngleDeg()
        );

        telemetry.addData(
                "Objetivo",
                "%.2f deg",
                turret.getTargetAngleDeg()
        );

        telemetry.addData(
                "Error",
                "%.2f deg",
                turret.getErrorDeg()
        );

        telemetry.addData(
                "Motor Power",
                "%.3f",
                turret.getMotorPower()
        );

        telemetry.addLine("========== PINPOINT ==========");

        telemetry.addData(
                "X",
                "%.2f in",
                drive.getX()
        );

        telemetry.addData(
                "Y",
                "%.2f in",
                drive.getY()
        );

        telemetry.addData(
                "Heading",
                "%.2f deg",
                drive.getHeadingDegrees()
        );

        telemetry.update();
    }

    // =====================================================
    // REINICIAR CONTROLADOR PI
    // =====================================================

    private void resetVisionCorrection() {

        accumulatedVisionCorrection = 0.0;

        proportionalVisionCorrection = 0.0;

        requestedVisionCorrection = 0.0;

        appliedVisionCorrection = 0.0;

        previousVisionTime = System.nanoTime();

        if (tracking != null) {

//            tracking.setVisionCorrectionDeg(0.0);
        }
    }

    // =====================================================
    // LIMITADOR
    // =====================================================

    private double clamp(
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

        visionEnabled = false;

        resetVisionCorrection();

        if (limelight != null) {

            limelight.stop();
        }

        if (tracking != null) {

            tracking.disable();
        }

        if (turret != null) {

            turret.stop();
        }

        if (driveCommand != null) {

            driveCommand.end();
        }
    }
}

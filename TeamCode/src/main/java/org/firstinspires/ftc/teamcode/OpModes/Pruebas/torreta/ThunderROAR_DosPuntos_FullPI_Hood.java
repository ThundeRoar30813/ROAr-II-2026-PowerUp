
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.TurretFieldTrackingCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

@Disabled
@TeleOp(name = "ThunderROAR - DOS PUNTOS FULL PI")
public class ThunderROAR_DosPuntos_FullPI_Hood extends OpMode {

    // =====================================================
    // COORDENADAS DE LOS DOS OBJETIVOS
    // =====================================================
    // TEMPORALES PARA PROBAR QUE B ACTIVE EL TRACKING.
    // NO SON LAS DOS ESQUINAS REALES DEL HIGH GOAL.

    private static final double GOAL_34_X = 57.0;
    private static final double GOAL_34_Y = 59.0;

    private static final double GOAL_36_X = 57.0;
    private static final double GOAL_36_Y = 59.0;

    // IDs de tu configuracion anterior.
    // Verifica que sean los que detecta tu Limelight.
    private static final int TAG_34 = 34;
    private static final int TAG_36 = 36;

    private static final int PIPELINE = 0;

    private int selectedTag = TAG_36;

    // =====================================================
    // SUBSYSTEMS
    // =====================================================

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCommand;

    private TurretSubsystem turret;
    private TurretFieldTrackingCommand tracking;

    private Limelight3A limelight;

    // =====================================================
    // SHOOTER
    // =====================================================

    private DcMotorEx shooter;

    // Ticks por segundo, NO RPM.
    private double targetVelocity = 800.0;

    private double p = 4.0;
    private double i = 0.0;
    private double d = 0.5;
    private double f = 12.0;

    private boolean shooterEnabled = false;

    // =====================================================
    // INTAKE / INDEXER
    // =====================================================

    private DcMotor intake;
    private DcMotor indexer;

    private boolean intakeEnabled = false;

    private static final double INTAKE_POWER = 1.0;
    private static final double INDEXER_POWER = 1.0;

    // =====================================================
    // GATE
    // =====================================================

    private Servo gate;

    private static final double GATE_CLOSED_DEG = 90.0;
    private static final double GATE_OPEN_DEG = 20.0;

    private boolean gateWasOpened = false;
    private double currentGateAngle = Double.NaN;

    // =====================================================
    // CAPUCHA
    // =====================================================

    private Servo hood;

    private static final double[] HOOD_POSITIONS = {
            0.4444,
            0.5560,
            0.6667,
            0.7778,
            0.8890
    };

    private static final String[] HOOD_NAMES = {
            "ABAJO",
            "BAJA",
            "MEDIA",
            "ALTA",
            "ARRIBA"
    };

    private int hoodIndex = 2;

    // =====================================================
    // LIMELIGHT PI
    // =====================================================

    private static final double VISION_SIGN = -1.0;

    private static final double VISION_KP = 0.35;
    private static final double VISION_KI = 0.45;

    private static final double VISION_DEADBAND_DEG = 0.8;

    private static final double MAX_VISION_OFFSET_DEG = 12.0;
    private static final double MAX_OFFSET_RATE = 8.0;

    private static final double MAX_RESULT_AGE_MS = 150.0;

    private double offset34Deg = 0.0;
    private double offset36Deg = 0.0;

    private double proportionalVisionCorrection = 0.0;
    private double accumulatedVisionCorrection = 0.0;

    private double requestedVisionCorrection = 0.0;
    private double appliedVisionCorrection = 0.0;

    private long previousVisionTime = 0;

    private boolean visionEnabled = false;
    private boolean tagVisible = false;
    private boolean frameFresh = false;

    private double tx = Double.NaN;
    private double visualErrorDeg = Double.NaN;

    private String visionStatus = "OFF";

    // =====================================================
    // BOTONES
    // =====================================================

    private boolean previousA1 = false;
    private boolean previousB1 = false;
    private boolean previousY1 = false;

    private boolean previousLB1 = false;
    private boolean previousRB1 = false;

    private boolean previousHoodUp = false;
    private boolean previousHoodDown = false;

    private boolean previousA2 = false;

    private boolean previousP = false;
    private boolean previousI = false;
    private boolean previousD = false;
    private boolean previousF = false;

    // =====================================================
    // COORDENADAS DEL OBJETIVO SELECCIONADO
    // =====================================================

    private double getGoalX() {

        return selectedTag == TAG_34
                ? GOAL_34_X
                : GOAL_36_X;
    }

    private double getGoalY() {

        return selectedTag == TAG_34
                ? GOAL_34_Y
                : GOAL_36_Y;
    }

    private boolean goalConfigured() {

        return Double.isFinite(getGoalX())
                && Double.isFinite(getGoalY());
    }

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
                getGoalX(),
                getGoalY()
        );

        // LIMELIGHT
        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.pipelineSwitch(PIPELINE);

        // SHOOTER
        shooter = hardwareMap.get(
                DcMotorEx.class,
                "shoot"
        );

        shooter.setDirection(DcMotor.Direction.FORWARD);

        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        shooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        shooter.setPower(0);

        applyPIDF();

        // INTAKE
        intake = hardwareMap.get(
                DcMotor.class,
                "intake2"
        );

        intake.setDirection(DcMotor.Direction.FORWARD);

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        intake.setPower(0);

        // INDEXER
        indexer = hardwareMap.get(
                DcMotor.class,
                "intake1"
        );

        indexer.setDirection(DcMotor.Direction.REVERSE);

        indexer.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        indexer.setPower(0);

        // GATE
        gate = hardwareMap.get(
                Servo.class,
                "gate"
        );

        gate.setDirection(Servo.Direction.REVERSE);

        // CAPUCHA
        hood = hardwareMap.get(
                Servo.class,
                "hood"
        );

        hood.setDirection(Servo.Direction.FORWARD);

        // No mover los servos durante INIT.

        telemetry.addLine("THUNDERROAR FULL READY");

        telemetry.addLine("GAMEPAD 1:");
        telemetry.addLine("A = SHOOTER");
        telemetry.addLine("B = TRACKING");
        telemetry.addLine("Y = INTAKE");
        telemetry.addLine("X = ALIMENTAR");
        telemetry.addLine("LB = TAG 34");
        telemetry.addLine("RB = TAG 36");
        telemetry.addLine("DPAD UP/DOWN = CAPUCHA");

        telemetry.addLine("GAMEPAD 2:");
        telemetry.addLine("A = LIMELIGHT PI");
        telemetry.addLine("DPAD = AJUSTAR PIDF");

        telemetry.addData(
                "Goal configurado",
                goalConfigured()
        );

        telemetry.update();
    }

    // =====================================================
    // START
    // =====================================================

    @Override
    public void start() {

        limelight.start();

        resetVisionCorrection();
    }

    // =====================================================
    // LOOP
    // =====================================================

    @Override
    public void loop() {

        // =================================================
        // 1. CHASIS
        // =================================================

        driveCommand.execute();
        drive.update();

        // =================================================
        // 2. CAMBIAR ENTRE DOS OBJETIVOS
        // =================================================

        if (gamepad1.left_bumper && !previousLB1) {

            selectGoal(TAG_34);
        }

        if (gamepad1.right_bumper && !previousRB1) {

            selectGoal(TAG_36);
        }

        previousLB1 = gamepad1.left_bumper;
        previousRB1 = gamepad1.right_bumper;

        // =================================================
        // 3. ACTIVAR TRACKING CON B
        // =================================================

        if (gamepad1.b && !previousB1) {

            if (tracking.isEnabled()) {

                tracking.disable();

                visionEnabled = false;

                resetVisionCorrection();

            } else {

                if (goalConfigured()) {

                    tracking.setTarget(
                            getGoalX(),
                            getGoalY()
                    );

                    resetVisionCorrection();

                    tracking.enable();
                }
            }
        }

        previousB1 = gamepad1.b;

        // =================================================
        // 4. ACTIVAR LIMELIGHT CON GAMEPAD 2 A
        // =================================================

        if (gamepad2.a && !previousA2) {

            if (tracking.isEnabled()) {

                visionEnabled = !visionEnabled;

                resetVisionCorrection();
            }
        }

        previousA2 = gamepad2.a;

        // =================================================
        // 5. SHOOTER
        // =================================================

        if (gamepad1.a && !previousA1) {

            shooterEnabled = !shooterEnabled;
        }

        previousA1 = gamepad1.a;

        // Velocidad del shooter.
        targetVelocity += gamepad1.right_trigger * 5.0;
        targetVelocity -= gamepad1.left_trigger * 5.0;

        targetVelocity = clamp(
                targetVelocity,
                0.0,
                6000.0
        );

        if (shooterEnabled) {

            shooter.setVelocity(targetVelocity);

        } else {

            shooter.setVelocity(0.0);
        }

        // =================================================
        // 6. PIDF SHOOTER
        // =================================================

        if (gamepad2.dpad_up && !previousP) {

            p += 0.2;
            applyPIDF();
        }

        if (gamepad2.dpad_left && !previousI) {

            i += 0.01;
            applyPIDF();
        }

        if (gamepad2.dpad_right && !previousD) {

            d += 0.1;
            applyPIDF();
        }

        if (gamepad2.dpad_down && !previousF) {

            f += 0.5;
            applyPIDF();
        }

        previousP = gamepad2.dpad_up;
        previousI = gamepad2.dpad_left;
        previousD = gamepad2.dpad_right;
        previousF = gamepad2.dpad_down;

        // =================================================
        // 7. CAPUCHA
        // =================================================

        if (gamepad1.dpad_up && !previousHoodUp) {

            setHoodIndex(hoodIndex + 1);
        }

        if (gamepad1.dpad_down && !previousHoodDown) {

            setHoodIndex(hoodIndex - 1);
        }

        previousHoodUp = gamepad1.dpad_up;
        previousHoodDown = gamepad1.dpad_down;

        // =================================================
        // 8. INTAKE / INDEXER / GATE
        // =================================================

        if (gamepad1.y && !previousY1) {

            intakeEnabled = !intakeEnabled;
        }

        previousY1 = gamepad1.y;

        boolean feeding = gamepad1.x;

        if (feeding) {

            setGateAngle(GATE_OPEN_DEG);

            gateWasOpened = true;

            intake.setPower(INTAKE_POWER);
            indexer.setPower(INDEXER_POWER);

        } else if (intakeEnabled) {

            setGateAngle(GATE_CLOSED_DEG);

            gateWasOpened = false;

            intake.setPower(INTAKE_POWER);
            indexer.setPower(INDEXER_POWER);

        } else {

            intake.setPower(0);
            indexer.setPower(0);

            if (gateWasOpened) {

                setGateAngle(GATE_CLOSED_DEG);

                gateWasOpened = false;
            }
        }

        // =================================================
        // 9. LEER LIMELIGHT
        // =================================================

        tagVisible = false;
        frameFresh = false;

        tx = Double.NaN;
        visualErrorDeg = Double.NaN;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double ageMs = result.getStaleness();

            frameFresh =
                    ageMs >= 0
                            && ageMs <= MAX_RESULT_AGE_MS;

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

        if (tagVisible) {

            double targetOffset =
                    selectedTag == TAG_34
                            ? offset34Deg
                            : offset36Deg;

            visualErrorDeg = tx - targetOffset;
        }

        // =================================================
        // 10. CALCULAR DT
        // =================================================

        long now = System.nanoTime();

        double dt =
                previousVisionTime == 0
                        ? 0.02
                        : (now - previousVisionTime)
                          / 1_000_000_000.0;

        previousVisionTime = now;

        dt = clamp(dt, 0.001, 0.05);

        // =================================================
        // 11. CONTROLADOR PI VISUAL
        // =================================================

        if (!tracking.isEnabled()) {

            visionStatus = "TRACKING OFF";

            resetVisionCorrection();

        } else if (!visionEnabled) {

            visionStatus = "VISION OFF";

            resetVisionCorrection();

        } else if (!frameFresh) {

            visionStatus = "FRAME NO RECIENTE";

            proportionalVisionCorrection = 0;

            requestedVisionCorrection =
                    appliedVisionCorrection;

        } else if (!tagVisible) {

            visionStatus = "TAG NO VISIBLE";

            proportionalVisionCorrection = 0;

            requestedVisionCorrection =
                    appliedVisionCorrection;

        } else {

            double error = visualErrorDeg;

            if (Math.abs(error)
                    <= VISION_DEADBAND_DEG) {

                proportionalVisionCorrection = 0;

                requestedVisionCorrection =
                        appliedVisionCorrection;

                visionStatus = "CENTRADO";

            } else {

                // COMPONENTE P
                proportionalVisionCorrection =
                        VISION_SIGN
                                * VISION_KP
                                * error;

                // COMPONENTE I
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

                // ANTI-WINDUP
                boolean pushingUpper =
                        proposedOutput > MAX_VISION_OFFSET_DEG
                                && integralChange > 0;

                boolean pushingLower =
                        proposedOutput < -MAX_VISION_OFFSET_DEG
                                && integralChange < 0;

                if (!pushingUpper && !pushingLower) {

                    accumulatedVisionCorrection = clamp(
                            proposedIntegral,
                            -MAX_VISION_OFFSET_DEG,
                            MAX_VISION_OFFSET_DEG
                    );
                }

                requestedVisionCorrection = clamp(
                        accumulatedVisionCorrection
                                + proportionalVisionCorrection,
                        -MAX_VISION_OFFSET_DEG,
                        MAX_VISION_OFFSET_DEG
                );

                visionStatus = "PI CORRIGIENDO";
            }
        }

        // =================================================
        // 12. LIMITAR VELOCIDAD DE CORRECCION
        // =================================================

        if (visionEnabled && tracking.isEnabled()) {

            double maxChange = MAX_OFFSET_RATE * dt;

            double difference =
                    requestedVisionCorrection
                            - appliedVisionCorrection;

            double limitedChange = clamp(
                    difference,
                    -maxChange,
                    maxChange
            );

            appliedVisionCorrection += limitedChange;

            appliedVisionCorrection = clamp(
                    appliedVisionCorrection,
                    -MAX_VISION_OFFSET_DEG,
                    MAX_VISION_OFFSET_DEG
            );

        } else {

            appliedVisionCorrection = 0;
        }

        // =================================================
        // 13. EJECUTAR TRACKING
        // =================================================

//        //tracking.setVisionCorrectionDeg(
//                appliedVisionCorrection
//        );

        if (tracking.isEnabled()) {

            tracking.execute();

        } else {

            turret.stop();
        }

        // =================================================
        // 14. TELEMETRIA
        // =================================================

        telemetry.addLine(
                "========== THUNDERROAR =========="
        );

        telemetry.addData(
                "Boton B",
                gamepad1.b ? "PRESIONADO" : "LIBRE"
        );

        telemetry.addData(
                "Goal configurado",
                goalConfigured()
        );

        telemetry.addData(
                "Tracking",
                tracking.isEnabled() ? "ON" : "OFF"
        );

        telemetry.addData(
                "Objetivo seleccionado",
                selectedTag
        );

        telemetry.addData(
                "Goal X",
                getGoalX()
        );

        telemetry.addData(
                "Goal Y",
                getGoalY()
        );

        telemetry.addData(
                "Vision PI",
                visionEnabled ? "ON" : "OFF"
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
                visualErrorDeg
        );

        telemetry.addData(
                "Vision P",
                "%.2f",
                proportionalVisionCorrection
        );

        telemetry.addData(
                "Vision I",
                "%.2f",
                accumulatedVisionCorrection
        );

        telemetry.addData(
                "Offset aplicado",
                "%.2f",
                appliedVisionCorrection
        );

        telemetry.addData(
                "Vision Status",
                visionStatus
        );

        telemetry.addLine(
                "========== TORRETA =========="
        );

        telemetry.addData(
                "Actual",
                "%.2f",
                turret.getCurrentAngleDeg()
        );

        telemetry.addData(
                "Objetivo",
                "%.2f",
                turret.getTargetAngleDeg()
        );

        telemetry.addData(
                "Error",
                "%.2f",
                turret.getErrorDeg()
        );

        telemetry.addData(
                "Power",
                "%.3f",
                turret.getMotorPower()
        );

        telemetry.addLine(
                "========== SHOOTER =========="
        );

        telemetry.addData(
                "Shooter",
                shooterEnabled ? "ON" : "OFF"
        );

        telemetry.addData(
                "Target ticks/s",
                "%.1f",
                targetVelocity
        );

        telemetry.addData(
                "Actual ticks/s",
                "%.1f",
                shooter.getVelocity()
        );

        telemetry.addData(
                "PIDF P/I/D/F",
                "%.2f / %.3f / %.2f / %.2f",
                p, i, d, f
        );

        telemetry.addLine(
                "========== MECANISMOS =========="
        );

        telemetry.addData(
                "Intake",
                intakeEnabled ? "ON" : "OFF"
        );

        telemetry.addData(
                "Feeding",
                feeding ? "ON" : "OFF"
        );

        telemetry.addData(
                "Intake Power",
                "%.2f",
                intake.getPower()
        );

        telemetry.addData(
                "Indexer Power",
                "%.2f",
                indexer.getPower()
        );

        telemetry.addData(
                "Gate",
                "%.1f",
                currentGateAngle
        );

        telemetry.addData(
                "Hood",
                "%d - %s",
                hoodIndex + 1,
                HOOD_NAMES[hoodIndex]
        );

        telemetry.addData(
                "Hood servo",
                "%.4f",
                HOOD_POSITIONS[hoodIndex]
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
                drive.getHeadingDegrees()
        );

        telemetry.update();
    }

    // =====================================================
    // CAMBIAR OBJETIVO SIN REINICIAR EL CERO
    // =====================================================

    private void selectGoal(int tagId) {

        if (selectedTag == tagId) {
            return;
        }

        selectedTag = tagId;

        resetVisionCorrection();

        tracking.setTarget(
                getGoalX(),
                getGoalY()
        );
    }

    // =====================================================
    // CAPUCHA
    // =====================================================

    private void setHoodIndex(int index) {

        hoodIndex = (int) clamp(
                index,
                0,
                HOOD_POSITIONS.length - 1
        );

        hood.setPosition(
                HOOD_POSITIONS[hoodIndex]
        );
    }

    // =====================================================
    // GATE
    // =====================================================

    private void setGateAngle(double degrees) {

        degrees = clamp(
                degrees,
                0.0,
                180.0
        );

        gate.setPosition(
                degrees / 180.0
        );

        currentGateAngle = degrees;
    }

    // =====================================================
    // PIDF
    // =====================================================

    private void applyPIDF() {

        shooter.setVelocityPIDFCoefficients(
                p,
                i,
                d,
                f
        );
    }

    // =====================================================
    // RESET VISION
    // =====================================================

    private void resetVisionCorrection() {

        proportionalVisionCorrection = 0;
        accumulatedVisionCorrection = 0;

        requestedVisionCorrection = 0;
        appliedVisionCorrection = 0;

        previousVisionTime = System.nanoTime();

//        if (tracking != null) {
//
//            tracking.setVisionCorrectionDeg(0);
//        }
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

        visionEnabled = false;

        resetVisionCorrection();

        if (limelight != null) {

            limelight.stop();
        }

        if (shooter != null) {

            shooter.setVelocity(0);
            shooter.setPower(0);
        }

        if (intake != null) {

            intake.setPower(0);
        }

        if (indexer != null) {

            indexer.setPower(0);
        }

        if (tracking != null) {

            tracking.disable();
        }

        if (turret != null) {

            turret.stop();
        }

        if (driveCommand != null) {

            driveCommand.end();

        } else if (drive != null) {

            drive.stop();
        }
    }
}

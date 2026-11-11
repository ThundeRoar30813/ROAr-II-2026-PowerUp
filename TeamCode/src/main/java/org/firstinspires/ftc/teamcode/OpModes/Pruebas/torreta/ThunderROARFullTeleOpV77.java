
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV77;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV5;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

@TeleOp(name = "ThunderROAR FULL V77")
public class ThunderROARFullTeleOpV77 extends OpMode {

    // ==========================================
    // POSICION INICIAL PEDRO
    // ==========================================

    public static double START_X = 72;
    public static double START_Y = 72;
    public static double START_HEADING = 0;

    // ==========================================
    // SUBSYSTEMS
    // ==========================================

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCmd;

    private TurretSubsystemV4 turret;
    private TurretVisionV5 vision;
    private TurretAimControllerV77 aim;

    // ==========================================
    // SHOOTER
    // ==========================================

    private DcMotorEx shooter;

    private boolean shooterOn = false;

    private double targetTicks = 1000;

    private double p = 30;
    private double i = 0;
    private double d = 0.5;
    private double f = 16;

    private long lastSpeedNs = 0;

    // ==========================================
    // INTAKE
    // ==========================================

    private DcMotor intake;
    private DcMotor indexer;

    private boolean autoIntake = false;

    private double intakePower = 0;
    private double indexerPower = 0;

    // ==========================================
    // GATE / HOOD
    // ==========================================

    private Servo gate;
    private Servo hood;

    private final double[] hoodPos = {
            0.4444,
            0.5560,
            0.6667,
            0.7778,
            0.8890
    };

    private int hoodIndex = 2;

    // ==========================================
    // SENSORES
    // ==========================================

    private ColorSensor bottom;
    private ColorSensor top;

    private int ballCount = 0;

    private boolean previousBottom = false;
    private boolean previousTop = false;

    private boolean bottomDetected = false;
    private boolean topDetected = false;

    private int bottomAlpha = 0;
    private int topAlpha = 0;

    private long lastBottomNs = 0;
    private long lastTopNs = 0;

    // ==========================================
    // TURRET STATE
    // ==========================================

    private boolean tracking = false;
    private boolean home = false;

    private int tagId = 37;

    private String turretStatus = "READY";

    // ==========================================
    // BUTTON MEMORY
    // ==========================================

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastLB = false;

    private boolean lastUp = false;
    private boolean lastDown = false;

    private boolean lastModeA = false;
    private boolean lastModeLB = false;
    private boolean lastModeRB = false;

    private boolean lastTag = false;
    private boolean lastBack = false;
    private boolean lastStart = false;

    private boolean lastP = false;
    private boolean lastF = false;
    private boolean lastD = false;
    private boolean lastI = false;

    // ==========================================
    // INIT
    // ==========================================

    @Override
    public void init() {

        // DRIVE
        drive = new DriveSubsystem(hardwareMap);

        drive.setInitialPose(
                START_X,
                START_Y,
                START_HEADING
        );

        driveCmd = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );

        // TURRET
        turret = new TurretSubsystemV4(hardwareMap);

        // Potencia reducida para las
        // primeras pruebas.
        turret.maxPower = 0.35;

        // LIMELIGHT
        vision = new TurretVisionV5(hardwareMap);
        vision.setSelectedTag(tagId);

        // AIM CONTROLLER
        aim = new TurretAimControllerV77(
                drive,
                turret,
                vision
        );

        aim.setMode(
                TurretAimControllerV77.Mode.COMBINED
        );

        // SHOOTER
        shooter = hardwareMap.get(
                DcMotorEx.class,
                "shoot"
        );

        shooter.setDirection(
                DcMotor.Direction.FORWARD
        );

        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        shooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        shooter.setPower(0);

        applyShooterPIDF();

        // INTAKE
        intake = hardwareMap.get(
                DcMotor.class,
                "intake2"
        );

        indexer = hardwareMap.get(
                DcMotor.class,
                "intake1"
        );

        intake.setDirection(
                DcMotor.Direction.FORWARD
        );

        indexer.setDirection(
                DcMotor.Direction.REVERSE
        );

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        indexer.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        intake.setPower(0);
        indexer.setPower(0);

        // GATE
        gate = hardwareMap.tryGet(
                Servo.class,
                "gate"
        );

        if (gate != null) {

            gate.setDirection(
                    Servo.Direction.REVERSE
            );
        }

        setGate(false);

        // HOOD
        hood = hardwareMap.tryGet(
                Servo.class,
                "hood"
        );

        if (hood != null) {

            hood.setDirection(
                    Servo.Direction.FORWARD
            );

            hood.setPosition(
                    hoodPos[hoodIndex]
            );
        }

        // COLOR SENSORS
        bottom = hardwareMap.tryGet(
                ColorSensor.class,
                "colorBottom"
        );

        top = hardwareMap.tryGet(
                ColorSensor.class,
                "colorTop"
        );

        telemetry.addLine("THUNDERROAR FULL V77 READY");
        telemetry.addLine("TURRET -80 TO +230");
        telemetry.addLine("NO AUTOMATIC RETURN TO ZERO");

        telemetry.addData(
                "Shooter Start",
                "%.0f ticks/s",
                targetTicks
        );

        telemetry.addData(
                "PIDF",
                "P %.1f I %.1f D %.1f F %.1f",
                p, i, d, f
        );

        telemetry.update();
    }

    // ==========================================
    // START
    // ==========================================

    @Override
    public void start() {
        vision.start();
    }

    // ==========================================
    // LOOP
    // ==========================================

    @Override
    public void loop() {

        long now = System.nanoTime();

        // ======================================
        // 1. DRIVE
        // ======================================

        driveCmd.execute();
        drive.update();

        // ======================================
        // 2. VISION
        // ======================================

        vision.update();

        // ======================================
        // 3. SHOOTER - G1 A
        // ======================================

        if (gamepad1.a && !lastA) {
            shooterOn = !shooterOn;
        }

        lastA = gamepad1.a;

        // ======================================
        // 4. TRACKING - G1 B
        // ======================================

        if (gamepad1.b && !lastB) {

            if (tracking || home) {

                aim.disable();

                tracking = false;
                home = false;

                turretStatus = "TRACKING_OFF";

            } else if (!turret.isFault()) {

                aim.enable();

                tracking = aim.isEnabled();

                turretStatus = tracking
                        ? "TRACKING_ON"
                        : "TRACKING_BLOCKED";
            }
        }

        lastB = gamepad1.b;

        // ======================================
        // 5. AUTO INTAKE - G1 LB
        // ======================================

        if (gamepad1.left_bumper && !lastLB) {
            autoIntake = !autoIntake;
        }

        lastLB = gamepad1.left_bumper;

        // ======================================
        // 6. HOOD - G1 DPAD
        // ======================================

        if (gamepad1.dpad_up && !lastUp) {
            setHood(hoodIndex + 1);
        }

        if (gamepad1.dpad_down && !lastDown) {
            setHood(hoodIndex - 1);
        }

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        // ======================================
        // 7. TRACKING MODES - GAMEPAD 2
        // ======================================

        if (gamepad2.a && !lastModeA) {

            aim.setMode(
                    TurretAimControllerV77.Mode.COMBINED
            );
        }

        if (gamepad2.left_bumper && !lastModeLB) {

            aim.setMode(
                    TurretAimControllerV77.Mode.VISION_ONLY
            );
        }

        if (gamepad2.right_bumper && !lastModeRB) {

            aim.setMode(
                    TurretAimControllerV77.Mode.PEDRO_ONLY
            );
        }

        lastModeA = gamepad2.a;
        lastModeLB = gamepad2.left_bumper;
        lastModeRB = gamepad2.right_bumper;

        // ======================================
        // 8. CHANGE TAG - G2 R3
        // ======================================

        if (gamepad2.right_stick_button && !lastTag) {

            tagId = tagId == 34 ? 37 : 34;

            vision.setSelectedTag(tagId);
        }

        lastTag = gamepad2.right_stick_button;

        // ======================================
        // 9. RESET FAULT - G2 START
        // ======================================

        if (gamepad2.start && !lastStart) {

            aim.disable();

            tracking = false;
            home = false;

            turret.clearFault();

            turretStatus = "FAULT_RESET";
        }

        lastStart = gamepad2.start;

        // ======================================
        // 10. MANUAL HOME - G2 BACK
        // ======================================

        if (gamepad2.back && !lastBack) {

            aim.disable();

            tracking = false;

            if (!turret.isFault()) {

                turret.enableHold();
                turret.setTarget(0);

                home = turret.isEnabled();

                turretStatus = "MANUAL_HOME";

            } else {

                home = false;
                turretStatus = "HOME_BLOCKED_FAULT";
            }
        }

        lastBack = gamepad2.back;

        // ======================================
        // 11. SHOOTER VELOCITY
        // ======================================

        if (now - lastSpeedNs > 200_000_000L) {

            if (gamepad1.right_trigger > 0.5) {

                targetTicks += 5;
                lastSpeedNs = now;

            } else if (gamepad1.left_trigger > 0.5) {

                targetTicks -= 5;
                lastSpeedNs = now;
            }
        }

        targetTicks = Range.clip(
                targetTicks,
                0,
                3000
        );

        // ======================================
        // 12. SHOOTER PIDF
        // ======================================

        boolean pidChanged = false;

        if (gamepad2.dpad_up && !lastP) {

            p += 0.2;
            pidChanged = true;
        }

        if (gamepad2.dpad_down && !lastF) {

            f += 0.5;
            pidChanged = true;
        }

        if (gamepad2.dpad_right && !lastD) {

            d += 0.1;
            pidChanged = true;
        }

        if (gamepad2.dpad_left && !lastI) {

            i += 0.01;
            pidChanged = true;
        }

        lastP = gamepad2.dpad_up;
        lastF = gamepad2.dpad_down;
        lastD = gamepad2.dpad_right;
        lastI = gamepad2.dpad_left;

        if (pidChanged) {
            applyShooterPIDF();
        }

        // ======================================
        // 13. SHOOTER MOTOR
        // ======================================

        if (shooterOn) {

            shooter.setVelocity(targetTicks);

        } else {

            shooter.setPower(0);
        }

        // ======================================
        // 14. SENSORS
        // ======================================

        updateSensors(now);

        // ======================================
        // 15. INTAKE - GAMEPAD 1
        // ======================================

        boolean feeding = gamepad1.x;
        boolean forward = gamepad1.y;
        boolean reverse = gamepad1.right_bumper;

        intakePower = 0;
        indexerPower = 0;

        if (feeding) {

            setGate(true);

            intakePower = 1;
            indexerPower = 1;

        } else if (reverse) {

            setGate(false);

            intakePower = -1;
            indexerPower = -1;

        } else if (forward) {

            setGate(false);

            intakePower = 1;
            indexerPower = 1;

        } else if (autoIntake) {

            setGate(false);

            if (bottom != null
                    && top != null
                    && ballCount < 4) {

                intakePower = 1;

                indexerPower = topDetected
                        ? 0
                        : 1;
            }

        } else {

            setGate(false);
        }

        intake.setPower(intakePower);
        indexer.setPower(indexerPower);

        // ======================================
        // 16. TURRET V77
        // ======================================

        if (tracking) {

            // El tracking permanece activo
            // aunque llegue al limite.
            aim.update();

            if (!aim.isEnabled()
                    || turret.isFault()) {

                tracking = false;

                turretStatus = turret.isFault()
                        ? turret.getFaultReason()
                        : aim.getStatus();

            } else {

                turretStatus = aim.getStatus();
            }

        } else if (home) {

            turret.setTarget(0);
            turret.update();

            if (turret.isFault()
                    || !turret.isEnabled()) {

                home = false;
                turretStatus = "HOME_FAULT";

            } else if (
                    Math.abs(turret.getError()) < 1.5
                            && Math.abs(turret.getVelocity()) < 8) {

                turret.disable();

                home = false;
                turretStatus = "HOME_REACHED";
            }

        } else {

            turret.update();
        }

        // ======================================
        // 17. TELEMETRY
        // ======================================

        telemetry.addLine(
                "===== THUNDERROAR V77 ====="
        );

        telemetry.addData(
                "Robot X", "%.2f",
                drive.getX()
        );

        telemetry.addData(
                "Robot Y", "%.2f",
                drive.getY()
        );

        telemetry.addData(
                "Heading", "%.2f",
                drive.getHeadingDegrees()
        );

        telemetry.addLine("===== TURRET =====");

        telemetry.addData(
                "Tracking", tracking
        );

        telemetry.addData(
                "Mode", aim.getMode()
        );

        telemetry.addData(
                "Status", turretStatus
        );

        telemetry.addData(
                "Selected Tag", tagId
        );

        telemetry.addData(
                "Tag Visible", vision.hasTarget()
        );

        telemetry.addData(
                "Tx", vision.getTx()
        );

        telemetry.addData(
                "Filtered Tx", aim.getFilteredTx()
        );

        telemetry.addData(
                "Actual Angle", "%.2f",
                turret.getAngle()
        );

        telemetry.addData(
                "Physical Target", "%.2f",
                turret.getTarget()
        );

        telemetry.addData(
                "Virtual Target", "%.2f",
                aim.getBaseTarget()
        );

        telemetry.addData(
                "Turret Error", "%.2f",
                turret.getError()
        );

        telemetry.addData(
                "Turret Velocity", "%.2f",
                turret.getVelocity()
        );

        telemetry.addData(
                "Turret Power", "%.2f",
                turret.getPower()
        );

        telemetry.addData(
                "Fault",
                turret.getFaultReason()
        );

        telemetry.addData(
                "Limits",
                "-80 / +230"
        );

        telemetry.addLine("===== SHOOTER =====");

        telemetry.addData(
                "Shooter ON", shooterOn
        );

        telemetry.addData(
                "Target ticks/s", "%.1f",
                targetTicks
        );

        telemetry.addData(
                "Actual ticks/s", "%.1f",
                shooter.getVelocity()
        );

        telemetry.addData(
                "PIDF",
                "P %.1f I %.2f D %.2f F %.1f",
                p, i, d, f
        );

        telemetry.addLine("===== INTAKE =====");

        telemetry.addData(
                "Auto Intake", autoIntake
        );

        telemetry.addData(
                "Ball Count", ballCount
        );

        telemetry.addData(
                "Bottom Alpha", bottomAlpha
        );

        telemetry.addData(
                "Top Alpha", topAlpha
        );

        telemetry.addData(
                "Intake Power", "%.1f",
                intakePower
        );

        telemetry.addData(
                "Indexer Power", "%.1f",
                indexerPower
        );

        telemetry.addLine("===== HOOD =====");

        telemetry.addData(
                "Hood Index", hoodIndex
        );

        telemetry.addData(
                "Hood Position", "%.4f",
                hoodPos[hoodIndex]
        );

        telemetry.update();
    }

    // ==========================================
    // PIDF
    // ==========================================

    private void applyShooterPIDF() {

        shooter.setVelocityPIDFCoefficients(
                p, i, d, f
        );
    }

    // ==========================================
    // GATE
    // ==========================================

    private void setGate(boolean open) {

        if (gate != null) {

            gate.setPosition(
                    open
                            ? 20.0 / 180.0
                            : 90.0 / 180.0
            );
        }
    }

    // ==========================================
    // HOOD
    // ==========================================

    private void setHood(int index) {

        hoodIndex = Math.max(
                0,
                Math.min(
                        hoodPos.length - 1,
                        index
                )
        );

        if (hood != null) {

            hood.setPosition(
                    hoodPos[hoodIndex]
            );
        }
    }

    // ==========================================
    // COLOR SENSORS
    // ==========================================

    private void updateSensors(long now) {

        bottomAlpha = bottom == null
                ? 0
                : bottom.alpha();

        topAlpha = top == null
                ? 0
                : top.alpha();

        bottomDetected =
                bottom != null
                        && bottomAlpha >= 300;

        topDetected =
                top != null
                        && topAlpha >= 300;

        if (autoIntake
                && !gamepad1.x
                && bottomDetected
                && !previousBottom
                && now - lastBottomNs
                > 150_000_000L) {

            ballCount = Math.min(
                    4,
                    ballCount + 1
            );

            lastBottomNs = now;
        }

        if (gamepad1.x
                && previousTop
                && !topDetected
                && now - lastTopNs
                > 150_000_000L) {

            ballCount = Math.max(
                    0,
                    ballCount - 1
            );

            lastTopNs = now;
        }

        previousBottom = bottomDetected;
        previousTop = topDetected;
    }

    // ==========================================
    // STOP
    // ==========================================

    @Override
    public void stop() {

        if (shooter != null) {
            shooter.setPower(0);
        }

        if (intake != null) {
            intake.setPower(0);
        }

        if (indexer != null) {
            indexer.setPower(0);
        }

        setGate(false);

        if (aim != null) {
            aim.disable();
        }

        if (turret != null) {
            turret.stop();
        }

        if (vision != null) {
            vision.stop();
        }

        if (drive != null) {
            drive.stop();
        }
    }
}

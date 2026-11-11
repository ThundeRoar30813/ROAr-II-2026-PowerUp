
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV80;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV5;

@TeleOp(name = "BioBuzz Turret Diagnostic V80", group = "ThunderROAR")
public class BioBuzzTurretDiagnosticV80 extends OpMode {

    // =========================================
    // SUBSYSTEMS
    // =========================================

    private DriveSubsystem drive;
    private TurretSubsystemV4 turret;
    private TurretVisionV5 vision;

    private TurretAimControllerV80 aim;

    // =========================================
    // CONFIGURACION DRIVE
    // =========================================

    public static double DRIVE_SPEED = 1.00;
    public static double TURN_SPEED = 0.85;

    public static double DEADBAND = 0.05;

    // =========================================
    // ESTADOS DE BOTONES
    // =========================================

    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;

    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadUp = false;

    // =========================================
    // INIT
    // =========================================

    @Override
    public void init() {

        drive = new DriveSubsystem(hardwareMap);

        turret = new TurretSubsystemV4(hardwareMap);

        vision = new TurretVisionV5(hardwareMap);

        aim = new TurretAimControllerV80(
                drive,
                turret,
                vision
        );

        // AprilTag inicial
        vision.setSelectedTag(37);

        // Pose inicial de ejemplo.
        // Debe coincidir con la posicion real
        // del robot antes de probar el strafe.
        //
        // NO establecemos una pose inventada
        // automaticamente.

        aim.setMode(
                TurretAimControllerV80.Mode.COMBINED
        );

        telemetry.addLine("THUNDERROAR - TURRET V80");
        telemetry.addLine("Inicializacion completada");
        telemetry.addLine("");
        telemetry.addLine("CONTROLES:");
        telemetry.addLine("Left Stick: mover robot");
        telemetry.addLine("Right Stick X: girar");
        telemetry.addLine("B: tracking ON/OFF");
        telemetry.addLine("X: AprilTag 34");
        telemetry.addLine("Y: AprilTag 37");
        telemetry.addLine("DPad LEFT: VISION_ONLY");
        telemetry.addLine("DPad RIGHT: PEDRO_ONLY");
        telemetry.addLine("DPad UP: COMBINED");
        telemetry.update();
    }

    // =========================================
    // START
    // =========================================

    @Override
    public void start() {

        vision.start();

        // Iniciar tracking desactivado
        // para verificar primero la pose.
        aim.disable();
    }

    // =========================================
    // DEADBAND
    // =========================================

    private double deadband(double value) {

        if (Math.abs(value) < DEADBAND) {
            return 0.0;
        }

        return value;
    }

    // =========================================
    // LOOP
    // =========================================

    @Override
    public void loop() {

        // =====================================
        // 1. DRIVE MECANUM
        // =====================================

        // =====================================
// DRIVE MECANUM - CORREGIDO
// =====================================

        double forward =
                deadband(-gamepad1.left_stick_y)
                        * DRIVE_SPEED;

// Invertimos el STRAFE
        double strafe =
                deadband(-gamepad1.left_stick_x)
                        * DRIVE_SPEED;

// Invertimos el GIRO
        double turn =
                deadband(-gamepad1.right_stick_x)
                        * TURN_SPEED;

        drive.drive(forward, strafe, turn);
        drive.update();

        // =====================================
        // 2. LIMELIGHT
        // =====================================

        vision.update();

        // =====================================
        // 3. BOTONES
        // =====================================

        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;
        boolean dpadUp = gamepad1.dpad_up;

        // =====================================
        // B - TRACKING ON/OFF
        // =====================================

        if (b && !lastB) {

            if (aim.isEnabled()) {

                aim.disable();

            } else {

                aim.enable();
            }
        }

        // =====================================
        // X - APRILTAG 34
        // =====================================

        if (x && !lastX) {

            vision.setSelectedTag(34);
        }

        // =====================================
        // Y - APRILTAG 37
        // =====================================

        if (y && !lastY) {

            vision.setSelectedTag(37);
        }

        // =====================================
        // DPAD LEFT - VISION ONLY
        // =====================================

        if (dpadLeft && !lastDpadLeft) {

            aim.setMode(
                    TurretAimControllerV80.Mode.VISION_ONLY
            );
        }

        // =====================================
        // DPAD RIGHT - PEDRO ONLY
        // =====================================

        if (dpadRight && !lastDpadRight) {

            aim.setMode(
                    TurretAimControllerV80.Mode.PEDRO_ONLY
            );
        }

        // =====================================
        // DPAD UP - COMBINED
        // =====================================

        if (dpadUp && !lastDpadUp) {

            aim.setMode(
                    TurretAimControllerV80.Mode.COMBINED
            );
        }

        // =====================================
        // GUARDAR BOTONES
        // =====================================

        lastB = b;
        lastX = x;
        lastY = y;

        lastDpadLeft = dpadLeft;
        lastDpadRight = dpadRight;
        lastDpadUp = dpadUp;

        // =====================================
        // 4. ACTUALIZAR TORRETA
        // =====================================

        if (aim.isEnabled()) {

            aim.update();

        } else {

            // Mantener el subsystem actualizado
            // sin habilitar tracking.
            turret.update();
        }

        // =====================================
        // 5. TELEMETRIA
        // =====================================

        telemetry.addLine("====== THUNDERROAR V80 ======");

        telemetry.addData(
                "Tracking Enabled",
                aim.isEnabled()
        );

        telemetry.addData(
                "Tracking Mode",
                aim.getMode()
        );

        telemetry.addData(
                "Tracking Status",
                aim.getStatus()
        );

        telemetry.addLine("");

        telemetry.addLine("------ PEDRO PATHING ------");

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

        telemetry.addData(
                "Chassis Rate",
                "%.2f",
                aim.getChassisRate()
        );

        telemetry.addLine("");

        telemetry.addLine("------ LIMELIGHT ------");

        telemetry.addData(
                "Selected AprilTag",
                vision.getSelectedTag()
        );
//
//        telemetry.addData(
//                "Detected AprilTag",
//                vision.getDetectedTag()
//        );
//
//        telemetry.addData(
//                "Target Visible",
//                vision.hasTarget()
//        );
//
//        telemetry.addData(
//                "Vision Status",
//                vision.getStatus()
//        );

        telemetry.addData(
                "TX",
                "%.2f",
                vision.getTx()
        );

        telemetry.addData(
                "Filtered TX",
                "%.2f",
                aim.getFilteredTx()
        );

        telemetry.addData(
                "Frame Age",
                "%.1f ms",
                vision.getAgeMs()
        );

        telemetry.addData(
                "New Frame",
                vision.isNewFrame()
        );

        telemetry.addData(
                "Frame Sequence",
                vision.getFrameSequence()
        );

        telemetry.addData(
                "Vision Rate",
                "%.2f",
                aim.getVisionRate()
        );

        telemetry.addLine("");

        telemetry.addLine("------ TURRET ------");

        telemetry.addData(
                "Actual Angle",
                "%.2f",
                turret.getAngle()
        );

        telemetry.addData(
                "Target Angle",
                "%.2f",
                turret.getTarget()
        );

        telemetry.addData(
                "Turret Error",
                "%.2f",
                turret.getError()
        );

        telemetry.addData(
                "Turret Velocity",
                "%.2f",
                turret.getVelocity()
        );

        telemetry.addData(
                "Motor Power",
                "%.3f",
                turret.getPower()
        );

        telemetry.addData(
                "Turret Fault",
                turret.isFault()
        );

        telemetry.addData(
                "Base Target",
                "%.2f",
                aim.getBaseTarget()
        );

        telemetry.addData(
                "Smooth Target",
                "%.2f",
                aim.getSmoothTarget()
        );

        telemetry.addData(
                "Smooth Velocity",
                "%.2f",
                aim.getSmoothVelocity()
        );

        telemetry.addData(
                "Feedforward",
                "%.2f",
                aim.getFilteredFF()
        );

        telemetry.addLine("");

        telemetry.addLine("B: ON/OFF | X: Tag34 | Y: Tag37");
        telemetry.addLine("LEFT: Vision | RIGHT: Pedro | UP: Combined");

        telemetry.update();
    }

    // =========================================
    // STOP
    // =========================================

    @Override
    public void stop() {

        if (aim != null) {
            aim.disable();
        }

        if (drive != null) {
            drive.stop();
        }

        if (vision != null) {
            vision.stop();
        }

        if (turret != null) {
            turret.stop();
        }
    }
}

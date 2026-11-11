
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV76;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV5;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV75;

@TeleOp(
        name = "BioBuzz Turret Diagnostic V76",
        group = "ThunderROAR"
)
public class BioBuzzTurretDiagnosticV76 extends OpMode {

    private DriveSubsystem drive;
    private TurretSubsystemV4 turret;
    private TurretVisionV5 vision;
    private TurretAimControllerV75 aim;

    // Velocidad del chasis
    public static double DRIVE_SPEED = 0.80;
    public static double TURN_SPEED = 0.70;
    public static double DEADBAND = 0.05;

    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastLeft = false;
    private boolean lastRight = false;
    private boolean lastUp = false;

    @Override
    public void init() {

        drive = new DriveSubsystem(hardwareMap);

        turret = new TurretSubsystemV4(hardwareMap);

        vision = new TurretVisionV5(hardwareMap);

        aim = new TurretAimControllerV75(
                drive,
                turret,
                vision
        );

        // AprilTag inicial
        vision.setSelectedTag(37);

        // Modo inicial
        aim.setMode(
                TurretAimControllerV75.Mode.COMBINED
        );

        telemetry.addLine("THUNDERROAR - V76");
        telemetry.addLine("Verificar pose inicial de Pedro");
        telemetry.addLine("B: Tracking ON/OFF");
        telemetry.addLine("X: Tag 34 | Y: Tag 37");
        telemetry.addLine("DPAD LEFT: Vision");
        telemetry.addLine("DPAD RIGHT: Pedro");
        telemetry.addLine("DPAD UP: Combined");
        telemetry.update();
    }

    @Override
    public void start() {

        vision.start();

        // Tracking apagado hasta presionar B
    }

    private double deadband(double value) {

        if (Math.abs(value) < DEADBAND) {
            return 0.0;
        }

        return value;
    }

    @Override
    public void loop() {

        // =====================================
        // 1. DRIVE MECANUM
        // =====================================

        double forward =
                deadband(-gamepad1.left_stick_y)
                        * DRIVE_SPEED;

        // Strafe corregido
        double strafe =
                deadband(-gamepad1.left_stick_x)
                        * DRIVE_SPEED;

        // Giro corregido
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

        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        boolean up = gamepad1.dpad_up;

        // Seleccionar AprilTag 34
        if (x && !lastX) {
            vision.setSelectedTag(34);
        }

        // Seleccionar AprilTag 37
        if (y && !lastY) {
            vision.setSelectedTag(37);
        }

//        // Solo vision
//        if (left && !lastLeft) {
//            aim.setMode(
//                    TurretAimControllerV76.Mode.VISION_ONLY
//            );
//        }
//
//        // Solo Pedro
//        if (right && !lastRight) {
//            aim.setMode(
//                    TurretAimControllerV76.Mode.PEDRO_ONLY
//            );
//        }
//
//        // Pedro + Limelight
//        if (up && !lastUp) {
//            aim.setMode(
//                    TurretAimControllerV76.Mode.COMBINED
//            );
//        }

        // Tracking ON / OFF
        if (b && !lastB) {

            if (aim.isEnabled()) {
                aim.disable();
            } else {
                aim.enable();
            }
        }

        lastB = b;
        lastX = x;
        lastY = y;

        lastLeft = left;
        lastRight = right;
        lastUp = up;

        // =====================================
        // 4. ACTUALIZAR TORRETA
        // =====================================

        if (aim.isEnabled()) {
            aim.update();
        } else {
            turret.update();
        }

        // =====================================
        // 5. TELEMETRIA
        // =====================================

        telemetry.addLine("====== THUNDERROAR V76 ======");

        telemetry.addData(
                "Tracking",
                aim.isEnabled()
        );

        telemetry.addData(
                "Modo",
                aim.getMode()
        );

        telemetry.addData(
                "Estado",
                aim.getStatus()
        );

        telemetry.addLine("");

        telemetry.addLine("------ APRILTAG ------");

        telemetry.addData(
                "Tag seleccionado",
                vision.getSelectedTag()
        );

        telemetry.addData(
                "Tag visible",
                vision.hasTarget()
        );

        telemetry.addData(
                "TX",
                "%.2f",
                vision.getTx()
        );

        telemetry.addData(
                "TX filtrado",
                "%.2f",
                aim.getFilteredTx()
        );

        telemetry.addLine("");

        telemetry.addLine("------ PEDRO ------");

        telemetry.addData(
                "Pedro X",
                "%.2f",
                drive.getX()
        );

        telemetry.addData(
                "Pedro Y",
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

        telemetry.addLine("------ TURRET ------");

        telemetry.addData(
                "Torreta actual",
                "%.2f",
                turret.getAngle()
        );

        telemetry.addData(
                "Objetivo",
                "%.2f",
                aim.getSmoothTarget()
        );

        telemetry.addData(
                "Error objetivo",
                "%.2f",
                aim.getSmoothTarget() - turret.getAngle()
        );

        telemetry.addData(
                "Base Target",
                "%.2f",
                aim.getBaseTarget()
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

        telemetry.addData(
                "Fallo torreta",
                turret.isFault()
        );

        telemetry.addLine("");

        telemetry.addLine("B: ON/OFF");
        telemetry.addLine("X: Tag34 | Y: Tag37");
        telemetry.addLine("LEFT: Vision");
        telemetry.addLine("RIGHT: Pedro");
        telemetry.addLine("UP: Combined");

        telemetry.update();
    }

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
    }
}

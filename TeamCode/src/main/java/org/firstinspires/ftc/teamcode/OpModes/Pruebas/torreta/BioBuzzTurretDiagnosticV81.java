
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV81;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV5;


@TeleOp(
        name = "BioBuzz Turret Diagnostic V81",
        group = "ThunderROAR"
)
public class BioBuzzTurretDiagnosticV81 extends OpMode {

    private DriveSubsystem drive;
    private TurretSubsystemV4 turret;
    private TurretVisionV5 vision;
    private TurretAimControllerV81 aim;

    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;

    // IMPORTANTE:
    // Reemplazar con la pose inicial REAL de Pedro.
    public static double START_X = 0.0;
    public static double START_Y = 0.0;
    public static double START_HEADING_DEG = 0.0;

    public static double DRIVE_SPEED = 0.45;
    public static double TURN_SPEED = 0.40;

    @Override
    public void init() {

        drive = new DriveSubsystem(hardwareMap);

        turret = new TurretSubsystemV4(hardwareMap);

        vision = new TurretVisionV5(hardwareMap);

        aim = new TurretAimControllerV81(
                drive,
                turret,
                vision
        );

        vision.setSelectedTag(37);

        telemetry.addLine(
                "THUNDERROAR - BIOBUZZ V81"
        );

        telemetry.addLine(
                "Configurar pose inicial real de Pedro"
        );

        telemetry.addLine(
                "B = Tracking ON/OFF"
        );

        telemetry.addLine(
                "X = Tag 34 | Y = Tag 37"
        );

        telemetry.update();
    }

    @Override
    public void start() {

        drive.setInitialPose(
                START_X,
                START_Y,
                START_HEADING_DEG
        );

        vision.start();
    }

    private double deadband(double value) {

        if (Math.abs(value) < 0.05) {
            return 0.0;
        }

        return value;
    }

    @Override
    public void loop() {

        // ======================================
        // 1. CHASIS
        // ======================================

        double forward =
                deadband(-gamepad1.left_stick_y)
                        * DRIVE_SPEED;

        double strafe =
                deadband(-gamepad1.left_stick_x)
                        * DRIVE_SPEED;

        double turn =
                deadband(-gamepad1.right_stick_x)
                        * TURN_SPEED;

        drive.drive(forward, strafe, turn);
        drive.update();

        // ======================================
        // 2. VISION
        // ======================================

        vision.update();

        // ======================================
        // 3. SELECCION DE APRILTAG
        // ======================================

        if (gamepad1.x && !lastX) {
            vision.setSelectedTag(34);
        }

        if (gamepad1.y && !lastY) {
            vision.setSelectedTag(37);
        }

        // ======================================
        // 4. TRACKING
        // ======================================

        if (gamepad1.b && !lastB) {

            if (aim.isEnabled()) {
                aim.disable();
            } else {
                aim.enable();
            }
        }

        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastB = gamepad1.b;

        // ======================================
        // 5. ACTUALIZACION TORRETA
        // ======================================

        if (aim.isEnabled()) {
            aim.update();
        } else {
            turret.update();
        }

        // ======================================
        // 6. TELEMETRIA
        // ======================================

        telemetry.addLine(
                "====== THUNDERROAR V81 ======"
        );

        telemetry.addData(
                "Tracking",
                aim.isEnabled()
        );

        telemetry.addData(
                "Status",
                aim.getStatus()
        );

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
                "Vision offset",
                "%.2f",
                aim.getVisionOffset()
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

        telemetry.addLine("");

        telemetry.addLine("------ TARGET ------");

        telemetry.addData(
                "Geometry target",
                "%.2f",
                aim.getGeometryTarget()
        );

        telemetry.addData(
                "Desired target",
                "%.2f",
                aim.getDesiredTarget()
        );

        telemetry.addData(
                "Smooth target",
                "%.2f",
                aim.getSmoothTarget()
        );

        telemetry.addLine("");

        telemetry.addLine("------ MOTOR ------");

        telemetry.addData(
                "Turret actual",
                "%.2f",
                turret.getAngle()
        );

        telemetry.addData(
                "Turret velocity",
                "%.2f",
                turret.getVelocity()
        );

        telemetry.addData(
                "Motor power",
                "%.2f",
                turret.getPower()
        );

        telemetry.addData(
                "Fault",
                turret.getFaultReason()
        );

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

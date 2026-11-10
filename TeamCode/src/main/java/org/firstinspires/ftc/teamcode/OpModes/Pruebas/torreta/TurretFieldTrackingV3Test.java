
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimingV3;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV3;

@TeleOp(name = "TEST - Turret Field Tracking V3")
public class TurretFieldTrackingV3Test extends OpMode {

    // =====================================================
    // CHASIS
    // =====================================================

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCommand;

    // =====================================================
    // TORRETA
    // =====================================================

    private TurretSubsystemV3 turret;
    private TurretAimingV3 tracking;

    // Coordenadas del objetivo
    private static final double TARGET_X = 57.0;
    private static final double TARGET_Y = 59.0;

    // =====================================================
    // BOTONES
    // =====================================================

    private boolean previousA = false;
    private boolean previousB = false;
    private boolean previousX = false;
    private boolean previousY = false;

    // =====================================================
    // INIT
    // =====================================================

    @Override
    public void init() {

        drive = new DriveSubsystem(hardwareMap);

        driveCommand = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );

        turret = new TurretSubsystemV3(hardwareMap);

        // Potencia de prueba
        turret.setMaxPower(0.20);

        tracking = new TurretAimingV3(
                drive,
                turret,
                TARGET_X,
                TARGET_Y
        );

        telemetry.addLine("===== TURRET FIELD TRACKING V3 =====");
        telemetry.addLine("B = Tracking ON/OFF");
        telemetry.addLine("A = Turret 0 grados");
        telemetry.addLine("X = Turret +20 grados");
        telemetry.addLine("Y = Turret -20 grados");
        telemetry.addLine("Joysticks = Mover chasis");
        telemetry.addLine("");
        telemetry.addLine("Antes de activar B:");
        telemetry.addLine("Apuntar fisicamente al objetivo");
        telemetry.update();
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
        // 2. BOTONES
        // =================================================

        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        // B: ACTIVAR / DESACTIVAR TRACKING
        if (b && !previousB) {

            if (tracking.isEnabled()) {

                tracking.disable();

            } else {

                // Si hubo Stall, requiere revisar
                // el mecanismo antes de rearmar.
                if (!turret.isStalled()) {
                    tracking.enable();
                }
            }
        }

        // A: VOLVER A CERO
        if (a && !previousA) {

            tracking.disable();

            if (!turret.isStalled()) {

                turret.enable();
                turret.setTarget(0);
            }
        }

        // X: +20 GRADOS
        if (x && !previousX) {

            tracking.disable();

            if (!turret.isStalled()) {

                turret.enable();
                turret.setTarget(20);
            }
        }

        // Y: -20 GRADOS
        if (y && !previousY) {

            tracking.disable();

            if (!turret.isStalled()) {

                turret.enable();
                turret.setTarget(-20);
            }
        }

        previousA = a;
        previousB = b;
        previousX = x;
        previousY = y;

        // =================================================
        // 3. ACTUALIZAR TORRETA
        // =================================================

        if (tracking.isEnabled()) {

            tracking.execute();

        } else {

            turret.update();
        }

        // =================================================
        // 4. TELEMETRIA
        // =================================================

        telemetry.addLine("========== ROBOT ==========");

        telemetry.addData(
                "X", "%.2f", drive.getX()
        );

        telemetry.addData(
                "Y", "%.2f", drive.getY()
        );

        telemetry.addData(
                "Heading", "%.2f", drive.getHeadingDegrees()
        );

        telemetry.addLine("========== TURRET ==========");

        telemetry.addData(
                "Tracking",
                tracking.isEnabled() ? "ON" : "OFF"
        );

        telemetry.addData(
                "Reachable",
                tracking.isReachable()
        );

        telemetry.addData(
                "Target Angle",
                "%.2f", turret.getTarget()
        );

        telemetry.addData(
                "Actual Angle",
                "%.2f", turret.getAngle()
        );

        telemetry.addData(
                "Error",
                "%.2f", turret.getError()
        );

        telemetry.addData(
                "Motor Power",
                "%.3f", turret.getPower()
        );

        telemetry.addData(
                "Velocity",
                "%.2f", turret.getVelocity()
        );

        telemetry.addData(
                "Encoder Ticks",
                turret.getTicks()
        );

        telemetry.addData(
                "Turret Enabled",
                turret.isEnabled()
        );

        telemetry.addData(
                "Turret Stall",
                turret.isStalled()
        );

        telemetry.addLine("========== AIMING ==========");

        telemetry.addData(
                "Target X",
                "%.2f", tracking.getTargetX()
        );

        telemetry.addData(
                "Target Y",
                "%.2f", tracking.getTargetY()
        );

        telemetry.addData(
                "Field Angle",
                "%.2f", tracking.getFieldAngle()
        );

        telemetry.addData(
                "Distance",
                "%.2f", tracking.getDistance()
        );

        telemetry.addData(
                "Desired Angle",
                "%.2f", tracking.getDesiredAngle()
        );

        telemetry.update();
    }

    // =====================================================
    // STOP
    // =====================================================

    @Override
    public void stop() {

        if (tracking != null) {
            tracking.disable();
        }

        if (turret != null) {
            turret.stop();
        }

        if (driveCommand != null) {
            //driveCommand.end();
        }
    }
}

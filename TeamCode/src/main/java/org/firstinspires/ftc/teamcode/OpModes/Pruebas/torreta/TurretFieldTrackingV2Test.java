package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.turret.TurretFieldTrackingCommand1;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemp;

@TeleOp(name = "TEST - Turret Field Tracking V2")
public class TurretFieldTrackingV2Test extends OpMode {

    // =====================================================
    // SUBSYSTEMS
    // =====================================================

    private DriveSubsystem drive;
    private TurretSubsystemp turretp;

    // =====================================================
    // COMMANDS
    // =====================================================

    private TurretFieldTrackingCommand1 tracking;

    // =====================================================
    // TARGET EN EL CAMPO (PULGADAS)
    // =====================================================

    private static final double TARGET_X = 72.0;
    private static final double TARGET_Y = 72.0;

    // =====================================================
    // CONTROLES
    // =====================================================

    private boolean previousA = false;
    private boolean previousB = false;
    private boolean previousX = false;

    // =====================================================
    // INIT
    // =====================================================

    @Override
    public void init() {

        drive = new DriveSubsystem(hardwareMap);
        turretp = new TurretSubsystemp(hardwareMap);

        //turret = new TurretSubsystemp(hardwareMap);

        tracking = new TurretFieldTrackingCommand1(
                drive,
                turretp,
                TARGET_X,
                TARGET_Y
        );

        telemetry.addLine("TURRET FIELD TRACKING V2");
        telemetry.addLine();
        telemetry.addLine("Coloca la torreta mirando al frente");
        telemetry.addLine("A = Activar tracking");
        telemetry.addLine("B = Desactivar tracking");
        telemetry.addLine("X = Reset zero (tracking OFF)");
        telemetry.addLine();
        telemetry.addLine("Target: (72, 72)");
        telemetry.update();
    }

    // =====================================================
    // START
    // =====================================================

    @Override
    public void start() {

        // El encoder incremental toma esta posición como 0°.
        turretp.resetZero();
    }

    // =====================================================
    // LOOP
    // =====================================================

    @Override
    public void loop() {

        // =================================================
        // DRIVE
        // =================================================

        /*
         * Se asume que tu DriveSubsystem expone:
         *
         * drive.drive(forward, strafe, turn)
         * drive.update()
         *
         * Si tu método tiene otro nombre, ajusta
         * únicamente esta sección.
         */

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        drive.drive(
                forward,
                strafe,
                turn
        );

        drive.update();

        // =================================================
        // BOTONES
        // =================================================

        boolean a = gamepad2.a;
        boolean b = gamepad2.b;
        boolean x = gamepad2.x;

        // Activar tracking
        if (a && !previousA) {

            tracking.enable();
        }

        // Desactivar tracking
        if (b && !previousB) {

            tracking.disable();
        }

        // Reiniciar cero solamente con tracking OFF
        if (x && !previousX && !tracking.isEnabled()) {

            turretp.resetZero();
        }

        previousA = a;
        previousB = b;
        previousX = x;

        // =================================================
        // TRACKING
        // =================================================

        if (tracking.isEnabled()) {

            tracking.execute();
        }

        // =================================================
        // TELEMETRY - ROBOT
        // =================================================

        telemetry.addLine("===== ROBOT =====");

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

        // =================================================
        // TELEMETRY - TARGET
        // =================================================

        telemetry.addLine();
        telemetry.addLine("===== TARGET =====");

        telemetry.addData(
                "Target X",
                "%.2f",
                tracking.getTargetX()
        );

        telemetry.addData(
                "Target Y",
                "%.2f",
                tracking.getTargetY()
        );

        telemetry.addData(
                "Distance",
                "%.2f in",
                tracking.getDistanceToTarget()
        );

        telemetry.addData(
                "Field Angle",
                "%.2f deg",
                tracking.getFieldAngleDeg()
        );

        // =================================================
        // TELEMETRY - TURRET
        // =================================================

        telemetry.addLine();
        telemetry.addLine("===== TURRET V2 =====");

        telemetry.addData(
                "Tracking",
                tracking.isEnabled() ? "ON" : "OFF"
        );

        telemetry.addData(
                "Target Angle",
                "%.2f deg",
                turretp.getTargetAngleDeg()
        );

        telemetry.addData(
                "Actual Angle",
                "%.2f deg",
                turretp.getCurrentAngleDeg()
        );

        telemetry.addData(
                "Error",
                "%.2f deg",
                turretp.getErrorDeg()
        );

        // =================================================
        // VELOCITY FEEDFORWARD
        // =================================================

        telemetry.addLine();
        telemetry.addLine("===== VELOCITY FF =====");

        telemetry.addData(
                "Target Velocity",
                "%.2f deg/s",
                tracking.getTargetVelocityDegPerSec()
        );

//        telemetry.addData(
//                "PID Output",
//                "%.3f",
//                turret.getPIDOutput()
//        );
//
//        telemetry.addData(
//                "Feedforward",
//                "%.3f",
//                turret.getFeedforward()
//        );

        telemetry.addData(
                "Motor Power",
                "%.3f",
                turretp.getMotorPower()
        );

        // =================================================
        // ENCODER
        // =================================================

        telemetry.addLine();
        telemetry.addLine("===== ENCODER =====");

        telemetry.addData(
                "Raw Ticks",
                turretp.getRawTicks()
        );

        telemetry.addData(
                "Relative Ticks",
                turretp.getRelativeTicks()
        );

        telemetry.update();
    }

    // =====================================================
    // STOP
    // =====================================================

    @Override
    public void stop() {

        tracking.disable();

        turretp.stop();
    }
}
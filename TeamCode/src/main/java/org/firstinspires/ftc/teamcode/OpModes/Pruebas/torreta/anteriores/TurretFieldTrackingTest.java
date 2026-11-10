package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta.anteriores;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.TurretFieldTrackingCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

@TeleOp(name = "TEST - Turret Field Tracking")
public class TurretFieldTrackingTest extends OpMode {

    private DriveSubsystem drive;
    private TurretSubsystem turret;

    private MecanumDriveCmd driveCommand;

    private TurretFieldTrackingCommand tracking;




    // =========================================================
    // OBJETIVO DE PRUEBA
    // =========================================================

    private static final double TARGET_X = 72.0;
    private static final double TARGET_Y = 72.0;


    private boolean previousA = false;
    private boolean previousB = false;


    @Override
    public void init() {

        drive =
                new DriveSubsystem(
                        hardwareMap
                );

        turret =
                new TurretSubsystem(
                        hardwareMap
                );


        driveCommand =
                new MecanumDriveCmd(
                        drive,

                        () -> -gamepad1.left_stick_y,

                        () -> gamepad1.left_stick_x,

                        // Ya corregimos el giro.
                        () -> -gamepad1.right_stick_x
                );


        tracking =
                new TurretFieldTrackingCommand(
                        drive,
                        turret,

                        TARGET_X,
                        TARGET_Y
                );


        telemetry.addLine(
                "TURRET FIELD TRACKING"
        );

        telemetry.addLine("");

        telemetry.addData(
                "Objetivo",
                "(%.1f, %.1f)",
                TARGET_X,
                TARGET_Y
        );

        telemetry.addLine("");

        telemetry.addLine(
                "ANTES DE PRESIONAR A:"
        );

        telemetry.addLine(
                "Apunta físicamente la torreta al objetivo."
        );

        telemetry.addLine("");

        telemetry.addLine(
                "A = activar tracking"
        );

        telemetry.addLine(
                "B = desactivar"
        );

        telemetry.update();
    }


    @Override
    public void loop() {

        // =====================================================
        // CHASIS
        // =====================================================

        driveCommand.execute();

        drive.update();


        // =====================================================
        // ACTIVAR
        // =====================================================

        boolean currentA =
                gamepad1.a;

        if (currentA && !previousA) {

            tracking.enable();
        }

        previousA = currentA;


        // =====================================================
        // DESACTIVAR
        // =====================================================

        boolean currentB =
                gamepad1.b;

        if (currentB && !previousB) {

            tracking.disable();
        }

        previousB = currentB;


        // =====================================================
        // TRACKING
        // =====================================================

        tracking.execute();


        // =====================================================
        // TELEMETRY
        // =====================================================

        telemetry.addLine(
                "===== ROBOT ====="
        );

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


        telemetry.addLine(
                "===== TARGET ====="
        );

//        telemetry.addData(
//                "Target X",
//                "%.2f",
//                tracking.getTargetX()
//        );
//
//        telemetry.addData(
//                "Target Y",
//                "%.2f",
//                tracking.getTargetY()
//        );

        telemetry.addData(
                "Distance",
                "%.2f in",
                tracking.getDistanceToTarget()
        );

//        telemetry.addData(
//                "Field Angle",
//                "%.2f deg",
//                tracking.getFieldAngleDeg()
//        );


        telemetry.addLine(
                "===== TURRET ====="
        );

        telemetry.addData(
                "Tracking",
                tracking.isEnabled()
                        ? "ON"
                        : "OFF"
        );

        telemetry.addData(
                "Target Angle",
                "%.2f deg",
                turret.getTargetAngleDeg()
        );

        telemetry.addData(
                "Actual Angle",
                "%.2f deg",
                turret.getCurrentAngleDeg()
        );

        telemetry.addData(
                "Error",
                "%.2f deg",
                turret.getErrorDeg()
        );

        telemetry.addData(
                "Power",
                "%.3f",
                turret.getMotorPower()
        );

        telemetry.update();
    }


    @Override
    public void stop() {

        //driveCommand.ned();

        tracking.disable();

        turret.stop();
    }
}
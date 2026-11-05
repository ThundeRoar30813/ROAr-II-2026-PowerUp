
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Commands.intake.IntakeCmd;

@TeleOp(name = "TeleOp Principal")
public class IntakeTeleOp extends LinearOpMode {

    private final IntakeSubsystem intake =
            new IntakeSubsystem();

    private IntakeCmd intakeCommand;

    @Override
    public void runOpMode() {

        // =========================
        // INICIALIZACION
        // =========================

        intake.init(hardwareMap);

        intakeCommand = new IntakeCmd(intake);

        boolean previousA = false;
        boolean previousB = false;
        boolean previousX = false;

        telemetry.addLine("Robot listo");
        telemetry.update();

        waitForStart();

        // =========================
        // LOOP PRINCIPAL
        // =========================

        while (opModeIsActive()) {

            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;

            // A: activar/desactivar recoleccion
            if (a && !previousA) {
                intakeCommand.toggleCollecting();
            }

            // B: parada de emergencia
            if (b && !previousB) {
                intakeCommand.emergencyStop();
            }

            // X: reset de inventario
            if (x && !previousX
                    && !intakeCommand.isCollecting()
                    && !intakeCommand.isShooting()) {

                intakeCommand.resetInventory();
            }

            // Y: disparar mientras se mantiene
            intakeCommand.setShooting(gamepad1.y);

            // Ejecutar automatizacion
            intakeCommand.update();

            previousA = a;
            previousB = b;
            previousX = x;

            // =========================
            // TELEMETRIA
            // =========================

            telemetry.addData(
                    "Pelotas", "%d / 4",
                    intakeCommand.getBallCount());

            telemetry.addData(
                    "Recolectando",
                    intakeCommand.isCollecting());

            telemetry.addData(
                    "Disparando",
                    intakeCommand.isShooting());

            telemetry.addData(
                    "Sensor inferior",
                    intake.hasBottomPiece());

            telemetry.addData(
                    "Sensor superior",
                    intake.hasTopPiece());

            boolean[] inventory =
                    intakeCommand.getInventory();

            for (int i = 0; i < inventory.length; i++) {

                telemetry.addData(
                        "Pelota " + (i + 1),
                        inventory[i] ? "OCUPADA" : "VACIA");
            }

            telemetry.update();
        }

        intakeCommand.emergencyStop();
    }
}

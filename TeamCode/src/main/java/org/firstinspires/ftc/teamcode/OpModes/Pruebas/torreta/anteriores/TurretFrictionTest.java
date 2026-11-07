package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta.anteriores;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TEST - Turret Friction")
public class TurretFrictionTest extends LinearOpMode {

    private DcMotorEx turretMotor;

    @Override
    public void runOpMode() {

        turretMotor = hardwareMap.get(
                DcMotorEx.class,
                "turret"
        );

        turretMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        turretMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        turretMotor.setDirection(
                DcMotor.Direction.REVERSE
        );

        telemetry.addLine("TURRET FRICTION TEST");
        telemetry.addLine("");
        telemetry.addLine("RB = gira derecha");
        telemetry.addLine("LB = gira izquierda");
        telemetry.addLine("");
        telemetry.addLine("D-PAD:");
        telemetry.addLine("DOWN  = 10%");
        telemetry.addLine("LEFT  = 15%");
        telemetry.addLine("UP    = 20%");
        telemetry.addLine("RIGHT = 25%");
        telemetry.update();

        waitForStart();

        double testPower = 0.10;

        while (opModeIsActive()) {

            // Seleccionar potencia
            if (gamepad2.dpad_down) {
                testPower = 0.20;
            }

            if (gamepad2.dpad_left) {
                testPower = 0.25;
            }

            if (gamepad2.dpad_up) {
                testPower = 0.30;
            }

            if (gamepad2.dpad_right) {
                testPower = 0.35;
            }

            double motorPower = 0.0;

            // Mantener presionado para mover
            if (gamepad2.right_bumper) {
                motorPower = testPower;
            }

            if (gamepad2.left_bumper) {
                motorPower = -testPower;
            }

            turretMotor.setPower(motorPower);

            telemetry.addLine("===== FRICTION TEST =====");

            telemetry.addData(
                    "Test Power",
                    "%.2f",
                    testPower
            );

            telemetry.addData(
                    "Motor Power",
                    "%.2f",
                    motorPower
            );

            telemetry.addLine("");
            telemetry.addLine("RB = derecha");
            telemetry.addLine("LB = izquierda");

            telemetry.update();
        }

        turretMotor.setPower(0);
    }
}
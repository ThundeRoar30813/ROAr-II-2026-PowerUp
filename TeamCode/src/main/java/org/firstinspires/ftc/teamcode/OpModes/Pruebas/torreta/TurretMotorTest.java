package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TEST - Turret Motor")
public class TurretMotorTest extends LinearOpMode {

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

        telemetry.addLine("TURRET MOTOR TEST");
        telemetry.addLine("Joystick derecho Gamepad 2");
        telemetry.addLine("Empieza MUY despacio.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double joystick = gamepad2.right_stick_x;

            // SOLO 20% de potencia para esta prueba
            double power = joystick * 0.90;

            turretMotor.setPower(power);

            telemetry.addData("Joystick", joystick);
            telemetry.addData("Motor Power", power);
            telemetry.update();
        }

        turretMotor.setPower(0);
    }
}
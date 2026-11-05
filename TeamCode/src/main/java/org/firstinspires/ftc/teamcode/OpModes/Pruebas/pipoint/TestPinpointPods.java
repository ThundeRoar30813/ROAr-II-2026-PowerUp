package org.firstinspires.ftc.teamcode.OpModes.Pruebas.pipoint;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "TEST Pinpoint Pods", group = "Pruebas")
public class TestPinpointPods extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        // =========================================================
        // PINPOINT
        // =========================================================

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pip"
        );

        // =========================================================
        // DIRECCIÓN DE LOS PODS
        // =========================================================
        //
        // Primer valor  = FORWARD POD
        // Segundo valor = STRAFE POD
        //
        // Dejamos Forward normal
        // Dejamos Strafe INVERTIDO











        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );












        // =========================================================
        // RESET
        // =========================================================

        pinpoint.resetPosAndIMU();


        telemetry.addLine("==============================");
        telemetry.addLine("   PINPOINT POD TEST");
        telemetry.addLine("==============================");
        telemetry.addLine();
        telemetry.addLine("Forward = NORMAL");
        telemetry.addLine("Strafe  = REVERSED");
        telemetry.addLine();
        telemetry.addLine("Presiona PLAY");
        telemetry.update();


        waitForStart();


        // =========================================================
        // LOOP
        // =========================================================

        while (opModeIsActive()) {

            // Actualizar información del Pinpoint
            pinpoint.update();


            // =====================================================
            // ENCODERS RAW
            // =====================================================

            int forwardEncoder = pinpoint.getEncoderX();
            int strafeEncoder = pinpoint.getEncoderY();


            // =====================================================
            // TELEMETRY
            // =====================================================

            telemetry.addLine("======= PODS =======");

            telemetry.addData(
                    "FORWARD POD",
                    forwardEncoder
            );

            telemetry.addData(
                    "STRAFE POD",
                    strafeEncoder
            );


            telemetry.addLine();
            telemetry.addLine("======= PRUEBA =======");

            telemetry.addLine("Mueve robot ADELANTE:");
            telemetry.addLine("  Forward debe cambiar");
            telemetry.addLine();

            telemetry.addLine("Mueve robot ATRAS:");
            telemetry.addLine("  Forward debe cambiar al contrario");
            telemetry.addLine();

            telemetry.addLine("Mueve robot DERECHA:");
            telemetry.addLine("  Strafe debe cambiar");
            telemetry.addLine();

            telemetry.addLine("Mueve robot IZQUIERDA:");
            telemetry.addLine("  Strafe debe cambiar al contrario");


            telemetry.addLine();
            telemetry.addLine("====================");

            telemetry.update();

            sleep(20);
        }
    }
}
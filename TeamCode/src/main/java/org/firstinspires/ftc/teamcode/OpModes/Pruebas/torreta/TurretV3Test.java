
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV3;

@TeleOp(name = "TEST - Turret V3")
public class TurretV3Test extends OpMode {

    private TurretSubsystemV3 turret;

    private boolean previousA;
    private boolean previousB;
    private boolean previousX;
    private boolean previousY;

    @Override
    public void init() {

        turret = new TurretSubsystemV3(hardwareMap);
        turret.setMaxPower(0.20);

        telemetry.addLine("TURRET V3 TEST");
        telemetry.addLine("Colocar torreta en cero antes de INIT");
        telemetry.addLine("A = objetivo 0 grados");
        telemetry.addLine("X = objetivo +20 grados");
        telemetry.addLine("Y = objetivo -20 grados");
        telemetry.addLine("B = detener / rearmar");
        telemetry.update();
    }

    @Override
    public void loop() {

        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        if (b && !previousB) {

            turret.disable();

            // Rearmar solo despues de revisar
            // que no haya obstruccion
            turret.clearFault();
        }

        if (!turret.isStalled()) {

            if (a && !previousA) {

                if (!turret.isEnabled()) turret.enable();
                turret.setTarget(0);
            }

            if (x && !previousX) {

                if (!turret.isEnabled()) turret.enable();
                turret.setTarget(20);
            }

            if (y && !previousY) {

                if (!turret.isEnabled()) turret.enable();
                turret.setTarget(-20);
            }
        }

        previousA = a;
        previousB = b;
        previousX = x;
        previousY = y;

        turret.update();

        telemetry.addLine("===== TURRET V3 =====");

        telemetry.addData("Target", "%.2f", turret.getTarget());
        telemetry.addData("Actual", "%.2f", turret.getAngle());
        telemetry.addData("Error", "%.2f", turret.getError());

        telemetry.addData("Power", "%.3f", turret.getPower());
        telemetry.addData("Velocity", "%.2f", turret.getVelocity());
        telemetry.addData("Ticks", turret.getTicks());

        telemetry.addData("Enabled", turret.isEnabled());
        telemetry.addData("Stall", turret.isStalled());

        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
    }
}

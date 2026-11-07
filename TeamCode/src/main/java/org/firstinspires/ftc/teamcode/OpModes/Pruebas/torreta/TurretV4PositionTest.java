
package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV3;

@TeleOp(name = "TEST - Turret V4 Position 50%")
public class TurretV4PositionTest extends OpMode {

    private TurretSubsystemV3 turret;

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;

    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastLeft = false;
    private boolean lastRight = false;

    private boolean lastReset = false;

    @Override
    public void init() {

        turret = new TurretSubsystemV3(hardwareMap);

        // Potencia maxima 50%
        turret.setMaxPower(0.50);

        telemetry.addLine("TURRET V4 - POSITION TEST");
        telemetry.addLine("A = 0 grados");
        telemetry.addLine("X = +90 grados");
        telemetry.addLine("Y = -90 grados");
        telemetry.addLine("DPad UP = +180 grados");
        telemetry.addLine("DPad RIGHT = +220 grados");
        telemetry.addLine("DPad LEFT = -100 grados");
        telemetry.addLine("DPad DOWN = 0 grados");
        telemetry.addLine("B = Desactivar torreta");
        telemetry.addLine("Gamepad 2 START = Clear Fault");
        telemetry.update();
    }

    private void moveTo(double angle) {

        if (turret.isStalled()) {
            return;
        }

        turret.enable();
        turret.setTarget(angle);
    }

    @Override
    public void loop() {

        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;

        boolean reset = gamepad2.start;

        if (reset && !lastReset) {
            turret.clearFault();
        }

        if (a && !lastA) {
            moveTo(0);
        }

        if (x && !lastX) {
            moveTo(90);
        }

        if (y && !lastY) {
            moveTo(-90);
        }

        if (up && !lastUp) {
            moveTo(180);
        }

        if (right && !lastRight) {
            moveTo(220);
        }

        if (left && !lastLeft) {
            moveTo(-100);
        }

        if (down && !lastDown) {
            moveTo(0);
        }

        if (b && !lastB) {
            turret.disable();
        }

        lastA = a;
        lastB = b;
        lastX = x;
        lastY = y;

        lastUp = up;
        lastDown = down;
        lastLeft = left;
        lastRight = right;

        lastReset = reset;

        turret.update();

        telemetry.addLine("===== TURRET V4 =====");

        telemetry.addData(
                "Target", "%.2f", turret.getTarget()
        );

        telemetry.addData(
                "Actual", "%.2f", turret.getAngle()
        );

        telemetry.addData(
                "Error", "%.2f", turret.getError()
        );

        telemetry.addData(
                "Power", "%.3f", turret.getPower()
        );

        telemetry.addData(
                "Max Power", "%.2f", turret.getMaxPower()
        );

        telemetry.addData(
                "Velocity", "%.2f", turret.getVelocity()
        );

        telemetry.addData(
                "Ticks", turret.getTicks()
        );

        telemetry.addData(
                "Enabled", turret.isEnabled()
        );

        telemetry.addData(
                "Stall", turret.isStalled()
        );

        telemetry.addData(
                "Fault Reason", turret.getFaultReason()
        );

        telemetry.update();
    }

    @Override
    public void stop() {

        if (turret != null) {
            turret.stop();
        }
    }
}

package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;

@TeleOp(name = "CALIBRACION LIMITES TORRETA")
public class TurretLimitCalibration extends OpMode {

    private DcMotorEx motor;
    private DcMotorEx encoder;

    private int zeroTicks = 0;
    private double minRecordedDeg = 0.0;
    private double maxRecordedDeg = 0.0;

    private boolean lastA = false;
    private boolean lastX = false;

    // Potencia máxima baja para no romper cables ni engranes al probar topes
    private static final double SAFE_MAX_POWER = 0.25;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "turret");
        encoder = hardwareMap.get(DcMotorEx.class, "mfl");

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);

        zeroTicks = encoder.getCurrentPosition();
        minRecordedDeg = 0.0;
        maxRecordedDeg = 0.0;

        telemetry.addLine("=== CALIBRACION DE LIMITES MECANICOS ===");
        telemetry.addLine("1. Pon la torreta en su CENTRO (0 grados) antes de darle START.");
        telemetry.addLine("2. Mueve despacio con Joystick Izquierdo X.");
        telemetry.addLine("3. Boton A: Marcar posicion actual como 0 (CERO).");
        telemetry.addLine("4. Boton X: Reiniciar min/max registrados.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Reiniciar el 0° en la posición actual
        if (gamepad1.a && !lastA) {
            zeroTicks = encoder.getCurrentPosition();
            minRecordedDeg = 0.0;
            maxRecordedDeg = 0.0;
        }
        lastA = gamepad1.a;

        // Reiniciar máximos y mínimos sin mover el 0°
        if (gamepad1.x && !lastX) {
            double current = getCurrentAngle();
            minRecordedDeg = current;
            maxRecordedDeg = current;
        }
        lastX = gamepad1.x;

        // Control manual suave (con zona muerta)
        double stick = -gamepad1.left_stick_x;
        double power = 0.0;
        if (Math.abs(stick) > 0.08) {
            power = Range.clip(stick * SAFE_MAX_POWER, -SAFE_MAX_POWER, SAFE_MAX_POWER);
        }
        motor.setPower(power);

        // Leer ángulo actual y actualizar récords
        double currentDeg = getCurrentAngle();
        int rawTicks = encoder.getCurrentPosition() - zeroTicks;

        if (currentDeg < minRecordedDeg) minRecordedDeg = currentDeg;
        if (currentDeg > maxRecordedDeg) maxRecordedDeg = currentDeg;

        double totalSpan = maxRecordedDeg - minRecordedDeg;

        // Sugerencia con margen de seguridad de 5 grados por lado
        double safeMin = minRecordedDeg + 5.0;
        double safeMax = maxRecordedDeg - 5.0;

        telemetry.addLine("=== LECTURAS EN TIEMPO REAL ===");
        telemetry.addData("Angulo Actual", "%.2f deg", currentDeg);
        telemetry.addData("Ticks desde Cero", rawTicks);
        telemetry.addData("Potencia Aplicada", "%.2f", power);

        telemetry.addLine("\n=== LIMITES DETECTADOS ===");
        telemetry.addData("Minimo alcanzado (Izq)", "%.2f deg", minRecordedDeg);
        telemetry.addData("Maximo alcanzado (Der)", "%.2f deg", maxRecordedDeg);
        telemetry.addData("Recorrido Total", "%.2f deg", totalSpan);

        telemetry.addLine("\n=== VALORES PARA TurretSubsystemV4 ===");
        telemetry.addData("MIN_ANGLE_DEG sugerido", "%.1f", safeMin);
        telemetry.addData("MAX_ANGLE_DEG sugerido", "%.1f", safeMax);
        telemetry.addData("Soporta Unwrap 360?", (safeMax - safeMin) >= 360.0 ? "SI (Sin zona ciega)" : "NO (Faltan " + String.format("%.1f", 360.0 - (safeMax - safeMin)) + " deg)");
        telemetry.update();
    }

    private double getCurrentAngle() {
        double ticks = encoder.getCurrentPosition() - zeroTicks;
        double ticksPerDegree = (TurretSubsystemV4.TICKS_PER_REV * TurretSubsystemV4.GEAR_RATIO) / 360.0;
        return -TurretSubsystemV4.ENCODER_SIGN * (ticks / ticksPerDegree);
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }
}
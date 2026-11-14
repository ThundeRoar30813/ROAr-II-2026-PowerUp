package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Calibracion Rampa", group = "FTC")
public class rampa extends OpMode {

    private CRServo rampap;
    private DcMotor Encoramp;

    // Constantes de conversion (REV Encoder 8192 ticks + engranes 12T:26T)
    private static final double TICKS_POR_REV_ENCODER = 8192.0;
    private static final double DIENTES_ENCODER = 12.0;
    private static final double DIENTES_RAMPA = 26.0;
    private static final double TICKS_POR_GRADO_RAMPA =
            (TICKS_POR_REV_ENCODER * (DIENTES_RAMPA / DIENTES_ENCODER)) / 360.0;

    // POSICIONES OBJETIVO CALIBRADAS
    private static final int TICKS_ARRIBA = 0;
    private static final int TICKS_OBJETIVO = 6184; // Tu medida exacta (135.57 deg)

    // PARAMETROS DEL CONTROL AUTOMATICO (Proporcional)
    private static final double KP = 0.0008;        // Suaviza la llegada para no pasarse
    private static final int TOLERANCIA_TICKS = 35; // Margen de error permitido (~0.7 grados)
    private static final double MIN_POWER = 0.15;   // Potencia minima para vencer friccion
    private static final double MAX_POWER = 0.75;   // Velocidad maxima del servo continuo

    private int targetTicks = TICKS_ARRIBA;
    private boolean modoAutomatico = false;

    private int offsetTicks = 0;
    private boolean lastBtnA = false;
    private boolean lastBtnB = false;
    private boolean lastBtnY = false;

    @Override
    public void init() {
        rampap = hardwareMap.get(CRServo.class, "rampa");
        rampap.setDirection(DcMotorSimple.Direction.FORWARD);
        rampap.setPower(0.0);

        Encoramp = hardwareMap.get(DcMotor.class, "mbl");
        Encoramp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Encoramp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("CONTROL DE RAMPA LISTO");
        telemetry.addLine("A: Ir a 0 ticks (Arriba)");
        telemetry.addLine("B: Ir a 6684 ticks (Objetivo)");
        telemetry.addLine("Y: Fijar posicion actual como 0");
        telemetry.addLine("D-Pad Arriba/Abajo: Ajuste manual");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. LEER ENCODER
        int rawTicks = Encoramp.getCurrentPosition();
        int ticksActuales = rawTicks - offsetTicks;
        double gradosActuales = ticksActuales / TICKS_POR_GRADO_RAMPA;

        // 2. BOTONES DE OBJETIVO
        // Boton A -> Regresar a 0 (Arriba)
        if (gamepad1.a && !lastBtnA) {
            targetTicks = TICKS_ARRIBA;
            modoAutomatico = true;
        }
        lastBtnA = gamepad1.a;

        // Boton B -> Ir al objetivo (6684 ticks)
        if (gamepad1.b && !lastBtnB) {
            targetTicks = TICKS_OBJETIVO;
            modoAutomatico = true;
        }
        lastBtnB = gamepad1.b;

        // Boton Y -> Reiniciar cero en la posicion actual
        if (gamepad1.y && !lastBtnY) {
            offsetTicks = rawTicks;
            targetTicks = TICKS_ARRIBA;
            modoAutomatico = false;
        }
        lastBtnY = gamepad1.y;

        // 3. CALCULO DE POTENCIA (MANUAL O AUTOMATICO)
        double power = 0.0;
        int error = targetTicks - ticksActuales;

        if (gamepad1.dpad_down) {
            modoAutomatico = false;
            power = 0.35;
        } else if (gamepad1.dpad_up) {
            modoAutomatico = false;
            power = -0.35;
        } else if (modoAutomatico) {
            if (Math.abs(error) <= TOLERANCIA_TICKS) {
                // Ya llego al objetivo: frena el servo continuo
                power = 0.0;
            } else {
                // Control proporcional: baja velocidad conforme se acerca al objetivo
                double rawPower = error * KP;
                double signo = Math.signum(rawPower);
                double magnitud = Range.clip(Math.abs(rawPower), MIN_POWER, MAX_POWER);
                power = signo * magnitud;
            }
        }

        rampap.setPower(power);

        // 4. TELEMETRIA
        telemetry.addData("Modo", modoAutomatico ? "AUTOMATICO" : "MANUAL");
        telemetry.addData("Objetivo (ticks)", targetTicks);
        telemetry.addData("Actual (ticks)", ticksActuales);
        telemetry.addData("Error (ticks)", error);
        telemetry.addData("Angulo actual", "%.2f deg", gradosActuales);
        telemetry.addData("Potencia CRServo", "%.2f", power);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (rampap != null) {
            rampap.setPower(0.0);
        }
    }
}
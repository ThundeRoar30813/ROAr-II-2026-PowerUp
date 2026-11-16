package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class RampaSubsystem {

    private CRServo rampaServo;
    private DcMotor encoderMotor; // Usando el puerto mbl como encoder

    // Constantes de conversión y posiciones que ya probamos y calibramos
    public static final double TICKS_POR_REV_ENCODER = 8192.0;
    public static final double DIENTES_ENCODER = 12.0;
    public static final double DIENTES_RAMPA = 26.0;
    public static final double TICKS_POR_GRADO_RAMPA = (TICKS_POR_REV_ENCODER * (DIENTES_RAMPA / DIENTES_ENCODER)) / 360.0;

    public static final int POSICION_ARRIBA = 0;
    public static final int POSICION_OBJETIVO = 6484; // 135.57 grados

    public static final int TOLERANCIA_TICKS = 35;
    public static final double KP = 0.0008;
    public static final double MIN_POWER = 0.15;
    public static final double MAX_POWER = 0.75;

    private int targetTicks = POSICION_ARRIBA;
    private int offsetTicks = 0;
    private boolean fault = false;

    public RampaSubsystem(HardwareMap hardwareMap) {
        try {
            rampaServo = hardwareMap.get(CRServo.class, "rampa");
            rampaServo.setDirection(DcMotorSimple.Direction.FORWARD);
            rampaServo.setPower(0.0);

            encoderMotor = hardwareMap.get(DcMotor.class, "mbl");
            encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            fault = true;
        }
    }

    public void setTargetTicks(int ticks) {
        this.targetTicks = ticks;
    }

    public int getTicksActuales() {
        if (encoderMotor == null) return 0;
        return encoderMotor.getCurrentPosition() - offsetTicks;
    }

    public void resetCero() {
        if (encoderMotor != null) {
            offsetTicks = encoderMotor.getCurrentPosition();
            targetTicks = POSICION_ARRIBA;
        }
    }

    public boolean isReady() {
        int error = targetTicks - getTicksActuales();
        return Math.abs(error) <= TOLERANCIA_TICKS;
    }

    public boolean hasFault() {
        return fault;
    }

    public void aplicarControl() {
        if (rampaServo == null || fault) return;

        int error = targetTicks - getTicksActuales();
        double power = 0.0;

        if (Math.abs(error) <= TOLERANCIA_TICKS) {
            power = 0.0;
        } else {
            double rawPower = error * KP;
            double signo = Math.signum(rawPower);
            double magnitud = Range.clip(Math.abs(rawPower), MIN_POWER, MAX_POWER);
            power = signo * magnitud;
        }

        rampaServo.setPower(power);
    }

    public void stop() {
        if (rampaServo != null) {
            rampaServo.setPower(0.0);
        }
    }

    public String getStatus() {
        if (fault) return "FAULT";
        return isReady() ? "LISTO" : "MOVIENDO";
    }
}
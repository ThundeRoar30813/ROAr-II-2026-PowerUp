package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class RampaSubsystem {

    public static final double TICKS_POR_REV_ENCODER = 8192.0;
    public static final double DIENTES_ENCODER = 12.0;
    public static final double DIENTES_RAMPA = 26.0;
    public static final double TICKS_POR_GRADO_RAMPA =
            (TICKS_POR_REV_ENCODER * (DIENTES_RAMPA / DIENTES_ENCODER)) / 360.0;

    public static int TICKS_ARRIBA = 0;
    public static int TICKS_OBJETIVO = 6484;

    public static double KP = 0.0008;
    public static int TOLERANCIA_TICKS = 35;
    public static double MIN_POWER = 0.15;
    public static double MAX_POWER = 0.75;
    public static double MOVE_TIMEOUT_S = 3.0;

    private final CRServo rampap;
    private final DcMotor encoramp;

    private int targetTicks = TICKS_ARRIBA;
    private int offsetTicks = 0;

    private boolean modoAutomatico = false;
    private double manualPower = 0.0;
    private double currentPower = 0.0;

    private boolean fault = false;
    private String status = "INIT";
    private long moveStartNs = 0;

    public RampaSubsystem(HardwareMap hardwareMap) {
        rampap = hardwareMap.get(CRServo.class, "rampa");
        rampap.setDirection(DcMotorSimple.Direction.FORWARD);
        rampap.setPower(0.0);

        encoramp = hardwareMap.get(DcMotor.class, "mbl");
        encoramp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoramp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void irArriba() { setTargetTicks(TICKS_ARRIBA); }
    public void irAbajo() { setTargetTicks(TICKS_OBJETIVO); }

    public void setTargetTicks(int target) {
        if (fault) return;
        if (!modoAutomatico || targetTicks != target) {
            moveStartNs = System.nanoTime();
        }
        targetTicks = target;
        modoAutomatico = true;
        manualPower = 0.0;
    }

    public void setManualPower(double power) {
        modoAutomatico = false;
        manualPower = Range.clip(power, -1.0, 1.0);
    }

    public void zeroHere() {
        offsetTicks = encoramp.getCurrentPosition();
        targetTicks = TICKS_ARRIBA;
        modoAutomatico = false;
        manualPower = 0.0;
        clearFault();
    }

    public void clearFault() {
        fault = false;
        status = "OK";
        moveStartNs = System.nanoTime();
    }

    public void stop() {
        modoAutomatico = false;
        manualPower = 0.0;
        currentPower = 0.0;
        rampap.setPower(0.0);
        status = "STOPPED";
    }

    public void update() {
        if (fault) {
            currentPower = 0.0;
            rampap.setPower(0.0);
            return;
        }

        int ticksActuales = getTicks();
        int error = targetTicks - ticksActuales;

        if (modoAutomatico) {
            if (Math.abs(error) <= TOLERANCIA_TICKS) {
                currentPower = 0.0;
                status = "EN_POSICION";
                moveStartNs = System.nanoTime();
            } else {
                double elapsedS = (System.nanoTime() - moveStartNs) * 1e-9;
                if (elapsedS > MOVE_TIMEOUT_S) {
                    fault = true;
                    status = "TIMEOUT_ENCODER";
                    currentPower = 0.0;
                    rampap.setPower(0.0);
                    return;
                }

                double rawPower = error * KP;
                double signo = Math.signum(rawPower);
                double magnitud = Range.clip(Math.abs(rawPower), MIN_POWER, MAX_POWER);
                currentPower = signo * magnitud;
                status = "MOVIENDO";
            }
        } else {
            currentPower = manualPower;
            status = (Math.abs(manualPower) > 0.01) ? "MANUAL" : "IDLE";
        }

        rampap.setPower(currentPower);
    }

    public int getTicks() { return encoramp.getCurrentPosition() - offsetTicks; }
    public int getTargetTicks() { return targetTicks; }
    public int getErrorTicks() { return targetTicks - getTicks(); }
    public double getDegrees() { return getTicks() / TICKS_POR_GRADO_RAMPA; }
    public boolean isReady() {
        if (fault) return false;
        if (!modoAutomatico) return true;
        return Math.abs(getErrorTicks()) <= TOLERANCIA_TICKS;
    }
    public boolean hasFault() { return fault; }
    public String getStatus() { return status; }
    public double getPower() { return currentPower; }
}
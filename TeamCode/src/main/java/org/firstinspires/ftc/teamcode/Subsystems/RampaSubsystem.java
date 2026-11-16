
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RampaSubsystem {

    // ==========================================
    // CONFIGURACION
    // ==========================================

    public static String MOTOR_NAME = "rampa";

    // Valores provisionales: CALIBRAR.
    public static int UP_TICKS = 0;
    public static int DOWN_TICKS = 500;

    public static double MOTOR_POWER = 0.35;

    public static int POSITION_TOLERANCE = 20;

    public static double MOVE_TIMEOUT_S = 3.0;

    // ==========================================
    // HARDWARE
    // ==========================================

    private final DcMotorEx motor;

    // ==========================================
    // ESTADOS
    // ==========================================

    public enum State {
        IDLE,
        MOVING_UP,
        MOVING_DOWN,
        READY,
        FAULT
    }

    private State state = State.IDLE;

    private int targetTicks = 0;

    private boolean fault = false;
    private String faultReason = "NONE";

    private long movementStartNs = 0;

    // ==========================================
    // CONSTRUCTOR
    // ==========================================

    public RampaSubsystem(HardwareMap hardwareMap) {

        motor = hardwareMap.get(
                DcMotorEx.class,
                MOTOR_NAME
        );

        motor.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        motor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        // IMPORTANTE:
        // La rampa debe iniciar fisicamente
        // en su posicion superior.
        motor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        motor.setTargetPosition(UP_TICKS);

        motor.setMode(
                DcMotor.RunMode.RUN_TO_POSITION
        );

        motor.setPower(0);

        targetTicks = UP_TICKS;
        state = State.READY;
    }

    // ==========================================
    // MOVER A POSICION
    // ==========================================

    public void moveTo(int ticks) {

        if (fault) {
            return;
        }

        int min = Math.min(UP_TICKS, DOWN_TICKS);
        int max = Math.max(UP_TICKS, DOWN_TICKS);

        targetTicks = Math.max(
                min,
                Math.min(max, ticks)
        );

        motor.setTargetPosition(targetTicks);

        motor.setMode(
                DcMotor.RunMode.RUN_TO_POSITION
        );

        motor.setPower(
                Math.abs(MOTOR_POWER)
        );

        movementStartNs = System.nanoTime();

        if (targetTicks == DOWN_TICKS) {
            state = State.MOVING_DOWN;
        } else {
            state = State.MOVING_UP;
        }
    }

    // ==========================================
    // SUBIR
    // ==========================================

    public void subir() {
        moveTo(UP_TICKS);
    }

    // ==========================================
    // BAJAR
    // ==========================================

    public void bajar() {
        moveTo(DOWN_TICKS);
    }

    // ==========================================
    // UPDATE
    // ==========================================

    public void update() {

        if (fault) {
            motor.setPower(0);
            return;
        }

        if (state != State.MOVING_UP
                && state != State.MOVING_DOWN) {
            return;
        }

        int error = Math.abs(
                targetTicks - motor.getCurrentPosition()
        );

        if (error <= POSITION_TOLERANCE) {

            motor.setPower(0);
            state = State.READY;
            return;
        }

        double elapsed =
                (System.nanoTime() - movementStartNs)
                        * 1e-9;

        if (elapsed >= MOVE_TIMEOUT_S) {

            fault = true;
            faultReason = "MOVE_TIMEOUT";

            motor.setPower(0);
            state = State.FAULT;
        }
    }

    // ==========================================
    // GETTERS
    // ==========================================

    public boolean isReady() {

        return !fault
                && state == State.READY
                && Math.abs(
                targetTicks - motor.getCurrentPosition()
        ) <= POSITION_TOLERANCE;
    }

    public boolean hasFault() {
        return fault;
    }

    public int getTicks() {
        return motor.getCurrentPosition();
    }

    public int getTargetTicks() {
        return targetTicks;
    }

    public String getStatus() {

        if (fault) {
            return "FAULT: " + faultReason;
        }

        return state.name();
    }

    public String getFaultReason() {
        return faultReason;
    }

    // ==========================================
    // RESET FAULT
    // ==========================================

    public void clearFault() {

        motor.setPower(0);

        fault = false;
        faultReason = "NONE";

        targetTicks = motor.getCurrentPosition();

        state = State.READY;
    }

    // ==========================================
    // STOP
    // ==========================================

    public void stop() {

        motor.setPower(0);

        if (!fault) {
            state = State.IDLE;
        }
    }
}

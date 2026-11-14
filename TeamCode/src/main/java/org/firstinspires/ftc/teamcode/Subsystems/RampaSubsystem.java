
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class RampaSubsystem {

    public static final int ARRIBA_TICKS = 0;
    public static final int ABAJO_TICKS = 6184;

    // Ajustar si la lectura disminuye al bajar
    public static int ENCODER_SIGN = 1;

    // Ajustar si el servo gira al reves
    public static int SERVO_SIGN = 1;

    public static double KP = 0.0008;
    public static double MIN_POWER = 0.15;
    public static double MAX_POWER = 0.75;

    public static int TOLERANCE = 35;
    public static double TIMEOUT_S = 4.0;

    public static double STALL_TIMEOUT_S = 0.8;
    public static int MIN_PROGRESS_TICKS = 15;

    private final CRServo servo;
    private final DcMotorEx encoderPort;

    private final int zeroTicks;

    private int targetTicks = 0;
    private int lastProgressTicks = 0;

    private long startNs;
    private long progressNs;

    private boolean moving = false;
    private boolean fault = false;

    private String status = "IDLE";

    public RampaSubsystem(HardwareMap hw) {

        servo = hw.get(CRServo.class, "rampa");
        encoderPort = hw.get(DcMotorEx.class, "mbl");

        servo.setDirection(
                DcMotorSimple.Direction.FORWARD);

        servo.setPower(0);

        // NO resetear ni cambiar el modo de mbl
        zeroTicks = encoderPort.getCurrentPosition();

        lastProgressTicks = 0;
    }

    public int getTicks() {
        return ENCODER_SIGN *
                (encoderPort.getCurrentPosition()
                        - zeroTicks);
    }

    public int getTarget() {
        return targetTicks;
    }

    public int getError() {
        return targetTicks - getTicks();
    }

    public String getStatus() {
        return status;
    }

    public boolean hasFault() {
        return fault;
    }

    public boolean isMoving() {
        return moving;
    }

    public boolean isReady() {
        return !moving && !fault &&
                Math.abs(getError()) <= TOLERANCE;
    }

    public void subir() {
        moveTo(ARRIBA_TICKS);
    }

    public void bajar() {
        moveTo(ABAJO_TICKS);
    }

    public void moveTo(int ticks) {

        if (fault) return;

        targetTicks = Range.clip(
                ticks, ARRIBA_TICKS, ABAJO_TICKS);

        startNs = System.nanoTime();
        progressNs = startNs;
        lastProgressTicks = getTicks();

        moving = true;
        status = "MOVING";
    }

    public void update() {

        if (!moving || fault) {
            servo.setPower(0);
            return;
        }

        long now = System.nanoTime();

        double elapsed =
                (now - startNs) * 1e-9;

        int current = getTicks();
        int error = targetTicks - current;

        if (Math.abs(error) <= TOLERANCE) {
            moving = false;
            servo.setPower(0);
            status = "READY";
            return;
        }

        if (elapsed >= TIMEOUT_S) {
            fail("TIMEOUT");
            return;
        }

        if (Math.abs(current - lastProgressTicks)
                >= MIN_PROGRESS_TICKS) {

            lastProgressTicks = current;
            progressNs = now;
        }

        double stallTime =
                (now - progressNs) * 1e-9;

        if (stallTime >= STALL_TIMEOUT_S) {
            fail("NO_ENCODER_PROGRESS");
            return;
        }

        double rawPower = KP * error;

        double power = Range.clip(
                Math.abs(rawPower),
                MIN_POWER,
                MAX_POWER
        );

        servo.setPower(
                SERVO_SIGN *
                        Math.signum(rawPower) *
                        power
        );
    }

    private void fail(String reason) {
        servo.setPower(0);
        moving = false;
        fault = true;
        status = reason;
    }

    public void clearFault() {
        servo.setPower(0);
        moving = false;
        fault = false;
        status = "IDLE";
    }

    public void stop() {
        servo.setPower(0);
        moving = false;
        status = "STOPPED";
    }
}

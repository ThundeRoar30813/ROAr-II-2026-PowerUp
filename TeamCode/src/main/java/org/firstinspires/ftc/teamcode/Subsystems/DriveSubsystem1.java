package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class DriveSubsystem1 {

    private DcMotorEx fl;
    private DcMotorEx fr;
    private DcMotorEx bl;
    private DcMotorEx br;

    public void init(HardwareMap hw) {

        fl = hw.get(DcMotorEx.class, "mfl");
        fr = hw.get(DcMotorEx.class, "mfr");
        bl = hw.get(DcMotorEx.class, "mbl");
        br = hw.get(DcMotorEx.class, "mbr");

        // Direcciones indicadas en tu código
        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);

        // Frenado al soltar los joysticks
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // El control del chasis se hace por potencia
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stop();
    }

    public void drive(double forward, double strafe, double turn) {

        // Mezcla mecanum
        double flPower = forward + strafe + turn;
        double frPower = forward - strafe - turn;
        double blPower = forward - strafe + turn;
        double brPower = forward + strafe - turn;

        // Normalizar para conservar las proporciones
        double max = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(flPower), Math.abs(frPower)),
                        Math.max(Math.abs(blPower), Math.abs(brPower))
                )
        );

        setMotorPowers(
                flPower / max,
                frPower / max,
                blPower / max,
                brPower / max
        );
    }

    public void setMotorPowers(
            double flPower,
            double frPower,
            double blPower,
            double brPower
    ) {
        fl.setPower(Range.clip(flPower, -1.0, 1.0));
        fr.setPower(Range.clip(frPower, -1.0, 1.0));
        bl.setPower(Range.clip(blPower, -1.0, 1.0));
        br.setPower(Range.clip(brPower, -1.0, 1.0));
    }

    public void stop() {
        if (fl != null && fr != null && bl != null && br != null) {
            setMotorPowers(0, 0, 0, 0);
        }
    }

    public double getFLPower() { return fl.getPower(); }
    public double getFRPower() { return fr.getPower(); }
    public double getBLPower() { return bl.getPower(); }
    public double getBRPower() { return br.getPower(); }

    public double getFLVelocity() { return fl.getVelocity(); }
    public double getFRVelocity() { return fr.getVelocity(); }
    public double getBLVelocity() { return bl.getVelocity(); }
    public double getBRVelocity() { return br.getVelocity(); }
}

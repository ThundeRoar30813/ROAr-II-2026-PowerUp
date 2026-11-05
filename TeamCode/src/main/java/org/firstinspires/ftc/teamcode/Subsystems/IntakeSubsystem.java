
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem {

    public DcMotor intake1;
    public DcMotor intake2;

    public Servo gateServo;

    public RevColorSensorV3 bottomSensor;
    public RevColorSensorV3 topSensor;

    public static double INTAKE_POWER = 1.0;
    public static double INDEXER_POWER = -0.50;

    public static double FEED_INTAKE = 1.0;
    public static double FEED_INDEXER = -1.0;

    public static double GATE_CLOSED = 0.20;
    public static double GATE_OPEN = 0.75;

    public static double BOTTOM_DISTANCE = 4.7;
    public static double TOP_DISTANCE = 4.7;

    public void init(HardwareMap hw) {

        intake1 = hw.get(DcMotor.class, "mIntake1");
        intake2 = hw.get(DcMotor.class, "mIntake2");

        gateServo = hw.get(Servo.class, "gateServo");

        bottomSensor = hw.get(RevColorSensorV3.class, "downSensor");

        topSensor = hw.get(RevColorSensorV3.class, "topSensor");

        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        closeGate();
        stop();
    }

    // =========================
    // SENSORES
    // =========================

    public boolean hasBottomPiece() {
        return getBottomDistance() < BOTTOM_DISTANCE;
    }

    public boolean hasTopPiece() {
        return getTopDistance() < TOP_DISTANCE;
    }

    public double getBottomDistance() {
        return bottomSensor.getDistance(DistanceUnit.CM);
    }

    public double getTopDistance() {
        return topSensor.getDistance(DistanceUnit.CM);
    }

    // =========================
    // SERVO
    // =========================

    public void openGate() {
        gateServo.setPosition(GATE_OPEN);
    }

    public void closeGate() {
        gateServo.setPosition(GATE_CLOSED);
    }

    // =========================
    // MOTORES
    // =========================

    public void set(double intake, double indexer) {

        intake1.setPower(Range.clip(intake, -1, 1));
        intake2.setPower(Range.clip(indexer, -1, 1));
    }

    public void collect() {
        set(INTAKE_POWER, INDEXER_POWER);
    }

    public void collectIntakeOnly() {
        set(INTAKE_POWER, 0);
    }

    public void feedShooter() {
        set(FEED_INTAKE, FEED_INDEXER);
    }

    public void stop() {
        set(0, 0);
    }
}


package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodSubsystem {

    private static final double[] HOOD_POSITIONS = {
            0.4444,
            0.5560,
            0.6667,
            0.7778,
            0.8890
    };

    private final Servo hood;

    // Posicion inicial: 1 de 5
    private int hoodIndex = 0;

    public HoodSubsystem(HardwareMap hardwareMap) {

        hood = hardwareMap.tryGet(Servo.class, "hood");

        if (hood != null) {
            hood.setDirection(Servo.Direction.FORWARD);
            hood.setPosition(HOOD_POSITIONS[hoodIndex]);
        }
    }

    public void nextPosition() {
        setIndex(hoodIndex + 1);
    }

    public void previousPosition() {
        setIndex(hoodIndex - 1);
    }

    public void setIndex(int index) {

        hoodIndex = Math.max(
                0,
                Math.min(index, HOOD_POSITIONS.length - 1)
        );

        if (hood != null) {
            hood.setPosition(HOOD_POSITIONS[hoodIndex]);
        }
    }

    public int getPositionNumber() {
        return hoodIndex + 1;
    }

    public double getPosition() {
        return HOOD_POSITIONS[hoodIndex];
    }

    public boolean isConnected() {
        return hood != null;
    }
}

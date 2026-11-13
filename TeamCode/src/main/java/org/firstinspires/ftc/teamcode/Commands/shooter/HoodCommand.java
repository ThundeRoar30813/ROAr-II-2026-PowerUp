
package org.firstinspires.ftc.teamcode.Commands.shooter;

import org.firstinspires.ftc.teamcode.Subsystems.HoodSubsystem;

public class HoodCommand {

    private final HoodSubsystem hood;

    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastLeft = false;
    private boolean lastRight = false;

    public HoodCommand(HoodSubsystem hood) {
        this.hood = hood;
    }

    public void execute(
            boolean up,
            boolean down,
            boolean left,
            boolean right) {

        if (up && !lastUp) {
            hood.nextPosition();
        }

        if (down && !lastDown) {
            hood.previousPosition();
        }

        if (left && !lastLeft) {
            hood.setIndex(0);
        }

        if (right && !lastRight) {
            hood.setIndex(4);
        }

        lastUp = up;
        lastDown = down;
        lastLeft = left;
        lastRight = right;
    }
}

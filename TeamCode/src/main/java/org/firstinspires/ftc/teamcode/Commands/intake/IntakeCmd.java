package org.firstinspires.ftc.teamcode.Commands.intake;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

public class IntakeCmd {

    private final IntakeSubsystem intake;

    private final boolean[] inventory = new boolean[4];

    private int ballCount = 0;

    private boolean collecting = false;
    private boolean shooting = false;

    private boolean previousBottom;
    private boolean previousTop;

    private final ElapsedTime bottomTimer = new ElapsedTime();
    private final ElapsedTime topTimer = new ElapsedTime();
    private final ElapsedTime gateTimer = new ElapsedTime();

    public static double COUNT_DELAY = 0.15;
    public static double GATE_OPEN_DELAY = 0.20;

    private boolean previousShooting = false;

    public IntakeCmd(IntakeSubsystem intake) {

        this.intake = intake;

        previousBottom = intake.hasBottomPiece();
        previousTop = intake.hasTopPiece();

        bottomTimer.reset();
        topTimer.reset();
        gateTimer.reset();
    }

    // =========================
    // CONTROLES
    // =========================

    public void toggleCollecting() {
        collecting = !collecting;
    }

    public void setShooting(boolean shooting) {
        this.shooting = shooting;
    }

    public void emergencyStop() {
        collecting = false;
        shooting = false;
        previousShooting = false;

        intake.closeGate();
        intake.stop();
    }

    // =========================
    // INVENTARIO
    // =========================

    private void addBall() {

        if (ballCount >= 4) return;

        inventory[ballCount] = true;
        ballCount++;
    }

    private void removeBall() {

        if (ballCount <= 0) return;

        ballCount--;
        inventory[ballCount] = false;
    }

    public void resetInventory() {

        ballCount = 0;

        for (int i = 0; i < 4; i++) {
            inventory[i] = false;
        }

        previousBottom = intake.hasBottomPiece();
        previousTop = intake.hasTopPiece();

        bottomTimer.reset();
        topTimer.reset();
    }

    public int getBallCount() {
        return ballCount;
    }

    public boolean[] getInventory() {
        return inventory.clone();
    }

    public boolean isFull() {
        return ballCount >= 4;
    }

    public boolean isEmpty() {
        return ballCount == 0;
    }

    public boolean isCollecting() {
        return collecting;
    }

    public boolean isShooting() {
        return shooting;
    }

    // =========================
    // UPDATE PRINCIPAL
    // =========================

    public void update() {

        boolean bottom = intake.hasBottomPiece();
        boolean top = intake.hasTopPiece();

        // Detectar entrada de una pelota nueva
        if (collecting && !shooting
                && bottom && !previousBottom
                && bottomTimer.seconds() >= COUNT_DELAY) {

            addBall();
            bottomTimer.reset();
        }

        // Detectar salida del sensor superior
        if (shooting
                && !top && previousTop
                && topTimer.seconds() >= COUNT_DELAY) {

            removeBall();
            topTimer.reset();
        }

        previousBottom = bottom;
        previousTop = top;

        // =========================
        // MODO DISPARO
        // =========================

        if (shooting) {

            if (!previousShooting) {
                gateTimer.reset();
            }

            previousShooting = true;
            intake.openGate();

            if (isEmpty()) {
                intake.stop();
            } else if (gateTimer.seconds() >= GATE_OPEN_DELAY) {
                intake.feedShooter();
            } else {
                intake.stop();
            }

            return;
        }

        previousShooting = false;

        // =========================
        // MODO RECOLECCION
        // =========================

        intake.closeGate();

        if (!collecting || isFull()) {
            intake.stop();
            return;
        }

        if (top) {
            // Detener indexer, mantener intake
            intake.collectIntakeOnly();
        } else {
            intake.collect();
        }
    }
}

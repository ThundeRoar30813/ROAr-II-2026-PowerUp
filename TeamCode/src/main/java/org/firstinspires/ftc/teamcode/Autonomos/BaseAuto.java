package org.firstinspires.ftc.teamcode.Autonomos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Commands.rampa.RampaCommand;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV77;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV5;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.RampaSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

public abstract class BaseAuto extends OpMode {

    // =========================================
    // AJUSTES GLOBALES
    // =========================================
    public static double SHOOTER_TPS = 1200;
    public static double SHOOTER_TOLERANCE = 60;
    public static double AIM_TOLERANCE = 2.5;
    public static double READY_STABLE_S = 0.20;
    public static double SHOOT_WAIT_MAX_S = 3.0;
    public static double PATH_TIMEOUT_S = 12.0;

    public static double FIELD_MID_Y = 72.0;
    public static double TAG_HYSTERESIS = 1.5;
    public static int INITIAL_TAG = 37;
    public static int OPPOSITE_TAG = 30;

    public static boolean ENABLE_RAMP = false;

    // =========================================
    // SUBSYSTEMS
    // =========================================
    protected Follower follower;
    protected DriveSubsystem drive;
    protected TurretSubsystemV4 turret;
    protected TurretVisionV5 vision;
    protected TurretAimControllerV77 aim;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected RampaSubsystem rampa;
    protected RampaCommand rampaCmd;

    // =========================================
    // SISTEMA DE PASOS (DSL)
    // =========================================
    protected interface AutoStep {
        void start();
        boolean isDone();
        String getName();
    }

    private List<AutoStep> routine;
    private int stepIndex = 0;
    private boolean stepStarted = false;
    private boolean finished = false;
    private String faultReason = "NONE";

    // Estados internos de mecanismos
    private boolean collecting = false;
    private boolean feeding = false;
    private int selectedTag = INITIAL_TAG;
    private boolean oppositeSide = false;

    // =========================================
    // METODOS ABSTRACTOS PARA CADA AUTONOMO
    // =========================================
    protected abstract Pose getStartPose();
    protected abstract List<AutoStep> buildRoutine();

    // =========================================
    // COMANDOS LEGIBLES PARA ARMAR RUTINAS
    // =========================================

    /** Mueve el robot siguiendo una ruta de Pedro Pathing */
    protected AutoStep irA(Path path, String nombre) {
        return new AutoStep() {
            private long startNs;
            private boolean observedBusy;

            @Override
            public void start() {
                startNs = System.nanoTime();
                observedBusy = false;
                schedule(follow(follower, path));
            }

            @Override
            public boolean isDone() {
                if (follower.isBusy()) {
                    observedBusy = true;
                    if (secondsSince(startNs) > PATH_TIMEOUT_S) {
                        triggerFault("TIMEOUT_" + nombre);
                    }
                    return false;
                }
                return observedBusy;
            }

            @Override
            public String getName() { return "IR_A: " + nombre; }
        };
    }

    /** Espera a que torreta y shooter estén listos y alimenta las pelotas */
    protected AutoStep disparar(double duracionFeedSeg) {
        return new AutoStep() {
            private long stepStartNs;
            private long readyStartNs;
            private long feedStartNs;
            private boolean isFeeding;

            @Override
            public void start() {
                stepStartNs = System.nanoTime();
                readyStartNs = 0;
                feedStartNs = 0;
                isFeeding = false;
            }

            @Override
            public boolean isDone() {
                if (!isFeeding) {
                    if (mechanismsReady()) {
                        if (readyStartNs == 0) readyStartNs = System.nanoTime();
                        if (secondsSince(readyStartNs) >= READY_STABLE_S) {
                            isFeeding = true;
                            feeding = true;
                            feedStartNs = System.nanoTime();
                        }
                    } else {
                        readyStartNs = 0;
                    }

                    if (secondsSince(stepStartNs) > SHOOT_WAIT_MAX_S) {
                        triggerFault("SHOOT_NOT_READY");
                    }
                    return false;
                } else {
                    if (secondsSince(feedStartNs) >= duracionFeedSeg) {
                        feeding = false;
                        return true;
                    }
                    return false;
                }
            }

            @Override
            public String getName() { return isFeeding ? "DISPARANDO" : "APUNTANDO"; }
        };
    }

    /** Enciende el intake y baja la rampa (si está habilitada) */
    protected AutoStep encenderIntake() {
        return new AutoStep() {
            @Override
            public void start() {
                collecting = true;
                if (!intake.isAutoIntakeEnabled()) intake.toggleAutoIntake();
                if (ENABLE_RAMP) rampaCmd.bajar();
            }
            @Override
            public boolean isDone() { return true; }
            @Override
            public String getName() { return "INTAKE_ON"; }
        };
    }

    /** Apaga el intake y sube la rampa (si está habilitada) */
    protected AutoStep apagarIntake() {
        return new AutoStep() {
            @Override
            public void start() {
                collecting = false;
                if (intake.isAutoIntakeEnabled()) intake.toggleAutoIntake();
                if (ENABLE_RAMP) rampaCmd.subir();
            }
            @Override
            public boolean isDone() { return true; }
            @Override
            public String getName() { return "INTAKE_OFF"; }
        };
    }

    /** Pausa controlada en segundos */
    protected AutoStep esperar(double segundos) {
        return new AutoStep() {
            private long startNs;
            @Override
            public void start() { startNs = System.nanoTime(); }
            @Override
            public boolean isDone() { return secondsSince(startNs) >= segundos; }
            @Override
            public String getName() { return "ESPERAR_" + segundos + "s"; }
        };
    }

    // =========================================
    // CICLO DE VIDA OPMODE
    // =========================================
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        drive = new DriveSubsystem(follower);
        drive.setPose(getStartPose());

        turret = new TurretSubsystemV4(hardwareMap);
        vision = new TurretVisionV5(hardwareMap);
        aim = new TurretAimControllerV77(drive, turret, vision);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        rampa = new RampaSubsystem(hardwareMap);
        rampaCmd = new RampaCommand(rampa);

        aim.setMode(TurretAimControllerV77.Mode.COMBINED);
        turret.maxPower = 1.0;

        shooter.setTargetTPS(SHOOTER_TPS);
        shooter.setEnabled(false);

        routine = buildRoutine();

        telemetry.addLine("LISTO PARA INICIAR");
        telemetry.addData("Pasos cargados", routine.size());
        telemetry.update();
    }

    @Override
    public void start() {
        oppositeSide = false;
        selectedTag = INITIAL_TAG;
        vision.start();
        updateSelectedTag();

        aim.enable();
        shooter.setTargetTPS(SHOOTER_TPS);
        shooter.setEnabled(true);

        stepIndex = 0;
        stepStarted = false;
        finished = false;
    }

    @Override
    public void loop() {
        drive.update();
        Scheduler.execute();

        updateSelectedTag();
        vision.update();

        if (!finished && faultReason.equals("NONE")) {
            aim.update();
        }

        shooter.update();
        if (ENABLE_RAMP) rampaCmd.execute();
        intake.update(false, false, feeding, false);

        // Revisión de fallos de hardware
        if (turret.isFault()) triggerFault("TURRET_FAULT");
        if (ENABLE_RAMP && rampa.hasFault()) triggerFault("RAMP_FAULT");

        // Ejecución secuencial de pasos
        if (!finished && faultReason.equals("NONE")) {
            if (stepIndex < routine.size()) {
                AutoStep current = routine.get(stepIndex);
                if (!stepStarted) {
                    current.start();
                    stepStarted = true;
                }
                if (current.isDone()) {
                    stepIndex++;
                    stepStarted = false;
                }
            } else {
                finished = true;
                stopAllMechanisms();
            }
        }

        // Telemetría limpia
        String currentStepName = (stepIndex < routine.size()) ? routine.get(stepIndex).getName() : "COMPLETADO";
        telemetry.addData("Paso", (stepIndex + 1) + "/" + routine.size() + " -> " + currentStepName);
        telemetry.addData("Pos", String.format("(%.1f, %.1f) %.1f°", drive.getX(), drive.getY(), drive.getHeadingDegrees()));
        telemetry.addData("Tag / Aim", selectedTag + " / " + aim.getStatus());
        telemetry.addData("Shooter TPS", shooter.getActualTPS());
        telemetry.addData("Pelotas", intake.getBallCount());
        telemetry.addData("Fallo", faultReason);
        telemetry.update();
    }

    @Override
    public void stop() {
        stopAllMechanisms();
        vision.stop();
        drive.stop();
    }

    // =========================================
    // UTILIDADES Y SEGURIDAD
    // =========================================
    private void updateSelectedTag() {
        double y = drive.getY();
        if (!Double.isFinite(y)) return;

        if (oppositeSide) {
            if (y < FIELD_MID_Y - TAG_HYSTERESIS) oppositeSide = false;
        } else {
            if (y > FIELD_MID_Y + TAG_HYSTERESIS) oppositeSide = true;
        }

        selectedTag = oppositeSide ? OPPOSITE_TAG : INITIAL_TAG;
        vision.setSelectedTag(selectedTag);
    }

    private boolean mechanismsReady() {
        boolean shooterReady = Math.abs(shooter.getActualTPS() - shooter.getTargetTPS()) <= SHOOTER_TOLERANCE;
        boolean turretReady = turret.isReadyToShoot(AIM_TOLERANCE) && !turret.isFault() && aim.isEnabled();
        boolean rampReady = !ENABLE_RAMP || rampa.isReady();
        boolean targetKnown = selectedTag == INITIAL_TAG || selectedTag == OPPOSITE_TAG;
        return shooterReady && turretReady && rampReady && targetKnown;
    }

    private void triggerFault(String reason) {
        if (!faultReason.equals("NONE")) return;
        faultReason = reason;
        stopAllMechanisms();
        follower.manual(0, 0, 0);
    }

    private void stopAllMechanisms() {
        feeding = false;
        collecting = false;
        shooter.setEnabled(false);
        intake.stop();
        if (ENABLE_RAMP) rampaCmd.stop();
        aim.disable();
        turret.stop();
        Scheduler.reset();
    }

    private double secondsSince(long ns) {
        return (System.nanoTime() - ns) * 1e-9;
    }
}
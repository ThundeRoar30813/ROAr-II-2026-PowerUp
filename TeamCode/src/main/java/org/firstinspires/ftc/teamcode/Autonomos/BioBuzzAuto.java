
package org.firstinspires.ftc.teamcode.OpModes.Autonomos;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystemV4;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.RampaSubsystem;

import org.firstinspires.ftc.teamcode.Commands.rampa.RampaCommand;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV5;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV77;

@Autonomous(
        name = "BIOBUZZ AUTO FINAL V1",
        group = "Pedro"
)
public class BioBuzzAuto extends OpMode {

    // =========================================
    // AJUSTES
    // =========================================

    public static double SHOOTER_TPS = 1200;
    public static double SHOOTER_TOLERANCE = 60;

    public static double AIM_TOLERANCE = 2.5;

    public static double READY_STABLE_S = 0.20;
    public static double SHOOT_WAIT_MAX_S = 3.0;

    public static double FEED_DURATION_S = 2.0;

    public static double PATH_TIMEOUT_S = 12.0;
    public static double PATH_POSITION_TOLERANCE = 3.0;

    // Cambiar a true solamente cuando se haya
    // validado el encoder y sentido de la rampa.
    public static boolean ENABLE_RAMP = false;

    // =========================================
    // SUBSYSTEMS
    // =========================================

    private Follower follower;
    private DriveSubsystem drive;

    private TurretSubsystemV4 turret;
    private TurretVisionV5 vision;
    private TurretAimControllerV77 aim;

    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;

    private RampaSubsystem rampa;
    private RampaCommand rampaCmd;

    // =========================================
    // POSES
    // =========================================

    private final PoseFactory pf =
            PoseFactory.degrees();

    private final Pose p0 =
            pf.of(44.5916, 9.0788, 90);

    private final Pose p1 =
            pf.of(44.6143, 15.2436, 90);

    private final Pose p2 =
            pf.of(17.8901, 46.9881, 180);

    private final Pose p3 =
            pf.of(9.9444, 46.7875, 180);

    private final Pose p4 =
            pf.of(17.4832, 47.0671, 180);

    private final Pose p5 =
            pf.of(47.38, 125.9494, 90);

    private final Pose p6 =
            pf.of(47.047, 131.2394, 90);

    private final Pose p7 =
            pf.of(47.4948, 127.4309, 90);

    private final Pose p8 =
            pf.of(9.8474, 120.0793, 90);

    private Path[] paths;

    private final Pose[] destinations = {
            p1, p2, p3, p4,
            p5, p6, p7, p8
    };

    // =========================================
    // ESTADOS
    // =========================================

    private enum State {
        DRIVE_TO_START,
        COLLECT_ONE,
        RETURN_ONE,
        WAIT_SHOT_ONE,
        FEED_ONE,
        COLLECT_TWO,
        RETURN_TWO,
        WAIT_SHOT_TWO,
        FEED_TWO,
        PARK,
        DONE,
        FAULT
    }

    private State state = State.DRIVE_TO_START;

    private int currentPath = -1;

    private boolean pathRunning = false;
    private boolean observedBusy = false;

    private boolean collecting = false;
    private boolean feeding = false;

    private boolean trackingStarted = false;

    private String faultReason = "NONE";

    private long stateStartNs;
    private long pathStartNs;
    private long readyStartNs;

    // =========================================
    // INICIALIZACION
    // =========================================

    @Override
    public void init() {

        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        // IMPORTANTE: MISMO FOLLOWER
        drive = new DriveSubsystem(follower);
        drive.setPose(p0);

        turret = new TurretSubsystemV4(hardwareMap);

        vision = new TurretVisionV5(hardwareMap);

        aim = new TurretAimControllerV77(
                drive,
                turret,
                vision
        );

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        rampa = new RampaSubsystem(hardwareMap);
        rampaCmd = new RampaCommand(rampa);

        aim.setMode(
                TurretAimControllerV77.Mode.COMBINED
        );

        turret.maxPower = 1.0;

        // =====================================
        // CONSTRUIR RUTAS
        // =====================================

        paths = new Path[] {

                Paths.line(p0, p1)
                        .constant(p1),

                Paths.curve(
                        p1,
                        pf.of(41.4224, 34.4952, 0),
                        pf.of(37.5983, 44.159, 0),
                        p2
                ).linear(p1, p2),

                Paths.line(p2, p3)
                        .constant(p3),

                Paths.line(p3, p4)
                        .constant(p4),

                Paths.curve(
                        p4,
                        pf.of(30.5411, 111.1691, 0),
                        p5
                ).linear(p4, p5),

                Paths.line(p5, p6)
                        .constant(p6),

                Paths.line(p6, p7)
                        .constant(p7),

                Paths.line(p7, p8)
                        .constant(p8)
        };

        shooter.setTargetTPS(SHOOTER_TPS);
        shooter.setEnabled(false);

        telemetry.addLine("BIOBUZZ AUTO INICIALIZADO");
        telemetry.addLine("Un Follower compartido");
        telemetry.addData("Rampa habilitada", ENABLE_RAMP);
        telemetry.update();
    }

    // =========================================
    // START
    // =========================================

    @Override
    public void start() {

        vision.start();

        // Inicializar tag antes de activar aim
        updateSelectedTag();

        aim.enable();
        trackingStarted = aim.isEnabled();

        shooter.setTargetTPS(SHOOTER_TPS);
        shooter.setEnabled(true);

        startPath(0);
        changeState(State.DRIVE_TO_START);
    }

    // =========================================
    // TEMPORIZADORES
    // =========================================

    private double secondsSince(long ns) {
        return (System.nanoTime() - ns) * 1e-9;
    }

    private double stateTime() {
        return secondsSince(stateStartNs);
    }

    private void changeState(State next) {
        state = next;
        stateStartNs = System.nanoTime();
        readyStartNs = 0;
    }

    // =========================================
    // PATH CONTROL
    // =========================================

    private void startPath(int index) {

        currentPath = index;

        pathRunning = true;
        observedBusy = false;

        pathStartNs = System.nanoTime();

        schedule(
                follow(follower, paths[index])
        );
    }

    private boolean pathFinished() {

        if (!pathRunning) return false;

        if (follower.isBusy()) {
            observedBusy = true;
            return false;
        }

        // Confirmar que el seguidor inicio
        if (!observedBusy) {
            return false;
        }

        Pose goal = destinations[currentPath];
        Pose actual = follower.pose();

        double dx = actual.x() - goal.x();
        double dy = actual.y() - goal.y();

        double distance = Math.hypot(dx, dy);

        if (distance > PATH_POSITION_TOLERANCE) {
            fail("PATH_END_POSITION_ERROR");
            return false;
        }

        pathRunning = false;
        return true;
    }

    private void checkPathTimeout() {

        if (pathRunning &&
                secondsSince(pathStartNs)
                        > PATH_TIMEOUT_S) {

            fail("PATH_TIMEOUT_" + currentPath);
        }
    }

    // =========================================
    // APRILTAG AUTOMATICO
    // =========================================

    private int selectedTag = 37;

    private void updateSelectedTag() {

        double x = drive.getX();
        double y = drive.getY();

        if (!Double.isFinite(x) ||
                !Double.isFinite(y)) {
            return;
        }

        if (y < 72.0) {

            selectedTag = x < 72.0 ? 37 : 35;

        } else {

            selectedTag = x < 72.0 ? 30 : 33;
        }

        vision.setSelectedTag(selectedTag);
    }

    // =========================================
    // INTAKE
    // =========================================

    private void startCollection() {

        collecting = true;

        if (!intake.isAutoIntakeEnabled()) {
            intake.toggleAutoIntake();
        }

        if (ENABLE_RAMP) {
            rampaCmd.bajar();
        }
    }

    private void stopCollection() {

        collecting = false;

        if (intake.isAutoIntakeEnabled()) {
            intake.toggleAutoIntake();
        }

        if (ENABLE_RAMP) {
            rampaCmd.subir();
        }
    }

    // =========================================
    // SHOOTER
    // =========================================

    private boolean mechanismsReady() {

        boolean shooterReady =
                Math.abs(
                        shooter.getActualTPS()
                                - shooter.getTargetTPS()
                ) <= SHOOTER_TOLERANCE;

        boolean turretReady =
                turret.isReadyToShoot(AIM_TOLERANCE)
                        && !turret.isFault()
                        && aim.isEnabled();

        boolean rampReady =
                !ENABLE_RAMP || rampa.isReady();

        // No disparar con un objetivo desconocido.
        boolean targetKnown =
                selectedTag == 30 ||
                        selectedTag == 33 ||
                        selectedTag == 35 ||
                        selectedTag == 37;

        return shooterReady &&
                turretReady &&
                rampReady &&
                targetKnown;
    }

    private boolean readyStable() {

        if (!mechanismsReady()) {
            readyStartNs = 0;
            return false;
        }

        if (readyStartNs == 0) {
            readyStartNs = System.nanoTime();
            return false;
        }

        return secondsSince(readyStartNs)
                >= READY_STABLE_S;
    }

    private void beginFeed() {
        feeding = true;
    }

    private void stopFeed() {
        feeding = false;
    }

    // =========================================
    // FALLOS
    // =========================================

    private void fail(String reason) {

        if (state == State.FAULT ||
                state == State.DONE) {
            return;
        }

        faultReason = reason;
        changeState(State.FAULT);
    }

    // =========================================
    // LOOP
    // =========================================

    @Override
    public void loop() {

        // Actualizar una sola vez el Follower
        drive.update();

        // Ejecutar comandos de rutas
        Scheduler.execute();

        // Seleccion de tag y vision
        updateSelectedTag();
        vision.update();

        // Actualizar tracking continuamente
        if (state != State.FAULT &&
                state != State.DONE) {
            aim.update();
        }

        shooter.update();

        if (ENABLE_RAMP) {
            rampaCmd.execute();
        }

        intake.update(
                false,
                false,
                feeding,
                false
        );

        // Comprobar fallos
        if (turret.isFault()) {
            fail("TURRET_FAULT");
        }

        if (ENABLE_RAMP && rampa.hasFault()) {
            fail("RAMP_FAULT");
        }

        if (state != State.FAULT &&
                state != State.DONE) {
            checkPathTimeout();
        }

        // =====================================
        // MAQUINA DE ESTADOS
        // =====================================

        switch (state) {

            case DRIVE_TO_START:

                if (pathFinished()) {

                    startCollection();
                    startPath(1);

                    changeState(State.COLLECT_ONE);
                }
                break;

            case COLLECT_ONE:

                if (pathFinished()) {
                    startPath(2);
                    changeState(State.RETURN_ONE);
                }
                break;

            case RETURN_ONE:

                if (pathFinished()) {
                    startPath(3);
                    changeState(State.WAIT_SHOT_ONE);
                }
                break;

            case WAIT_SHOT_ONE:

                // Primero terminar la ruta de regreso
                if (pathRunning) {
                    if (pathFinished()) {
                        stopCollection();
                    }
                    break;
                }

                if (readyStable()) {

                    beginFeed();
                    changeState(State.FEED_ONE);

                } else if (stateTime()
                        > SHOOT_WAIT_MAX_S) {

                    fail("SHOT_ONE_NOT_READY");
                }
                break;

            case FEED_ONE:

                if (stateTime() >= FEED_DURATION_S) {

                    stopFeed();

                    startCollection();
                    startPath(4);

                    changeState(State.COLLECT_TWO);
                }
                break;

            case COLLECT_TWO:

                if (pathFinished()) {
                    startPath(5);
                    changeState(State.RETURN_TWO);
                }
                break;

            case RETURN_TWO:

                if (pathFinished()) {
                    startPath(6);
                    changeState(State.WAIT_SHOT_TWO);
                }
                break;

            case WAIT_SHOT_TWO:

                if (pathRunning) {
                    if (pathFinished()) {
                        stopCollection();
                    }
                    break;
                }

                if (readyStable()) {

                    beginFeed();
                    changeState(State.FEED_TWO);

                } else if (stateTime()
                        > SHOOT_WAIT_MAX_S) {

                    fail("SHOT_TWO_NOT_READY");
                }
                break;

            case FEED_TWO:

                if (stateTime() >= FEED_DURATION_S) {

                    stopFeed();

                    startPath(7);
                    changeState(State.PARK);
                }
                break;

            case PARK:

                if (pathFinished()) {
                    changeState(State.DONE);
                }
                break;

            case DONE:

                stopFeed();
                shooter.setEnabled(false);
                intake.stop();

                if (ENABLE_RAMP) {
                    rampaCmd.stop();
                }

                break;

            case FAULT:

                stopFeed();
                shooter.setEnabled(false);
                intake.stop();

                if (ENABLE_RAMP) {
                    rampaCmd.stop();
                }

                aim.disable();
                turret.stop();

                // Detener scheduler y movimiento
                Scheduler.reset();
                follower.manual(0, 0, 0);

                break;
        }

        // =====================================
        // TELEMETRIA
        // =====================================

        telemetry.addData("State", state);
        telemetry.addData("Path", currentPath);
        telemetry.addData("Path Busy", follower.isBusy());

        telemetry.addData("X", drive.getX());
        telemetry.addData("Y", drive.getY());
        telemetry.addData(
                "Heading", drive.getHeadingDegrees());

        telemetry.addData("Tag", selectedTag);
        telemetry.addData("Aim", aim.getStatus());

        telemetry.addData(
                "Turret Angle", turret.getAngle());

        telemetry.addData(
                "Turret Error", turret.getError());

        telemetry.addData(
                "Shooter TPS", shooter.getActualTPS());

        telemetry.addData(
                "Shooter Target", shooter.getTargetTPS());

        telemetry.addData(
                "Ball Count", intake.getBallCount());

        telemetry.addData("Collecting", collecting);
        telemetry.addData("Feeding", feeding);

        telemetry.addData("Ramp Enabled", ENABLE_RAMP);
        telemetry.addData("Ramp Ticks", rampa.getTicks());
        telemetry.addData("Ramp Status", rampa.getStatus());

        telemetry.addData("Fault", faultReason);

        telemetry.update();
    }

    // =========================================
    // STOP
    // =========================================

    @Override
    public void stop() {

        stopFeed();

        intake.stop();
        shooter.stop();

        aim.disable();
        turret.stop();
        vision.stop();

        rampaCmd.stop();

        Scheduler.reset();

        drive.stop();
    }
}

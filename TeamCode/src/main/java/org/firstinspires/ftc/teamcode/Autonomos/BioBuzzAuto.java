package org.firstinspires.ftc.teamcode.Autonomos;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
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
        name = "BIOBUZZ AUTO FINAL V2",
        group = "Pedro"
)
public class BioBuzzAuto extends OpMode {
    public static double SHOOTER_TPS = 1200;
    public static double SHOOTER_TOLERANCE = 60;
    public static double AIM_TOLERANCE = 2.5;
    public static double READY_STABLE_S = 0.08;
    public static double SHOOT_WAIT_MAX_S = 2.0;
    public static double INITIAL_FEED_DURATION_S = 2.0;
    public static double FEED_DURATION_S = 2.0;
    public static double INTAKE_DWELL_S = 2.0;
    public static double PATH_TIMEOUT_S = 12.0;
    public static double PATH_POSITION_TOLERANCE = 3.0;
    public static double FIELD_MID_Y = 72.0;
    public static double TAG_HYSTERESIS = 1.5;
    public static int INITIAL_TAG = 37;
    public static int OPPOSITE_TAG = 30;
    public static boolean ENABLE_RAMP = true;
    private Follower follower;
    private DriveSubsystem drive;
    private TurretSubsystemV4 turret;
    private TurretVisionV5 vision;
    private TurretAimControllerV77 aim;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private RampaSubsystem rampa;
    private RampaCommand rampaCmd;

    private final PoseFactory pf = PoseFactory.degrees();

    private final Pose p0 = pf.of(44.5916, 9.0788, 90);
    private final Pose p1 = pf.of(44.6143, 15.2436, 90);
    private final Pose p2 = pf.of(17.8901, 46.9881, 180);
    private final Pose p3 = pf.of(9.9444, 46.7875, 180);
    private final Pose p4 = pf.of(17.4832, 47.0671, 180);
    private final Pose p5 = pf.of(47.38, 125.9494, 90);
    private final Pose p6 = pf.of(47.047, 131.2394, 90);
    private final Pose p7 = pf.of(47.4948, 127.4309, 90);
    private final Pose p8 = pf.of(9.8474, 120.0793, 90);

    private Path[] paths;
    private final Pose[] destinations = { p1, p2, p3, p4, p5, p6, p7, p8 };

    private enum State {
        DRIVE_TO_START,
        WAIT_INITIAL_SHOT,
        FEED_INITIAL,
        CURVE_TO_GRADIENT_ONE,
        APPROACH_GRADIENT_ONE,
        INTAKE_WAIT_ONE,
        RETURN_ONE,
        WAIT_SHOT_ONE,
        FEED_ONE,
        CURVE_TO_GRADIENT_TWO,
        APPROACH_GRADIENT_TWO,
        INTAKE_WAIT_TWO,
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
    private String faultReason = "NONE";

    private long stateStartNs;
    private long pathStartNs;
    private long readyStartNs;

    private int selectedTag = INITIAL_TAG;
    private boolean oppositeSide = false;

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);

        drive = new DriveSubsystem(follower);
        drive.setPose(p0);

        turret = new TurretSubsystemV4(hardwareMap);
        vision = new TurretVisionV5(hardwareMap);
        aim = new TurretAimControllerV77(drive, turret, vision);

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        rampa = new RampaSubsystem(hardwareMap);
        rampaCmd = new RampaCommand(rampa);

        aim.setMode(TurretAimControllerV77.Mode.COMBINED);
        turret.maxPower = 1.0;

        paths = new Path[]{
                Paths.line(p0, p1).constant(p1),
                Paths.curve(p1, pf.of(41.4224, 34.4952, 0), pf.of(37.5983, 44.159, 0), p2).linear(p1, p2),
                Paths.line(p2, p3).constant(p3),
                Paths.line(p3, p4).constant(p4),
                Paths.curve(p4, pf.of(30.5411, 111.1691, 0), p5).linear(p4, p5),
                Paths.line(p5, p6).constant(p6),
                Paths.line(p6, p7).constant(p7),
                Paths.line(p7, p8).constant(p8)
        };

        shooter.setTargetTPS(SHOOTER_TPS);
        shooter.setEnabled(false);

        telemetry.addLine("BIOBUZZ AUTO V2 LISTO");
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

        if (ENABLE_RAMP) rampaCmd.subir();

        startPath(0);
        changeState(State.DRIVE_TO_START);
    }

    private double secondsSince(long ns) { return (System.nanoTime() - ns) * 1e-9; }
    private double stateTime() { return secondsSince(stateStartNs); }
    private void changeState(State next) { state = next; stateStartNs = System.nanoTime(); readyStartNs = 0; }

    private void startPath(int index) {
        currentPath = index;
        pathRunning = true;
        observedBusy = false;
        pathStartNs = System.nanoTime();
        schedule(follow(follower, paths[index]));
    }

    private boolean pathFinished() {
        if (!pathRunning) return false;
        if (follower.isBusy()) { observedBusy = true; return false; }
        if (!observedBusy) return false;

        Pose goal = destinations[currentPath];
        Pose actual = follower.pose();
        double distance = Math.hypot(actual.x() - goal.x(), actual.y() - goal.y());

        if (distance > PATH_POSITION_TOLERANCE) {
            fail("PATH_END_POSITION_ERROR");
            return false;
        }
        pathRunning = false;
        return true;
    }

    private void checkPathTimeout() {
        if (pathRunning && secondsSince(pathStartNs) > PATH_TIMEOUT_S) {
            fail("PATH_TIMEOUT_" + currentPath);
        }
    }

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

    private void startCollection() {
        collecting = true;
        if (!intake.isAutoIntakeEnabled()) intake.toggleAutoIntake();
        if (ENABLE_RAMP) rampaCmd.bajar();
    }

    private void stopCollection() {
        collecting = false;
        if (intake.isAutoIntakeEnabled()) intake.toggleAutoIntake();
        if (ENABLE_RAMP) rampaCmd.subir();
    }

    private boolean mechanismsReady() {
        boolean shooterReady = Math.abs(shooter.getActualTPS() - shooter.getTargetTPS()) <= SHOOTER_TOLERANCE;
        boolean turretReady = turret.isReadyToShoot(AIM_TOLERANCE) && !turret.isFault() && aim.isEnabled();
        boolean rampReady = !ENABLE_RAMP || rampa.isReady();
        boolean targetKnown = selectedTag == INITIAL_TAG || selectedTag == OPPOSITE_TAG;
        return shooterReady && turretReady && rampReady && targetKnown;
    }

    private boolean readyStable() {
        if (!mechanismsReady()) { readyStartNs = 0; return false; }
        if (readyStartNs == 0) { readyStartNs = System.nanoTime(); return false; }
        return secondsSince(readyStartNs) >= READY_STABLE_S;
    }

    private void fail(String reason) {
        if (state == State.FAULT || state == State.DONE) return;
        faultReason = reason;
        changeState(State.FAULT);
    }

    @Override
    public void loop() {
        drive.update();
        Scheduler.execute();

        updateSelectedTag();
        vision.update();

        if (state != State.FAULT) {
            if (!aim.isEnabled() && !turret.isFault()) aim.enable();
            aim.update();
        }

        shooter.update();
        if (ENABLE_RAMP) rampaCmd.execute();
        intake.update(false, false, feeding, false);

        if (turret.isFault()) fail("TURRET_FAULT");
        if (ENABLE_RAMP && rampa.hasFault()) fail("RAMP_FAULT");
        if (state != State.FAULT && state != State.DONE) checkPathTimeout();

        switch (state) {
            case DRIVE_TO_START:
                if (pathFinished()) changeState(State.WAIT_INITIAL_SHOT);
                break;

            case WAIT_INITIAL_SHOT:
                if (readyStable() || stateTime() > SHOOT_WAIT_MAX_S) {
                    feeding = true;
                    changeState(State.FEED_INITIAL);
                }
                break;

            case FEED_INITIAL:
                if (stateTime() >= INITIAL_FEED_DURATION_S) {
                    feeding = false;
                    startPath(1);
                    changeState(State.CURVE_TO_GRADIENT_ONE);
                }
                break;

            case CURVE_TO_GRADIENT_ONE:
                if (pathFinished()) {
                    startCollection();
                    startPath(2);
                    changeState(State.APPROACH_GRADIENT_ONE);
                }
                break;

            case APPROACH_GRADIENT_ONE:
                if (pathFinished()) changeState(State.INTAKE_WAIT_ONE);
                break;

            case INTAKE_WAIT_ONE:
                if (stateTime() >= INTAKE_DWELL_S) {
                    stopCollection();
                    startPath(3);
                    changeState(State.RETURN_ONE);
                }
                break;

            case RETURN_ONE:
                if (pathFinished()) changeState(State.WAIT_SHOT_ONE);
                break;

            case WAIT_SHOT_ONE:
                if (readyStable() || stateTime() > SHOOT_WAIT_MAX_S) {
                    feeding = true;
                    changeState(State.FEED_ONE);
                }
                break;

            case FEED_ONE:
                if (stateTime() >= FEED_DURATION_S) {
                    feeding = false;
                    startPath(4);
                    changeState(State.CURVE_TO_GRADIENT_TWO);
                }
                break;

            case CURVE_TO_GRADIENT_TWO:
                if (pathFinished()) {
                    startCollection();
                    startPath(5);
                    changeState(State.APPROACH_GRADIENT_TWO);
                }
                break;

            case APPROACH_GRADIENT_TWO:
                if (pathFinished()) changeState(State.INTAKE_WAIT_TWO);
                break;

            case INTAKE_WAIT_TWO:
                if (stateTime() >= INTAKE_DWELL_S) {
                    stopCollection();
                    startPath(6);
                    changeState(State.RETURN_TWO);
                }
                break;

            case RETURN_TWO:
                if (pathFinished()) changeState(State.WAIT_SHOT_TWO);
                break;

            case WAIT_SHOT_TWO:
                if (readyStable() || stateTime() > SHOOT_WAIT_MAX_S) {
                    feeding = true;
                    changeState(State.FEED_TWO);
                }
                break;

            case FEED_TWO:
                if (stateTime() >= FEED_DURATION_S) {
                    feeding = false;
                    startPath(7);
                    changeState(State.PARK);
                }
                break;

            case PARK:
                if (pathFinished()) changeState(State.DONE);
                break;

            case DONE:
                feeding = false;
                shooter.setEnabled(false);
                intake.stop();
                break;

            case FAULT:
                feeding = false;
                shooter.setEnabled(false);
                intake.stop();
                if (ENABLE_RAMP) rampaCmd.stop();
                aim.disable();
                turret.stop();
                Scheduler.reset();
                follower.manual(0, 0, 0);
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Tiempo estado (s)", "%.2f", stateTime());
        telemetry.addData("Pose", "X: %.1f | Y: %.1f | H: %.1f°",
                drive.getX(), drive.getY(), drive.getHeadingDegrees());
        telemetry.addData("Tag Activo", "%d (%s)",
                selectedTag, oppositeSide ? "CONTRARIA" : "INICIAL");
        telemetry.addData("Aim Status", aim.getStatus());
        telemetry.addData("Turret Error", "%.2f°", turret.getError());
        telemetry.addData("Shooter TPS", "%.0f / %.0f",
                shooter.getActualTPS(), shooter.getTargetTPS());
        telemetry.addData("Ball Count", intake.getBallCount());
        telemetry.addData("Ramp Status", "%s (Ticks: %d)",
                rampa.getStatus(), rampa.getTicks());
        telemetry.addData("Fault", faultReason);
        telemetry.update();
    }

    @Override
    public void stop() {
        feeding = false;
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
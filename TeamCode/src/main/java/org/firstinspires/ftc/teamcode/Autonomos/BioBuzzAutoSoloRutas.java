package org.firstinspires.ftc.teamcode.Autonomos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(
        name = "BIOBUZZ AUTO - SOLO RUTAS",
        group = "Pedro"
)
public class BioBuzzAutoSoloRutas extends OpMode {

    // =========================================
    // AJUSTES DE PRUEBA DE RUTAS
    // =========================================

    public static double PATH_TIMEOUT_S = 12.0;
    public static double PATH_POSITION_TOLERANCE = 3.0;

    // Pausa breve en los puntos de disparo (p4 y p7) para verificar llegada
    public static double SHOT_POINT_PAUSE_S = 1.0;
    // Pausa rapida entre tramos normales
    public static double WAYPOINT_PAUSE_S = 0.25;

    private static final double FIELD_HALF = 72.0;

    // =========================================
    // FOLLOWER Y DRIVE
    // =========================================

    private Follower follower;
    private DriveSubsystem drive;
    private FtcDashboard dashboard;

    // =========================================
    // POSES (IDENTICAS A BIOBUZZ AUTO FINAL V2)
    // =========================================

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

    private final Pose[] destinations = {
            p1, p2, p3, p4,
            p5, p6, p7, p8
    };

    private final String[] pathNames = {
            "0: p0 -> p1 (Salida)",
            "1: p1 -> p2 (Curva Recoleccion 1)",
            "2: p2 -> p3 (Entrar Recoleccion 1)",
            "3: p3 -> p4 (Posicion Tiro 1)",
            "4: p4 -> p5 (Curva Cruce Cancha)",
            "5: p5 -> p6 (Entrar Recoleccion 2)",
            "6: p6 -> p7 (Posicion Tiro 2)",
            "7: p7 -> p8 (Estacionar / Park)"
    };

    // =========================================
    // ESTADOS
    // =========================================

    private enum State {
        RUNNING_PATH,
        PAUSE_AT_POINT,
        DONE,
        FAULT
    }

    private State state = State.RUNNING_PATH;

    private int currentPath = -1;
    private boolean pathRunning = false;
    private boolean observedBusy = false;

    private double lastEndErrorIn = 0.0;
    private double lastHeadingErrorDeg = 0.0;
    private double pauseDurationS = 0.0;

    private String faultReason = "NONE";

    private long startNs;
    private long stateStartNs;
    private long pathStartNs;

    // =========================================
    // INIT
    // =========================================

    @Override
    public void init() {

        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        drive = new DriveSubsystem(follower);
        drive.setPose(p0);

        dashboard = FtcDashboard.getInstance();

        // Construir exactamente las mismas 8 rutas
        paths = new Path[]{

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

        telemetry.addLine("BIOBUZZ SOLO RUTAS LISTO");
        telemetry.addLine("Todos los mecanismos desactivados");
        telemetry.addData("Pose Inicial", "(%.2f, %.2f, %.1f°)", p0.x(), p0.y(), Math.toDegrees(p0.heading()));
        telemetry.update();
    }

    // =========================================
    // START
    // =========================================

    @Override
    public void start() {
        startNs = System.nanoTime();
        startPath(0);
    }

    // =========================================
    // CONTROL DE RUTAS
    // =========================================

    private double secondsSince(long ns) {
        return (System.nanoTime() - ns) * 1e-9;
    }

    private void startPath(int index) {
        currentPath = index;
        pathRunning = true;
        observedBusy = false;
        pathStartNs = System.nanoTime();
        stateStartNs = System.nanoTime();
        state = State.RUNNING_PATH;

        schedule(follow(follower, paths[index]));
    }

    private boolean checkPathFinished() {
        if (!pathRunning) return false;

        if (follower.isBusy()) {
            observedBusy = true;
            return false;
        }

        if (!observedBusy) {
            return false;
        }

        Pose goal = destinations[currentPath];
        Pose actual = follower.pose();

        double dx = actual.x() - goal.x();
        double dy = actual.y() - goal.y();
        lastEndErrorIn = Math.hypot(dx, dy);

        double actualDeg = Math.toDegrees(actual.heading());
        double goalDeg = Math.toDegrees(goal.heading());
        lastHeadingErrorDeg = wrap180(actualDeg - goalDeg);

        if (lastEndErrorIn > PATH_POSITION_TOLERANCE) {
            faultReason = String.format(
                    "ERROR_POS_PATH_%d (%.2f in)",
                    currentPath,
                    lastEndErrorIn
            );
            state = State.FAULT;
            return false;
        }

        pathRunning = false;
        return true;
    }

    private double wrap180(double angle) {
        angle %= 360.0;
        if (angle > 180.0) angle -= 360.0;
        if (angle < -180.0) angle += 360.0;
        return angle;
    }

    // =========================================
    // LOOP
    // =========================================

    @Override
    public void loop() {

        drive.update();
        Scheduler.execute();

        switch (state) {

            case RUNNING_PATH:

                if (secondsSince(pathStartNs) > PATH_TIMEOUT_S) {
                    faultReason = "TIMEOUT_PATH_" + currentPath;
                    state = State.FAULT;
                    break;
                }

                if (checkPathFinished()) {
                    // Si terminamos la ultima ruta (p8), hemos terminado
                    if (currentPath >= paths.length - 1) {
                        state = State.DONE;
                    } else {
                        // Pausa mas larga en p4 (path 3) y p7 (path 6) donde dispararia
                        pauseDurationS = (currentPath == 3 || currentPath == 6)
                                ? SHOT_POINT_PAUSE_S
                                : WAYPOINT_PAUSE_S;
                        stateStartNs = System.nanoTime();
                        state = State.PAUSE_AT_POINT;
                    }
                }
                break;

            case PAUSE_AT_POINT:

                if (secondsSince(stateStartNs) >= pauseDurationS) {
                    startPath(currentPath + 1);
                }
                break;

            case DONE:
                follower.manual(0, 0, 0);
                break;

            case FAULT:
                Scheduler.reset();
                follower.manual(0, 0, 0);
                break;
        }

        // =====================================
        // TELEMETRIA DRIVER STATION
        // =====================================

        double totalTime = secondsSince(startNs);

        telemetry.addLine("=== PRUEBA SOLO RUTAS ===");
        telemetry.addData("Estado", state);
        telemetry.addData("Tiempo Total", "%.2f s", totalTime);

        if (currentPath >= 0 && currentPath < pathNames.length) {
            telemetry.addData("Tramo", pathNames[currentPath]);
        }

        telemetry.addData("Pose Actual", "X: %.2f | Y: %.2f | H: %.1f°",
                drive.getX(), drive.getY(), drive.getHeadingDegrees());

        if (currentPath >= 0 && currentPath < destinations.length) {
            Pose target = destinations[currentPath];
            telemetry.addData("Destino Tramo", "X: %.2f | Y: %.2f | H: %.1f°",
                    target.x(), target.y(), Math.toDegrees(target.heading()));
        }

        telemetry.addData("Ultimo Error Pos", "%.2f in", lastEndErrorIn);
        telemetry.addData("Ultimo Error Ang", "%.2f°", lastHeadingErrorDeg);
        telemetry.addData("Fallo", faultReason);
        telemetry.update();

        // =====================================
        // FTC DASHBOARD
        // =====================================
        sendDashboardTelemetry(totalTime);
    }

    private void sendDashboardTelemetry(double totalTime) {
        TelemetryPacket packet = new TelemetryPacket(false);
        Canvas c = packet.fieldOverlay();

        // Fondo de cancha
        c.setFill("#292929").fillRect(-72, -72, 144, 144);
        c.setStroke("#555555").setStrokeWidth(1);
        for (int i = -72; i <= 72; i += 24) {
            c.strokeLine(i, -72, i, 72);
            c.strokeLine(-72, i, 72, i);
        }
        c.setStroke("#B0B0B0").setStrokeWidth(1).strokeRect(-72, -72, 144, 144);

        // Dibujar secuencia de waypoints (p0 -> p8)
        Pose[] allPoints = {p0, p1, p2, p3, p4, p5, p6, p7, p8};
        c.setStroke("#64748B").setStrokeWidth(1);
        for (int i = 0; i < allPoints.length - 1; i++) {
            c.strokeLine(
                    allPoints[i].x() - FIELD_HALF,
                    allPoints[i].y() - FIELD_HALF,
                    allPoints[i + 1].x() - FIELD_HALF,
                    allPoints[i + 1].y() - FIELD_HALF
            );
        }

        // Resaltar destino actual en verde
        if (currentPath >= 0 && currentPath < destinations.length) {
            Pose dest = destinations[currentPath];
            c.setFill("#22C55E").fillCircle(dest.x() - FIELD_HALF, dest.y() - FIELD_HALF, 3.5);
        }

        // Robot actual
        double dashX = drive.getX() - FIELD_HALF;
        double dashY = drive.getY() - FIELD_HALF;
        double headingRad = drive.getHeadingRadians();

        c.setStroke(state == State.FAULT ? "#EF4444" : "#3B82F6")
                .setStrokeWidth(3)
                .strokeCircle(dashX, dashY, 9);

        c.setStroke(state == State.FAULT ? "#EF4444" : "#3B82F6")
                .setStrokeWidth(3)
                .strokeLine(
                        dashX,
                        dashY,
                        dashX + 14 * Math.cos(headingRad),
                        dashY + 14 * Math.sin(headingRad)
                );

        packet.put("Estado", state.toString());
        packet.put("Tramo", currentPath);
        packet.put("Tiempo Total (s)", totalTime);
        packet.put("X", drive.getX());
        packet.put("Y", drive.getY());
        packet.put("Heading", drive.getHeadingDegrees());
        packet.put("Ultimo Error (in)", lastEndErrorIn);

        dashboard.sendTelemetryPacket(packet);
    }

    // =========================================
    // STOP
    // =========================================

    @Override
    public void stop() {
        Scheduler.reset();
        if (drive != null) {
            drive.stop();
        }
    }
}
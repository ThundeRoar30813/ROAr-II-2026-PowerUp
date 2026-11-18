package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretVisionV5;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretAimControllerV77;

@TeleOp(name = "BIOBUZZ TELEOP - ROJO", group = "Competencia")
public class BioBuzzTeleOpRojo extends OpMode {

    public static double START_X = 48.0;
    public static double START_Y = 6.0;
    public static double START_HEADING = 90.0;

    public static double FIELD_SIZE = 144.0;
    public static double FIELD_MID_Y = 72.0;
    private static final double FIELD_HALF = 72.0;

    public static double RED_SPLIT_X_BOTTOM = 55.8; // Entre 34 (50.71) y 37 (60.89)
    public static double RED_SPLIT_X_TOP = 56.6;    // Entre 33 (50.33) y 30 (62.96)
    public static double ZONE_HYSTERESIS = 1.5;

    public static double IDLE_TPS = 600.0;
    public static double RT_THRESHOLD = 0.20;
    public static double HOOD_HYSTERESIS = 0.15;
    public static double MAX_TABLE_DISTANCE_IN = 36.0;

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCmd;
    private TurretSubsystemV4 turret;
    private TurretVisionV5 vision;
    private TurretAimControllerV77 aim;
    private IntakeSubsystem intake;
    private IntakeCommand intakeCmd;
    private ShooterSubsystem shooter;
    private HoodSubsystem hood;
    private FtcDashboard dashboard;

    private boolean isUpperHalf = false;
    private boolean isRightSide = false;
    private int selectedTag = 37;

    private boolean tracking = false;
    private boolean previousB = false;
    private boolean previousStart = false;
    private boolean previousBack = false;
    private boolean previousUp = false;
    private boolean previousDown = false;

    private int hoodLevel = 3;
    private double globalTPSOffset = 0.0;
    private double requestedTPS = IDLE_TPS;

    private double tableX = Double.NaN;
    private double tableY = Double.NaN;
    private boolean validTable = false;
    private ShooterInterpolation.Result interpolation;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap);
        drive.setInitialPose(START_X, START_Y, START_HEADING);

        driveCmd = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );

        turret = new TurretSubsystemV4(hardwareMap);
        vision = new TurretVisionV5(hardwareMap);
        aim = new TurretAimControllerV77(drive, turret, vision);
        aim.setMode(TurretAimControllerV77.Mode.COMBINED);

        selectedTag = 37;
        vision.setSelectedTag(selectedTag);

        intake = new IntakeSubsystem(hardwareMap);
        intakeCmd = new IntakeCommand(intake);

        shooter = new ShooterSubsystem(hardwareMap);
        shooter.setTargetTPS(IDLE_TPS);
        shooter.setEnabled(false);

        hood = new HoodSubsystem(hardwareMap);
        hoodLevel = 3;
        hood.setIndex(hoodLevel - 1);

        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("BIOBUZZ TELEOP ROJO LISTO");
        telemetry.addLine("Abajo: Izq 37 / Der 34 | Arriba: Izq 30 / Der 33");
        telemetry.update();
    }

    @Override
    public void start() {
        isUpperHalf = false;
        isRightSide = false;
        selectedTag = 37;
        vision.setSelectedTag(selectedTag);
        vision.start();

        shooter.setTargetTPS(IDLE_TPS);
        shooter.setEnabled(true);
        tracking = false;
    }

    private void updateZoneAndTag() {
        double x = drive.getX();
        double y = drive.getY();
        if (!Double.isFinite(x) || !Double.isFinite(y)) return;

        if (isUpperHalf) {
            if (y < FIELD_MID_Y - ZONE_HYSTERESIS) isUpperHalf = false;
        } else {
            if (y > FIELD_MID_Y + ZONE_HYSTERESIS) isUpperHalf = true;
        }

        double splitX = isUpperHalf ? RED_SPLIT_X_TOP : RED_SPLIT_X_BOTTOM;
        if (isRightSide) {
            if (x < splitX - ZONE_HYSTERESIS) isRightSide = false;
        } else {
            if (x > splitX + ZONE_HYSTERESIS) isRightSide = true;
        }

        if (!isUpperHalf) {
            selectedTag = isRightSide ? 34 : 37;
        } else {
            selectedTag = isRightSide ? 33 : 30;
        }

        vision.setSelectedTag(selectedTag);
    }

    private void updateTPSAdjustment() {
        boolean up = gamepad2.dpad_up;
        boolean down = gamepad2.dpad_down;
        if (up && !previousUp) globalTPSOffset += 5.0;
        if (down && !previousDown) globalTPSOffset -= 2.0;
        previousUp = up;
        previousDown = down;
    }

    private void updateInterpolation() {
        double x = drive.getX();
        double y = drive.getY();

        validTable = false;
        interpolation = null;

        if (!Double.isFinite(x) || !Double.isFinite(y)) return;

        tableX = x;
        tableY = isUpperHalf ? (FIELD_SIZE - y) : y;

        interpolation = ShooterInterpolation.calculate(tableX, tableY);
        if (interpolation == null) return;

        validTable = Double.isFinite(interpolation.tps)
                && Double.isFinite(interpolation.nearestDistance)
                && interpolation.nearestDistance <= MAX_TABLE_DISTANCE_IN;

        if (validTable) {
            updateHood(interpolation.hoodInterpolated);
        }
    }

    private void updateHood(double level) {
        if (!Double.isFinite(level)) return;
        int nextLevel = hoodLevel;
        while (nextLevel < 5 && level > nextLevel + 0.5 + HOOD_HYSTERESIS) nextLevel++;
        while (nextLevel > 1 && level < nextLevel - 0.5 - HOOD_HYSTERESIS) nextLevel--;
        if (nextLevel != hoodLevel) {
            hoodLevel = nextLevel;
            hood.setIndex(hoodLevel - 1);
        }
    }

    private void updateTurretControls() {
        boolean b = gamepad2.b;
        boolean home = gamepad2.start;
        boolean reset = gamepad2.back;

        if (reset && !previousBack) turret.clearFault();

        if (home && !previousStart) {
            if (aim.isEnabled()) aim.disable();
            tracking = false;
            turret.enableHold();
            turret.setTarget(0.0);
        }

        if (b && !previousB) {
            if (tracking) {
                aim.disable();
                tracking = false;
                turret.enableHold();
                turret.setTarget(turret.getAngle());
            } else {
                aim.enable();
                tracking = aim.isEnabled();
            }
        }

        previousB = b;
        previousStart = home;
        previousBack = reset;

        if (tracking && !aim.isEnabled()) tracking = false;

        if (tracking) {
            aim.update();
        } else {
            turret.setVelocityFeedforward(0);
            turret.update();
        }
    }

    @Override
    public void loop() {
        driveCmd.execute();
        drive.update();

        updateTPSAdjustment();
        updateZoneAndTag();
        vision.update();
        updateTurretControls();

        intakeCmd.execute(
                gamepad2.y,
                gamepad2.right_bumper,
                gamepad2.a,
                gamepad2.left_bumper
        );

        updateInterpolation();

        boolean shooting = gamepad2.right_trigger > RT_THRESHOLD;
        if (shooting && validTable && interpolation != null) {
            requestedTPS = interpolation.tps + globalTPSOffset;
        } else {
            requestedTPS = IDLE_TPS;
        }

        shooter.setTargetTPS(Math.max(0.0, requestedTPS));
        shooter.setEnabled(true);
        shooter.update();

        telemetry.addLine("=== BIOBUZZ TELEOP ROJO ===");
        telemetry.addData("Pose", "X: %.1f | Y: %.1f | H: %.1f°",
                drive.getX(), drive.getY(), drive.getHeadingDegrees());
        telemetry.addData("Zona", "%s - %s",
                isUpperHalf ? "ARRIBA" : "ABAJO",
                isRightSide ? "DERECHA" : "IZQUIERDA");
        telemetry.addData("Tag Activo", "%d (Visible: %s)", selectedTag, vision.hasTarget());
        telemetry.addData("Turret Error", "%.2f° (Ready: %s)",
                turret.getError(), turret.isReadyToShoot(2.5));
        telemetry.addData("Shooter TPS", "%.0f / %.0f (Offset %+.0f)",
                shooter.getActualTPS(), shooter.getTargetTPS(), globalTPSOffset);
        telemetry.addData("Hood Level", "%d / 5", hoodLevel);
        telemetry.addData("Pelotas", intake.getBallCount());
        telemetry.update();

        sendDashboardPacket(shooting);
    }

    private void sendDashboardPacket(boolean shooting) {
        TelemetryPacket packet = new TelemetryPacket(false);
        Canvas c = packet.fieldOverlay();

        c.setFill("#292929").fillRect(-72, -72, 144, 144);
        c.setStroke("#555555").setStrokeWidth(1);
        for (int i = -72; i <= 72; i += 24) {
            c.strokeLine(i, -72, i, 72);
            c.strokeLine(-72, i, 72, i);
        }
        c.setStroke("#EF3340").setStrokeWidth(2).strokeRect(-72, -72, 144, 144);

        double gx = TurretAimControllerV77.TAG37_X, gy = TurretAimControllerV77.TAG37_Y;
        if (selectedTag == 34) { gx = TurretAimControllerV77.TAG34_X; gy = TurretAimControllerV77.TAG34_Y; }
        else if (selectedTag == 30) { gx = TurretAimControllerV77.TAG30_X; gy = TurretAimControllerV77.TAG30_Y; }
        else if (selectedTag == 33) { gx = TurretAimControllerV77.TAG33_X; gy = TurretAimControllerV77.TAG33_Y; }

        double rx = drive.getX() - FIELD_HALF;
        double ry = drive.getY() - FIELD_HALF;
        double hRad = drive.getHeadingRadians();

        c.setStroke("#EAB308").setStrokeWidth(2).strokeLine(rx, ry, gx - FIELD_HALF, gy - FIELD_HALF);
        c.setFill("#EF3340").fillCircle(gx - FIELD_HALF, gy - FIELD_HALF, 4);
        c.setStroke("#3B82F6").setStrokeWidth(3).strokeCircle(rx, ry, 9);
        c.setStroke("#3B82F6").setStrokeWidth(3).strokeLine(rx, ry, rx + 14 * Math.cos(hRad), ry + 14 * Math.sin(hRad));

        packet.put("Alianza", "ROJO");
        packet.put("Tag Activo", selectedTag);
        packet.put("TPS Target", shooter.getTargetTPS());
        packet.put("TPS Real", shooter.getActualTPS());
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        if (shooter != null) shooter.stop();
        if (intakeCmd != null) intakeCmd.stop();
        if (aim != null) aim.disable();
        if (vision != null) vision.stop();
        if (turret != null) { turret.setVelocityFeedforward(0); turret.disable(); }
        if (driveCmd != null) driveCmd.stop();
    }
}
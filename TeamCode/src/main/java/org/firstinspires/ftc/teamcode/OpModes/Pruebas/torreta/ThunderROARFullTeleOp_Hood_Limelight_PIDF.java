package org.firstinspires.ftc.teamcode.OpModes.Pruebas.torreta;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.Commands.TurretFieldTrackingCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

@TeleOp(name = "ThunderROAR - Vision + Hood + PIDF")
public class ThunderROARFullTeleOp_Hood_Limelight_PIDF extends OpMode {

    // COORDENADAS EN PULGADAS: completar con las esquinas reales del HIGH GOAL.
    // Mientras sean NaN, el apuntado automático queda bloqueado por seguridad.
    private static final double LEFT_GOAL_X = Double.NaN;
    private static final double LEFT_GOAL_Y = Double.NaN;
    private static final double RIGHT_GOAL_X = Double.NaN;
    private static final double RIGHT_GOAL_Y = Double.NaN;

    private static final String LIMELIGHT_NAME = "limelight";
    private static final int LIMELIGHT_PIPELINE = 0; // Pipeline AprilTag en Limelight
    private static final long MAX_VISION_AGE_MS = 150;
    // IDs indicados por el usuario: confirmar en la Driver Station que corresponden a las esquinas.
    private static final int LEFT_TAG_ID = 34;
    private static final int RIGHT_TAG_ID = 36;
    private static final double VISION_CENTER_TOLERANCE_DEG = 2.0;

    private static final double[] HOOD_POSITIONS = {
            0.4444, 0.5560, 0.6667, 0.7778, 0.8890
    };
    private static final String[] HOOD_NAMES = {
            "ABAJO", "BAJA", "MEDIA", "ALTA", "ARRIBA"
    };

    private static final double GATE_CLOSED_DEG = 90.0;
    private static final double GATE_OPEN_DEG = 20.0;
    private static final double INTAKE_POWER = 1.0;
    private static final double INDEXER_POWER = 1.0;

    private DriveSubsystem drive;
    private MecanumDriveCmd driveCommand;
    private TurretSubsystem turret;
    private TurretFieldTrackingCommand tracking;
    private Limelight3A limelight;
    private DcMotorEx shooter;
    private DcMotor intake, indexer;
    private Servo gate, hood;

    private double targetVelocity = 800;
    private double p = 4.0, i = 0.0, d = 0.5, f = 12.0;
    private boolean shooterEnabled = false;
    private boolean intakeEnabled = false;
    private boolean rightCorner = false;
    private int hoodIndex = 2;
    private double currentGateAngle = Double.NaN;
    private boolean gateWasOpened = false;

    private boolean previousA, previousB, previousY;
    private boolean previousUp, previousDown, previousLeft, previousRight;
    private boolean previousHoodUp, previousHoodDown;
    private boolean previousCornerLeft, previousCornerRight;

    private double goalX() { return rightCorner ? RIGHT_GOAL_X : LEFT_GOAL_X; }
    private double goalY() { return rightCorner ? RIGHT_GOAL_Y : LEFT_GOAL_Y; }
    private boolean goalConfigured() {
        return Double.isFinite(goalX()) && Double.isFinite(goalY());
    }

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap);
        driveCommand = new MecanumDriveCmd(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x
        );

        turret = new TurretSubsystem(hardwareMap);
        // Se crea al habilitar B, solo cuando las coordenadas estén configuradas.

        shooter = hardwareMap.get(DcMotorEx.class, "shoot");
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPower(0);
        applyPIDF();

        intake = hardwareMap.get(DcMotor.class, "intake2");
        indexer = hardwareMap.get(DcMotor.class, "intake1");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        indexer.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);
        indexer.setPower(0);

        gate = hardwareMap.get(Servo.class, "gate");
        gate.setDirection(Servo.Direction.REVERSE);
        // No mover el gate durante INIT.

        hood = hardwareMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.FORWARD);
        // No mover la capucha durante INIT.

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);

        telemetry.addLine("READY - A shooter / B tracking / Y intake / X alimentar");
        telemetry.addLine("G1 DPAD UP/DOWN hood; LB/RB esquina");
        telemetry.addLine("G2 DPAD UP=P DOWN=F RIGHT=D LEFT=I");
        telemetry.addLine("Configura coordenadas antes de habilitar tracking");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
        // Sin movimientos inesperados en START.
    }

    @Override
    public void loop() {
        driveCommand.execute();
        drive.update();

        // GAMEPAD 1: ELEGIR ESQUINA SIN CAMBIAR EL PIDF
        boolean cornerLeft = gamepad1.left_bumper;
        boolean cornerRight = gamepad1.right_bumper;
        if (cornerLeft && !previousCornerLeft) selectCorner(false);
        if (cornerRight && !previousCornerRight) selectCorner(true);
        previousCornerLeft = cornerLeft;
        previousCornerRight = cornerRight;

        // HOOD: posiciones ya medidas, sin convertirlas a grados físicos.
        boolean hoodUp = gamepad1.dpad_up;
        boolean hoodDown = gamepad1.dpad_down;
        if (hoodUp && !previousHoodUp) setHoodIndex(hoodIndex + 1);
        if (hoodDown && !previousHoodDown) setHoodIndex(hoodIndex - 1);
        previousHoodUp = hoodUp;
        previousHoodDown = hoodDown;

        // INTAKE / GATE
        boolean y = gamepad1.y;
        if (y && !previousY) intakeEnabled = !intakeEnabled;
        previousY = y;
        boolean feeding = gamepad1.x;

        if (feeding) {
            setGateAngle(GATE_OPEN_DEG);
            gateWasOpened = true;
            intake.setPower(INTAKE_POWER);
            indexer.setPower(INDEXER_POWER);
        } else if (intakeEnabled) {
            setGateAngle(GATE_CLOSED_DEG);
            gateWasOpened = false;
            intake.setPower(INTAKE_POWER);
            indexer.setPower(INDEXER_POWER);
        } else {
            intake.setPower(0);
            indexer.setPower(0);
            // Si se acaba de soltar X, cerrar gate una sola vez.
            if (gateWasOpened) {
                setGateAngle(GATE_CLOSED_DEG);
                gateWasOpened = false;
            }
        }

        // SHOOTER
        boolean a = gamepad1.a;
        if (a && !previousA) shooterEnabled = !shooterEnabled;
        previousA = a;
        targetVelocity += gamepad1.right_trigger * 5.0;
        targetVelocity -= gamepad1.left_trigger * 5.0;
        targetVelocity = Math.max(0, Math.min(6000, targetVelocity));
        shooter.setVelocity(shooterEnabled ? targetVelocity : 0);

        // TRACKING: conserva el comando que ya funcionaba en tu robot.
        boolean b = gamepad1.b;
        if (b && !previousB) {
            if (tracking != null && tracking.isEnabled()) {
                tracking.disable();
            } else if (goalConfigured()) {
                tracking = new TurretFieldTrackingCommand(
                        drive, turret, goalX(), goalY());
                tracking.enable();
            }
        }
        previousB = b;
        if (tracking != null) tracking.execute();

        // PIDF: mismos valores e incrementos originales, ahora en GAMEPAD 2.
        boolean up = gamepad2.dpad_up;
        boolean down = gamepad2.dpad_down;
        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        if (up && !previousUp) { p += 0.2; applyPIDF(); }
        if (down && !previousDown) { f += 0.5; applyPIDF(); }
        if (right && !previousRight) { d += 0.1; applyPIDF(); }
        if (left && !previousLeft) { i += 0.01; applyPIDF(); }
        previousUp = up;
        previousDown = down;
        previousLeft = left;
        previousRight = right;

        // LIMELIGHT: lectura secundaria para validar el apuntado.
        LLResult result = limelight.getLatestResult();
        boolean visionValid = result != null && result.isValid()
                && result.getStaleness() >= 0
                && result.getStaleness() <= MAX_VISION_AGE_MS;
        double tx = visionValid ? result.getTx() : Double.NaN;
        double ty = visionValid ? result.getTy() : Double.NaN;
        double ta = visionValid ? result.getTa() : Double.NaN;
        boolean trackingOn = tracking != null && tracking.isEnabled();
        double distance = goalConfigured()
                ? Math.hypot(goalX() - drive.getX(), goalY() - drive.getY())
                : Double.NaN;
        boolean visionCentered = visionValid && Math.abs(tx) <= VISION_CENTER_TOLERANCE_DEG;

        telemetry.addLine("========== TABLA CALIBRACION ==========");
        telemetry.addData("Esquina", rightCorner ? "DERECHA" : "IZQUIERDA");
        telemetry.addData("Distancia odometria (in)", "%.2f", distance);
        telemetry.addData("RPM objetivo (ticks/s)", "%.1f", targetVelocity);
        telemetry.addData("Shooter real (ticks/s)", "%.1f", shooter.getVelocity());
        telemetry.addData("Hood preset", "%d - %s", hoodIndex + 1, HOOD_NAMES[hoodIndex]);
        telemetry.addData("Hood servo", "%.4f", HOOD_POSITIONS[hoodIndex]);
        telemetry.addData("Gate comando (deg logicos)", "%.1f", currentGateAngle);

        telemetry.addLine("========== VISION / DOBLE VALIDACION ==========");
        telemetry.addData("Limelight", visionValid ? "OBJETIVO VISIBLE (ID NO FILTRADO)" : "SIN OBJETIVO VALIDO");
        telemetry.addData("ID deseado", rightCorner ? RIGHT_TAG_ID : LEFT_TAG_ID);
        telemetry.addData("tx (deg)", "%.2f", tx);
        telemetry.addData("ty (deg)", "%.2f", ty);
        telemetry.addData("ta (%%)", "%.2f", ta);
        telemetry.addData("Vision centrada", visionCentered ? "SI" : "NO / SIN VISION");
        telemetry.addData("Odom objetivo configurado", goalConfigured() ? "SI" : "NO");
        telemetry.addData("Tracking odometria", trackingOn ? "ON" : "OFF");
        telemetry.addData("Validacion", "PENDIENTE: verificar ID y alineacion");

        telemetry.addLine("========== ROBOT / TORRETA ==========");
        telemetry.addData("Robot X", "%.2f", drive.getX());
        telemetry.addData("Robot Y", "%.2f", drive.getY());
        telemetry.addData("Heading (deg)", "%.2f", drive.getHeadingDegrees());
        telemetry.addData("Turret target (deg)", "%.2f", turret.getTargetAngleDeg());
        telemetry.addData("Turret actual (deg)", "%.2f", turret.getCurrentAngleDeg());
        telemetry.addData("Turret error (deg)", "%.2f", turret.getErrorDeg());
        telemetry.addData("Turret power", "%.3f", turret.getMotorPower());

        telemetry.addLine("========== CONTROLES / PIDF ==========");
        telemetry.addData("Intake", intakeEnabled ? "ON" : "OFF");
        telemetry.addData("Intake power", "%.2f", intake.getPower());
        telemetry.addData("Indexer power", "%.2f", indexer.getPower());
        telemetry.addData("Disparando", feeding ? "SI" : "NO");
        telemetry.addData("Shooter", shooterEnabled ? "ON" : "OFF");
        telemetry.addData("P / I / D / F", "%.3f / %.3f / %.3f / %.3f", p, i, d, f);
        telemetry.update();
    }

    private void selectCorner(boolean selectRight) {
        if (rightCorner == selectRight) return;
        rightCorner = selectRight;
        // No dejar un comando viejo apuntando a la esquina anterior.
        if (tracking != null) {
            tracking.disable();
            tracking = null;
        }
    }

    private void setHoodIndex(int index) {
        hoodIndex = Math.max(0, Math.min(HOOD_POSITIONS.length - 1, index));
        hood.setPosition(HOOD_POSITIONS[hoodIndex]);
    }

    private void setGateAngle(double degrees) {
        degrees = Math.max(0, Math.min(180, degrees));
        gate.setPosition(degrees / 180.0);
        currentGateAngle = degrees;
    }

    private void applyPIDF() {
        shooter.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void stop() {
        if (shooter != null) {
            shooter.setVelocity(0);
            shooter.setPower(0);
        }
        if (intake != null) intake.setPower(0);
        if (indexer != null) indexer.setPower(0);
        if (tracking != null) tracking.disable();
        if (turret != null) turret.stop();
        if (driveCommand != null) driveCommand.end();
        else if (drive != null) drive.stop();
        if (limelight != null) limelight.stop();
    }
}

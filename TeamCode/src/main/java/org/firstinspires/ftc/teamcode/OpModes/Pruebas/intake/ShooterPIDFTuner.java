package org.firstinspires.ftc.teamcode.OpModes.Pruebas.intake;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem1;
import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd1;

@TeleOp(name = "Shooter PIDF + Mecanum")
public class ShooterPIDFTuner extends OpMode {

    // Shooter
    private DcMotorEx shooter;

    // Chasis
    private DriveSubsystem1 drive;
    private MecanumDriveCmd1 driveCmd;
    private DcMotor intake;
    private DcMotor indexer;

    // Limelight
    private Limelight3A limelight;

    // Velocidad objetivo en ticks/segundo
    private double targetVelocity = 800;

    // PIDF inicial
    private double p = 4.0;
    private double i = 0.0;
    private double d = 0.5;
    private double f = 12.0;

    // Memoria de botones
    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastLeft = false;
    private boolean lastRight = false;
    private boolean lastA = false;

    private boolean shooterEnabled = false;

    @Override
    public void init() {

        // =========================
        // SHOOTER
        // =========================
        shooter = hardwareMap.get(DcMotorEx.class, "shoot");
        indexer = hardwareMap.get(DcMotor.class, "intake1");
        intake = hardwareMap.get(DcMotor.class, "intake2");

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPower(0);

        applyPIDF();

        // =========================
        // CHASIS MECANUM
        // =========================
        drive = new DriveSubsystem1();
        drive.init(hardwareMap);

        driveCmd = new MecanumDriveCmd1(drive, gamepad1);

        // =========================
        // LIMELIGHT
        // =========================
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.pipelineSwitch(0);
        //limelight.start();

        telemetry.addLine("Shooter + Mecanum inicializado");
        telemetry.addLine("Verifica las direcciones de los motores");
        telemetry.update();
    }
    @Override
    public void loop() {
        // Control del chasis
        driveCmd.execute();

        if (gamepad1.a && !lastA){
            shooterEnabled = !shooterEnabled;
        }
        lastA = gamepad1.a;

        // Ajustar la velocidad objetivo del shooter
        targetVelocity += gamepad1.right_trigger * 5.0;
        targetVelocity -= gamepad1.left_trigger * 5.0;
        targetVelocity = Math.max(0, Math.min(6000, targetVelocity));

        if (targetVelocity < 0){
            targetVelocity = 0;
        }

        // RT: solo shooter
//        if (gamepad1.right_trigger > 0.1 && targetVelocity > 0) {
//            shooter.setVelocity(targetVelocity);
//        } else {
//            shooter.setVelocity(0);
//        }

        // X: intake e indexer juntos
        if (gamepad1.y) {
            intake.setPower(-0.85);
            indexer.setPower(-0.85);

        } else if (gamepad1.x) {
            intake.setPower(0.85);
            indexer.setPower(0.85);
        } else {
            intake.setPower(0);
            indexer.setPower(0);
        }

        // Ajustes PIDF con la cruceta
        if (gamepad1.dpad_up && !lastUp) {
            p += 0.2;
        }
        if (gamepad1.dpad_down && !lastDown) {
            f += 0.5;
        }
        if (gamepad1.dpad_right && !lastRight) {
            d += 0.1;
        }
        if (gamepad1.dpad_left && !lastLeft) {
            i += 0.01;
        }

        if (shooterEnabled) {
            shooter.setVelocity(targetVelocity);
        }else {
            shooter.setVelocity(0);
        }

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastRight = gamepad1.dpad_right;
        lastLeft = gamepad1.dpad_left;

        //shooter.setVelocityPIDFCoefficients(p, i, d, f);

        telemetry.addData("Shooter activo", gamepad1.right_trigger > 0.1);
        telemetry.addData("Intake/Indexer activos", gamepad1.x);
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Velocidad objetivo", targetVelocity);
        telemetry.addData("Left Velocity", shooter.getVelocity());
        telemetry.addData("P", p);
        telemetry.addData("I", i);
        telemetry.addData("D", d);
        telemetry.addData("F", f);
        telemetry.update();
    }


    private void applyPIDF() {
        shooter.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void stop() {
        shooter.setPower(0);

        if (driveCmd != null) {
            driveCmd.reset();
        } else if (drive != null) {
            drive.stop();
        }

        if (limelight != null) {
            limelight.stop();
        }
    }
}

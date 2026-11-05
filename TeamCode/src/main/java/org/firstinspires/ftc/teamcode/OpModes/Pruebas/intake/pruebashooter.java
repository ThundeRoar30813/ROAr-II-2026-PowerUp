package org.firstinspires.ftc.teamcode.OpModes.Pruebas.intake;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Commands.drive.MecanumDriveCmd1;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem1;


@TeleOp
public class pruebashooter extends OpMode {

    private DcMotorEx shooter;

    // Chasis
    private DriveSubsystem1 drive;
    private MecanumDriveCmd1 driveCmd;
    private DcMotor intake;
    private DcMotor indexer;

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

        // Ajustar la velocidad objetivo del shooter
        // X: intake e indexer juntos
        if (gamepad1.x) {
            intake.setPower(-1.0);
            indexer.setPower(-1.0);

        } else if (gamepad1.y) {
            intake.setPower(1.0);
            indexer.setPower(1.0);

        } else if (gamepad1.a){
            shooter.setPower(1);
        } else {
            intake.setPower(0);
            indexer.setPower(0);
            shooter.setPower(0);
        }



        telemetry.update();
    }
}

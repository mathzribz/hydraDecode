package all.Main.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
@Disabled
public class DECODE_RobotCentric extends LinearOpMode {

    private DcMotor RMF, RMB, LMF, LMB, Intake, Shooter, Transfer;
    private Limelight3A limelight;
    private double driveSpeed = 0.8;
    public static double shooterSpeed = 0.65;
    public static double transferSpeed = 1;
    private static final double DEAD_ZONE = 0.2;


    @Override
    public void runOpMode() {
        initHardware();

        waitForStart();
        while (opModeIsActive()) {
            loc();
            intake();
            shooter();
            transfer();

            telemetry.addData("velocidade shooter", shooterSpeed);
            telemetry.addData("velocidade drive", driveSpeed);
            telemetry.update();
        }

    }

    private void initHardware() {
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");
        LMF = hardwareMap.get(DcMotor.class, "LMF");
        LMB = hardwareMap.get(DcMotor.class, "LMB");
        Intake = hardwareMap.get(DcMotor.class, "intake");
        Shooter = hardwareMap.get(DcMotor.class, "shooter");
        Transfer = hardwareMap.get(DcMotor.class, "transfer");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);
        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    private double applyDeadZone(double value) {
        return (Math.abs(value) > DEAD_ZONE) ? value : 0.0;
    }

    public void loc() {

        // Recupera o IMU do hardwareMap
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Define os parâmetros de orientação do Hub no robô
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        // Inicializa o IMU com os parâmetros definidos
        imu.initialize(parameters);
        if (gamepad1.dpad_right) {
            imu.resetYaw();
        }
        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x * 1.1;
        double turn = -gamepad1.right_stick_x;

        drive = applyDeadZone(drive);
        strafe = applyDeadZone(strafe);
        turn = applyDeadZone(turn);

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);

        double powerRMF = (drive - strafe - turn) / denominator;
        double powerRMB = (drive + strafe - turn) / denominator;
        double powerLMF = (drive + strafe + turn) / denominator;
        double powerLMB = (drive - strafe + turn) / denominator;

        RMF.setPower(powerRMF * driveSpeed);
        RMB.setPower(powerRMB * driveSpeed);
        LMF.setPower(powerLMF * driveSpeed);
        LMB.setPower(powerLMB * driveSpeed);


        if (gamepad1.dpad_left) {
            driveSpeed = 0.75;
        }
        if (gamepad1.x) {
            driveSpeed = 0.9;
        }


        telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles());
    }
    public void intake() {

        if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1  ) {
            Intake.setPower(0.6);
        } else {
            Intake.setPower(0);
        }

        if (gamepad1.left_bumper) {
            Intake.setPower(-0.6);
        }



    }

    public void transfer() {

        if (gamepad1.right_bumper || gamepad2.right_bumper  )  {
            Transfer.setPower(transferSpeed);
        } else {
            Transfer.setPower(0);
        }

        if (gamepad1.dpad_down || gamepad2.dpad_down ) {
            Transfer.setPower(-transferSpeed);
        }

    }

    public void shooter() {
        if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger  > 0.1 ) {
            Shooter.setPower(shooterSpeed);
        } else {
            Shooter.setPower(0);
        }

        if (gamepad1.a || gamepad2.a) {
            shooterSpeed = 0.65;
        }

        if (gamepad1.b || gamepad2.b ){
            shooterSpeed = 1;
        }

        if (gamepad1.y) {
            Shooter.setPower(-shooterSpeed);
            Transfer.setPower(-transferSpeed);
            Intake.setPower(-0.5);

        }

    }

//    public void ll() {
//        //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
//
//        LLResult llResult = limelight.getLatestResult();
//
//        if (llResult != null && llResult.isValid()) {
//
//            Pose3D botpose = llResult.getBotpose();
//
//            telemetry.addData("target X", llResult.getTx());
//            telemetry.addData("target Y", llResult.getTy());
//            telemetry.addData("target Area", llResult.getTa());
//            telemetry.addData(" botpose ", botpose.toString());
//        }

}

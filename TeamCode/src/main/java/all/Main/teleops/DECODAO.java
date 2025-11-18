
package all.Main.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
@TeleOp
public class DECODAO extends LinearOpMode {

    private DcMotor RMF, RMB, LMF, LMB, Intake, ShooterR ,ShooterL, Transfer;
    private Limelight3A limelight;
    private double driveSpeed = 0.8;
    public static double shooterSpeed = 0.65;
    public static double transferSpeed = 0.5;
    private static final double DEAD_ZONE = 0.2;
    private DistanceSensor dd;
    private String ch = "vazio";

    public static double kS = 10;
    public static double kV = 20;
    public static double kA = 0.5;


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
        ShooterR = hardwareMap.get(DcMotor.class, "shooter");
        ShooterL = hardwareMap.get(DcMotor.class, "shooter");
        Transfer = hardwareMap.get(DcMotor.class, "transfer");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        dd = hardwareMap.get(DistanceSensor.class, "dd");

        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);
        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        Transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        ShooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterL.setDirection(DcMotorSimple.Direction.REVERSE);


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


        double rawX = gamepad1.left_stick_x;
        double rawY = -gamepad1.left_stick_y; // Inverte Y pois no joystick o eixo positivo é para baixo
        double rawTurn = -gamepad1.right_stick_x;

        // Aplica a zona morta
        double x = applyDeadZone(rawX);
        double y = applyDeadZone(rawY);
        double turn = applyDeadZone(rawTurn);

        // Captura o ângulo de rotação do robô
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Converte os comandos do joystick para coordenadas relativas ao campo
        double fieldX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double fieldY = x * Math.sin(-heading) + y * Math.cos(-heading);

        // Compensação do strafe
        fieldX = fieldX * 1.1; // testar

        double denominator = Math.max(Math.abs(fieldY) + Math.abs(fieldX) + Math.abs(turn), 1);

        double frontLeftPower = (fieldY + fieldX + turn) / denominator;
        double backLeftPower = (fieldY - fieldX + turn) / denominator;
        double frontRightPower = (fieldY - fieldX - turn) / denominator;
        double backRightPower = (fieldY + fieldX - turn) / denominator;

        // Aplicar potências
        RMF.setPower(frontRightPower * driveSpeed);
        RMB.setPower(backRightPower * driveSpeed) ;
        LMF.setPower(frontLeftPower  * driveSpeed);
        LMB.setPower(backLeftPower  * driveSpeed);

        if (gamepad1.dpad_left) {
            driveSpeed = 0.75;
        }
        if (gamepad1.x) {
            driveSpeed = 0.9;
        }



        telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles());
    }

    public void intake() {

        if (gamepad1.left_trigger > 0.1 || gamepad1.right_bumper  ) {
            Intake.setPower(0.6);
        } else {
            Intake.setPower(0);
        }

        if (gamepad1.left_bumper) {
            Intake.setPower(-0.6);
        }



    }

    public void transfer() {
        double distance = dd.getDistance(DistanceUnit.CM);

        float lt = gamepad1.left_trigger;
        boolean rb = gamepad1.right_bumper;
        double threshold = 0.1;

        boolean prevRightBumper = false;




        if (distance <= 10) {
            ch = "cheio";
            Transfer.setPower(0);
        } else {

            if (rb && !prevRightBumper) {
                ch = "vazio";
                Transfer.setPower(0.5);
            }

            else if (lt > threshold && ch.equals("vazio")) {
                Transfer.setPower(0.5);
            }

            else {
                Transfer.setPower(0);
            }
        }

    }

    public void shooter() {
        SimpleMotorFeedforward feedforward =
                new SimpleMotorFeedforward(kS, kV, kA);

       double output = feedforward.calculate(kV,kA);

        if (gamepad1.right_trigger > 0.1) {
            ShooterR.setPower( output * shooterSpeed);
            ShooterL.setPower( output * shooterSpeed);
        } else {
            ShooterR.setPower(0);
            ShooterL.setPower(0);
        }

        if (gamepad1.a) {
            shooterSpeed = 0.65;
        }

        if (gamepad1.b) {
            shooterSpeed = 0.8;
        }

        if (gamepad1.y) {
            ShooterR.setPower(-shooterSpeed);
            Transfer.setPower(-transferSpeed);
            Intake.setPower(0.5);

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
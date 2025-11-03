
package all.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
@TeleOp
public class decodesDual extends LinearOpMode {

    private DcMotor RMF, RMB, LMF, LMB, Intake, Shooter, Transfer;
    private Limelight3A limelight;
    private IMU imu;
    private double driveSpeed = 0.8;
    public static double shooterSpeed = 0.9;


    @Override
    public void runOpMode() {
        initHardware();


        waitForStart();
        while (opModeIsActive()) {
            drive();
            intake();
            shooter();
            transfer();
            ll();
            imu();

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
        limelight = hardwareMap.get(Limelight3A.class, "ll");
        imu = hardwareMap.get(IMU.class, "imu");

        limelight.start();
        limelight.pipelineSwitch(2);

        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);
        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        Transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    private void drive() {
        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x * 1.1;
        double turn = -gamepad1.right_stick_x;

        drive = apllyDeadzone(drive);
        strafe = apllyDeadzone(strafe);
        turn = apllyDeadzone(turn);

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);

        double powerRMF = (drive - strafe - turn) / denominator;
        double powerRMB = (drive + strafe - turn) / denominator;
        double powerLMF = (drive + strafe + turn) / denominator;
        double powerLMB = (drive - strafe + turn) / denominator;

        RMF.setPower(powerRMF * driveSpeed);
        RMB.setPower(powerRMB * driveSpeed);
        LMF.setPower(powerLMF * driveSpeed);
        LMB.setPower(powerLMB * driveSpeed);

        if (gamepad1.dpad_up) {
            driveSpeed = 0.8;

        }

    }

    private double apllyDeadzone(double value) {
        return Math.abs(value) > 0.05 ? value : 0;
    }

    public void intake() {

        if (gamepad1.left_trigger > 0.1) {
            Intake.setPower(-1);
        } else {
            Intake.setPower(0);
        }

        if (gamepad1.left_bumper) {
            Intake.setPower(1);
        }

    }

    public void shooter() {
        if (gamepad2.right_trigger > 0.1) {
            Shooter.setPower(-shooterSpeed);
        } else {
            Shooter.setPower(0);
        }

        if (gamepad2.a) {
            shooterSpeed = 0.6;
        }

        if (gamepad2.b) {
            shooterSpeed = 0.73;
        }

        if (gamepad2.y) {
            shooterSpeed = 0.9;
        }

    }

    public void transfer() {

        if (gamepad2.right_bumper) {
            Transfer.setPower(1);
        } else {
            Transfer.setPower(0);
        }

        if (gamepad2.dpad_down) {
            Transfer.setPower(-1);
        }

    }

    public void ll() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {

            Pose3D botpose = llResult.getBotpose();

            telemetry.addData("target X", llResult.getTx());
            telemetry.addData("target Y", llResult.getTy());
            telemetry.addData("target Area", llResult.getTa());
            telemetry.addData(" botpose ", botpose.toString());
        }

    }

    public void imu() {

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));

        imu.initialize(parameters);

    }

}

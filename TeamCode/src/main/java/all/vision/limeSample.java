package all.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@TeleOp
public class limeSample extends OpMode {

    private Limelight3A limelight;
    private double distance;
    IMU imu;


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(3);
        imu = hardwareMap.get(IMU.class, "imu");

    }

    @Override
    public void start() {

        limelight.start();
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

    }

    @Override
    public void loop() {

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight.getLatestResult();
        if(llResult != null && llResult.isValid()){
            Pose3D botpose = llResult.getBotpose();
            telemetry.addData("Calculo distance ", distance);
            telemetry.addData("target X", llResult.getTx());
            telemetry.addData("target Area", llResult.getTa());
            telemetry.addData(" botpose ", botpose.toString());
        }

    }
}

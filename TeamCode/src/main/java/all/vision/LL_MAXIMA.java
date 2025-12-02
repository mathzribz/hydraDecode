package all.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
public class LL_MAXIMA extends OpMode {
    private Limelight3A limelight;
    IMU imu;

    public double distance;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        limelight.pipelineSwitch(3);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

    }

    @Override
    public void start() {
        limelight.start();

    }

    @Override
    public void loop() {

        YawPitchRollAngles yam = imu.getRobotYawPitchRollAngles();

        limelight.updateRobotOrientation(yam.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {

            Pose3D botPose = llResult.getBotpose();
            double area = llResult.getTa();
            double tX = llResult.getTx();
            double tY = llResult.getTy();

            telemetry.addData("distance", distance);
            telemetry.addData("botPose", botPose);
            telemetry.addData("area", area);
            telemetry.addData("tX", tX);
            telemetry.addData("tY", tY);
        }

        telemetry.update();
    }

}

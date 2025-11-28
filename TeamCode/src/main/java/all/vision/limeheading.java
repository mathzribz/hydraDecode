
package all.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Limelight Apriltag PID Tracker", group = "Vision")
@Config
public class limeheading extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;
    private Drivetrain drivetrain = new Drivetrain();

    public static double Kp = 0.05;  // Proportional gain
    public static double Ki = 0.0;   // Integral gain ( KEEP THIS 0 )
    public static double Kd = 0.0001  ;   // Derivative gain

    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        drivetrain.init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            LLResult result = limelight.getLatestResult();

            boolean validTarget = result != null && result.isValid();
            double tx = 0;

            if (validTarget) {
                tx = result.getTx();
            }

            double correction = PIDControl(0,(tx));

            drivetrain.power(correction);

            telemetry.addData("Target Found", validTarget);
            telemetry.addData("tx (deg)", tx);
            telemetry.addData("PID Output", correction);
            telemetry.update();
        }

        drivetrain.power(0);
        limelight.stop();
    }

    private double PIDControl(double reference, double state) {
        double error = angleWrap(reference - state);
        double dt = Math.max(timer.seconds(), 0.001);
        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (integralSum * Ki) + (derivative * Kd);
        return clamp(output, -0.5, 0.5);
    }

    private double angleWrap(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }


    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
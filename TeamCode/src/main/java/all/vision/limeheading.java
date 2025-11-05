package all.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import all.vision.Drivetrain;
import all.vision.PIDConstants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

@TeleOp(name = "Limelight Apriltag Tracker (No Helper)", group = "Vision")
public class limeheading extends LinearOpMode {

    private BNO055IMU imu;
    private Drivetrain drivetrain = new Drivetrain();

    double integralSum = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;
    double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        drivetrain.init(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            // Lê os dados JSON da Limelight
            JSONObject data = getLimelightData();

            double tx = 0;
            double ty = 0;
            double tv = 0;

            if (data != null) {
                tx = data.optDouble("tx", 0);
                ty = data.optDouble("ty", 0);
                tv = data.optDouble("tv", 0);
            }

            boolean valid = (tv == 1);

            double currentHeading = imu.getAngularOrientation(
                    AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

            double targetHeading = 0;

            if (valid) {
                // tx é o desvio angular em graus — converte para radianos
                targetHeading = Math.toRadians(tx);
            }

            double power = PIDControl(targetHeading, currentHeading);
            drivetrain.power(power);

            telemetry.addData("Target Found", valid);
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("Current Heading", Math.toDegrees(currentHeading));
            telemetry.addData("PID Output", power);
            telemetry.update();
        }
    }

    /**
     * Lê os dados JSON da Limelight via HTTP GET (docs oficiais da 3A).
     * Endpoint: http://limelight.local:5807/json
     */
    private JSONObject getLimelightData() {
        try {
            URL url = new URL("http://limelight.local:5807/json");
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            connection.setConnectTimeout(50); // milissegundos
            connection.setReadTimeout(50);

            BufferedReader in = new BufferedReader(new InputStreamReader(connection.getInputStream()));
            String inputLine = in.readLine();
            in.close();
            connection.disconnect();

            if (inputLine != null && !inputLine.isEmpty()) {
                JSONObject json = new JSONObject(inputLine);
                // dados do pipeline atual
                return json.getJSONObject("Results");
            }
        } catch (Exception e) {
            telemetry.addData("Limelight Error", e.getMessage());
        }
        return null;
    }

    private double PIDControl(double reference, double state) {
        double error = angleWrap(reference - state);
        double dt = timer.seconds();
        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;
        timer.reset();

        return (error * Kp) + (integralSum * Ki) + (derivative * Kd);
    }

    private double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}

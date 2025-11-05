package all.vision;

import com.acmerobotics.dashboard.FtcDashboard;
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
public class limeheading extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;
    private Drivetrain drivetrain = new Drivetrain();

    // PID constants
    private double Kp = PIDConstants.Kp;
    private double Ki = PIDConstants.Ki;
    private double Kd = PIDConstants.Kd;

    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Inicializa hardware
        drivetrain.init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        // Configuração da orientação do hub
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        // Telemetria combinada com o Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        limelight.pipelineSwitch(0); // pipeline configurado para AprilTags
        limelight.start();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            // Atualiza orientação do robô
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            // Lê resultado da Limelight
            LLResult result = limelight.getLatestResult();

            boolean validTarget = result != null && result.isValid();
            double tx = 0;

            if (validTarget) {
                tx = result.getTx(); // desvio lateral em graus
            }

            // Converte heading atual
            double currentHeading = orientation.getYaw(AngleUnit.RADIANS);
            double targetHeading = 0; // queremos centralizar a tag, logo o alvo é 0
            double correction = PIDControl(Math.toRadians(tx), currentHeading);

            // Aplica a correção de rotação
            drivetrain.power(correction);

            // Telemetria
            telemetry.addData("Target Found", validTarget);
            telemetry.addData("tx (deg)", tx);
            telemetry.addData("Current Heading (deg)", Math.toDegrees(currentHeading));
            telemetry.addData("PID Output", correction);
            telemetry.update();
        }

        drivetrain.power(0);
        limelight.stop();
    }

    /**
     * Controle PID básico para rotação
     */
    private double PIDControl(double reference, double state) {
        double error = angleWrap(reference - state);
        double dt = timer.seconds();
        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;
        timer.reset();

        // Saída final
        double output = (error * Kp) + (integralSum * Ki) + (derivative * Kd);
        return clamp(output, -0.5, 0.5); // limite de potência
    }

    /**
     * Normaliza ângulos para -π a π
     */
    private double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    /**
     * Garante que o valor está dentro dos limites
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

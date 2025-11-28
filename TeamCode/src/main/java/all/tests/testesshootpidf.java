package all.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Config
public class testesshootpidf extends LinearOpMode {

    private DcMotorEx AR, AL;
    private VoltageSensor battery;

    // PID constants
    public static double kP = 0.0040;
    public static double kI = 0.0;
    public static double kD = 0.00010;

    // Feedforward constants
    public static double kS = 0.15;
    public static double kV = 0.00125;
    public static double kA = 0.00008;

    public static double targetRPM = 3000;

    private PIDController pid = new PIDController(kP, kI, kD);
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

    @Override
    public void runOpMode() throws InterruptedException {

        AR = hardwareMap.get(DcMotorEx.class, "shooterR");
        AL = hardwareMap.get(DcMotorEx.class, "shooterL");
        battery = hardwareMap.voltageSensor.iterator().next();

        AR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        AR.setDirection(DcMotor.Direction.FORWARD);
        AL.setDirection(DcMotor.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while(opModeIsActive()) {


            double vr = AR.getVelocity();
            double vl = AL.getVelocity();
            double vAvg = (vr + vl) / 2.0;

            double pidPower = pid.calculate(vAvg, targetRPM);


            double ffPower = ff.calculate(targetRPM);

            // ajustar feedforward com compensação de voltagem
            double voltage = battery.getVoltage();
            double compensatedFF = ffPower * (12.0 / Math.max(10.0, voltage));

            double finalPower = pidPower + compensatedFF;


            if (gamepad1.right_trigger > 0.1) {
                AR.setPower(finalPower);
                AL.setPower(finalPower);
            } else {
                AR.setPower(0);
                AL.setPower(0);
                pid.reset();
            }



            if (gamepad1.a) {
                targetRPM = 2000;
            }
            if (gamepad1.b) {
                targetRPM = 1000;

            } if (gamepad1.x) {
                targetRPM = 500;
            }




            telemetry.addData("RPM avg", vAvg);
            telemetry.addData("targetRPM", targetRPM);
            telemetry.addData("PID", pidPower);
            telemetry.addData("FF", ffPower);
            telemetry.addData("FF compensado", compensatedFF);
            telemetry.addData("Voltagem", voltage);
            telemetry.addData("FinalPower", finalPower);
            telemetry.addData("RPM R", vr);
            telemetry.addData("RPM L", vl);
            telemetry.update();
        }
    }
}

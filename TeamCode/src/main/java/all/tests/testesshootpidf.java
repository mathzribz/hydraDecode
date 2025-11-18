
package all.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class testesshootpidf extends LinearOpMode {

    Servo pulse;
    private DcMotor AR, AL, KITL, KITR;

    public static double kP = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;
    public static double kS = 0.0;
    SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(kS, kV, kA);
    double result = feedforward.calculate(kV,kA);

    public static double speed = 0.5;



    public static double posPulseOpen = 0, posPulseClose = 0.58;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            AR = hardwareMap.get(DcMotor.class, "shooterR"); // porta 0
            AL = hardwareMap.get(DcMotor.class, "shooterL"); // porta 0



            AR.setDirection(DcMotor.Direction.FORWARD);
            AL.setDirection(DcMotor.Direction.FORWARD);


            SimpleMotorFeedforward feedforward =
                    new SimpleMotorFeedforward(kS, kV, kA);
            double result = feedforward.calculate(kV,kA);


            double output = result * speed;

     if (gamepad1.right_trigger > 0.1){
         AR.setPower(output );
         AL.setPower(output );
     }

            telemetry.addData("shooterR", AR.getPower());
            telemetry.addData("shooterL", AL.getPower());
            telemetry.update();
        }

    }

}
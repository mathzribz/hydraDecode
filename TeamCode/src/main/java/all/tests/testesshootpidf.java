
package all.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import dev.nextftc.ftc.ActiveOpMode;

@Config
@TeleOp
public class testesshootpidf extends LinearOpMode {

    VoltageSensor vs;
    private DcMotorEx AR, AL, KITL, KITR;

    public static double kP = 0.8;
    public static double kD = 0.0;
    public static double kI = 5;
    public static double kV = 10.0;
    public static double kA = 10.0;
    public static double kS = 10.0;
    public static double AA = 5;
    public static double VV = 5;

    public static double speed = 10
            ;

        SimpleMotorFeedforward feedforward =
                new SimpleMotorFeedforward(kS, kV, kA);

    PIDController pid = new PIDController(kP, kI, kD);


        double ff = feedforward.calculate(VV,AA) ;







    @Override
    public void runOpMode() throws InterruptedException {


        pid.setPID(kP,kI,kD);





        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {

            AR = hardwareMap.get(DcMotorEx.class, "shooterR"); // porta 0
            AL = hardwareMap.get(DcMotorEx.class, "shooterL"); // porta 0
            vs = hardwareMap.voltageSensor.iterator().next();

            double voltage = vs.getVoltage();

            AR.setDirection(DcMotor.Direction.REVERSE);
            AL.setDirection(DcMotor.Direction.REVERSE);

            double result = pid.calculate(AR.getCurrentPosition(), AL.getCurrentPosition() ) + ff;


     if (gamepad1.right_trigger > 0.1){
         AR.setVelocity(result * speed );
         AL.setVelocity(result * speed );


     }
     else {
         AR.setPower(0);
         AL.setPower(0);

     }

        if (voltage > 12.5 ){
            speed = 10;
        }


            telemetry.addData("shooterR", AR.getPower());
            telemetry.addData("shooterL", AL.getPower());
            telemetry.addData("VL ", AL.getVelocity());
            telemetry.addData("VR ", AR.getVelocity());

            telemetry.addData("voltage ", voltage);
            telemetry.addData("speed ", speed);


            telemetry.update();
        }

    }

}
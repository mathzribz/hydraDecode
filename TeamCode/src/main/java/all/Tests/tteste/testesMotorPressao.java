
package all.Tests.tteste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class testesMotorPressao extends LinearOpMode {

    Servo pulse;
    private DcMotor AR, AL, KITL, KITR;

    public static double posPulseOpen = 0, posPulseClose = 0.58;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            AR = hardwareMap.get(DcMotor.class, "shooterR"); // porta 0
            AL = hardwareMap.get(DcMotor.class, "shooterL"); // porta 0



            AR.setDirection(DcMotor.Direction.FORWARD);
            AL.setDirection(DcMotor.Direction.REVERSE);



            if (gamepad1.a ) {
                AR.setPower(1);
                AL.setPower(1);
            }

            if (gamepad1.a ) {
                AR.setPower(0);
                AL.setPower(0);
            }

            if (gamepad1.right_bumper){
                pulse.setPosition(posPulseClose);

            }


            else if (gamepad1.left_bumper){
                pulse.setPosition(posPulseOpen);

            }


            telemetry.addData("power AR", AR.getPower());
            telemetry.addData("ticks AR", AR.getCurrentPosition());

            telemetry.addData("power AL", AL.getPower());
            telemetry.addData("ticks Al", AL.getCurrentPosition());



            telemetry.update();

        }
    }
}
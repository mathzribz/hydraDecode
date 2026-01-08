
package all.tests;

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
public class testesMotor extends LinearOpMode {

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




            AR.setPower(gamepad1.right_stick_y );
            AL.setPower(gamepad1.right_stick_y );


            if (gamepad1.right_bumper){
                pulse.setPosition(posPulseClose);

            }


            else if (gamepad1.left_bumper){
                pulse.setPosition(posPulseOpen);

            }


            telemetry.addData("power AR", AR.getPower());
            telemetry.addData("ticks AR", AR.getCurrentPosition());


            telemetry.update();

        }
    }
}
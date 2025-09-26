package opmode.testes;

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
public class SHOOTER extends LinearOpMode {

    Servo pulse;
    private DcMotor AR, AL, KITL, KITR;

    public static double posPulseOpen = 0, posPulseClose = 0.58;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            AR = hardwareMap.get(DcMotor.class, "Kit"); // porta 0



            AR.setDirection(DcMotor.Direction.REVERSE);



            if(gamepad1.a){
            AR.setPower(1);
            }

            if(gamepad1.b){
                AR.setPower(0);
            }



            if (gamepad1.right_bumper){
                pulse.setPosition(posPulseClose);

            }


            else if (gamepad1.left_bumper){
                pulse.setPosition(posPulseOpen);

            }


            telemetry.addData("AR", AR.getPower());


            telemetry.update();

        }
    }
}
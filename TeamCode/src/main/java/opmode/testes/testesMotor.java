package opmode.testes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class testesMotor extends LinearOpMode {
    private DcMotor AR, AL, KITL, KITR;


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        while (opModeIsActive()) {
            AR = hardwareMap.get(DcMotor.class, "Kit"); // porta 0



            AR.setDirection(DcMotor.Direction.FORWARD);
            AL.setDirection(DcMotor.Direction.REVERSE);

            AR.setPower(gamepad1.right_stick_y);
            AL.setPower(gamepad2.right_stick_y);


            telemetry.addData("AR", AR.getPower());
            telemetry.addData("AL", AL.getPower());

            telemetry.update();

        }
    }
}
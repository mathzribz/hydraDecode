package all.tests;
import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Disabled
public class SHOOTER extends LinearOpMode {
    Servo pulse;
    private DcMotor AR, AL, KITL, KITR;


    private final GamepadManager g1 = PanelsGamepad.INSTANCE.getFirstManager();



    @Override
    public void runOpMode() throws InterruptedException {
        AR = hardwareMap.get(DcMotor.class, "Kit");


        waitForStart();
        while (opModeIsActive()) {
         g1.asCombinedFTCGamepad(gamepad1);





            AR.setDirection(DcMotor.Direction.REVERSE);



            if(gamepad1.a){
            AR.setPower(-1);
            }

            if(gamepad1.b){
                AR.setPower(0);
            }






            telemetry.addData("AR", AR.getPower());


            telemetry.update();

        }
    }
}
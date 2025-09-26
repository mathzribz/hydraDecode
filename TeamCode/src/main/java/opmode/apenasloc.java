
package opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class apenasloc extends LinearOpMode {

    private DcMotorEx RMF, RMB, LMF, LMB;
    private DcMotor viper;
    private Servo pulse, slide, claw, outtake;
    private CRServo intakeL, intakeR;

    private double speed = 0.5;

    public static double posSlideOpen = 0, posSlideClose = 1;
    public static double posPulseOpen = 0, posPulseClose = 0.58;
    public static double posOuttakeTransfer = 0, posOuttakeBasket = 0.58,
            posOuttakeSpecimen = 0.58, posOuttakeChamber = 0.58;
    public static double posClawOpen = 0, posClawClose = 0.58;

    private boolean clawOpen = false;
    private boolean lastButtonState = false;

    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {
            drive();


            if (gamepad1.a){
                LMF.setPower(1);
            }

            if (gamepad1.b){
                LMB.setPower(1);
            }


            if (gamepad1.x){
                RMF.setPower(1);
            }


            if (gamepad1.y){
                RMB.setPower(1);
            }




            telemetry.addData("Speed", speed);
            telemetry.update();
        }
    }

    private void initHardware() {
        RMF = hardwareMap.get(DcMotorEx.class, "RMF");
        RMB = hardwareMap.get(DcMotorEx.class, "RMB");
        LMF = hardwareMap.get(DcMotorEx.class, "LMF");
        LMB = hardwareMap.get(DcMotorEx.class, "LMB");


        RMF.setDirection(DcMotor.Direction.FORWARD);
        RMB.setDirection(DcMotor.Direction.FORWARD);
        LMF.setDirection(DcMotor.Direction.REVERSE);
        LMB.setDirection(DcMotor.Direction.REVERSE);


    }

    private void drive() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x * 1.1;
        double turn = gamepad1.right_stick_x;

        drive = applyDeadzone(drive);
        strafe = applyDeadzone(strafe);
        turn = applyDeadzone(turn);

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);

        double powerLMF = (drive + strafe + turn) / denominator;
        double powerLMB = (drive - strafe - turn) / denominator;
        double powerRMF = (drive - strafe - turn) / denominator;
        double powerRMB = (drive + strafe - turn) / denominator;

        RMF.setPower(powerRMF * speed);
        RMB.setPower(powerRMB * speed);
        LMF.setPower(powerLMF * speed);
        LMB.setPower(powerLMB * speed);
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < 0.05 ? 0 : value;
    }

}
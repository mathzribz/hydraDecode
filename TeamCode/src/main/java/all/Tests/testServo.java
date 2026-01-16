package all.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@TeleOp
public class testServo extends LinearOpMode {
    public Servo servoG,  servoA, servoP;
    public static double pos = 0;


    public void initservoA () {

        servoG = hardwareMap.get(Servo.class, "servo");


    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (opModeIsActive()) {
            initservoA();
            if (gamepad1.a) {
                servoG.setPosition(pos);

            }
            if (gamepad1.b) {
                servoG.setPosition(0);

            }






            telemetry.addData("pos servoG", servoG.getPosition());

            telemetry.update();

        }
    }}
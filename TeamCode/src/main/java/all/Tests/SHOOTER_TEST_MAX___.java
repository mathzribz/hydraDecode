package all.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class SHOOTER_TEST_MAX___ extends OpMode {

    private Servo angleServo;
    public static double initAngle = 0.05;
    public static double finalAngle = 0.83;

    @Override
    public void init() {
        angleServo = hardwareMap.get(Servo.class, "angle");

    }

    @Override
    public void loop() {

        if (gamepad1.a) angleServo.setPosition(initAngle);

        if (gamepad1.b) angleServo.setPosition(finalAngle);

        telemetry.addData("position", angleServo.getPosition());
        telemetry.addData("initAngle", initAngle);
        telemetry.addData("finalAngle", finalAngle);
        telemetry.update();
    }

}

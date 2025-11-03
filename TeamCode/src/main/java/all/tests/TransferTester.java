package all.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.hardware.impl.CRServoEx;

@TeleOp
public class TransferTester extends OpMode {

    private CRServoEx servo1;
    private CRServoEx servo2;
    public static double speed = 0.7;

    @Override
    public void init() {
        servo1 = hardwareMap.get(CRServoEx.class, "servo1");
        servo2 = hardwareMap.get(CRServoEx.class, "servo2");

    }

    @Override
    public void loop() {

        if (gamepad1.right_stick_y > 0.05) {
            servo1.setPower(speed);
            servo2.setPower(speed);
        } else {
            servo1.setPower(0);
            servo2.setPower(0);
        }

    }

}

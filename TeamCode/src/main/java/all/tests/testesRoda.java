
package all.tests;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class testesRoda extends LinearOpMode {
    private final GamepadManager g1 = PanelsGamepad.INSTANCE.getFirstManager();

    private DcMotor RMF, RMB, LMF, LMB;
    private double speed = 1;

    @Override
    public void runOpMode() {
        initHardware();

        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);
        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            g1.asCombinedFTCGamepad(gamepad1);
            drive();

            if (gamepad1.a) {
                speed = 0.25;

            }

            if (gamepad1.b) {
                speed = 0.5;

            }

            if (gamepad1.x) {
                speed = 0.75;

            }

            if (gamepad1.y) {
                speed = 1;

            }


            if (gamepad1.dpad_down) {
                RMF.setPower(speed);


            }

            if (gamepad1.dpad_right) {
                RMB.setPower(speed);

            }

            if (gamepad1.dpad_left) {
                LMF.setPower(speed);

            }

            if (gamepad1.dpad_up) {
                LMB.setPower(speed);

            }



            telemetry.update();
        }

    }

    private void initHardware() {
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");
        LMF = hardwareMap.get(DcMotor.class, "LMF");
        LMB = hardwareMap.get(DcMotor.class, "LMB");

    }

    private void drive() {
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x * 1.1;
        double turn = gamepad1.right_stick_x;

        drive = apllyDeadzone(drive);
        strafe = apllyDeadzone(strafe);
        turn = apllyDeadzone(turn);

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);

        double powerRMF = (drive - strafe - turn) / denominator;
        double powerRMB = (drive + strafe - turn) / denominator;
        double powerLMF = (drive + strafe + turn) / denominator;
        double powerLMB = (drive - strafe + turn) / denominator;

        RMF.setPower(powerRMF * speed);
        RMB.setPower(powerRMB * speed);
        LMF.setPower(powerLMF * speed);
        LMB.setPower(powerLMB * speed);

    }

    private double apllyDeadzone(double value) {
        return Math.abs(value) > 0.05 ? value : 0;
    }

}
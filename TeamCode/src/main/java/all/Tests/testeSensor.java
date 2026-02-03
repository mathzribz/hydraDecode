
package all.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp
public class testeSensor extends LinearOpMode {

    Servo pulse;
    private DcMotor AR, AL, KITL, KITR;

    private DistanceSensor up, down;

    public static double posPulseOpen = 0, posPulseClose = 0.58;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            AR = hardwareMap.get(DcMotor.class, "shooterR"); // porta 0
            AL = hardwareMap.get(DcMotor.class, "shooterL"); // porta 0

            up = hardwareMap.get(DistanceSensor.class, "up");
            down = hardwareMap.get(DistanceSensor.class, "down");

            double distanceUp = up.getDistance(DistanceUnit.CM);

            double distanceDown = down.getDistance(DistanceUnit.CM);




            AR.setDirection(DcMotor.Direction.REVERSE);
            AL.setDirection(DcMotor.Direction.FORWARD);




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

            telemetry.addData("power AL", AL.getPower());
            telemetry.addData("ticks Al", AL.getCurrentPosition());

            telemetry.addData("distance up", distanceUp);
            telemetry.addData("distance down", distanceDown);



            telemetry.update();

        }
    }
}
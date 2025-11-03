package all.tests;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class AprilTest extends OpMode {

    Limelight3A limelight;
    private DcMotor motor;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        motor = hardwareMap.get(DcMotor.class, "Kit");

    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s", status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), status.getFps());
        telemetry.addData("Pipeline:", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {

            double tx = result.getTx();
            double ty = result.getTy();
            telemetry.addData("Target X", tx);
            telemetry.addData("Target y", ty);

            if (tx > 5) { // faz a limelight rastrear a aprilTag
                motor.setPower(0.1);
            }else if (tx < -5) {
                motor.setPower(-0.1);
            }else{
                motor.setPower(0);
            }

        } else {
            telemetry.addData("Limelight", "No Targets");
        }


    }
}

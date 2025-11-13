
package all.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp
public class testeDistanceTransfer extends LinearOpMode {

    Servo pulse;
    private DcMotor AR, AL, KITL, KITR;
    DistanceSensor dd;
    String ch = "vazio";


    public static double posPulseOpen = 0, posPulseClose = 0.58;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());



        waitForStart();
        while (opModeIsActive()) {
            AR = hardwareMap.get(DcMotor.class, "transfer"); // porta 0

            dd = hardwareMap.get(DistanceSensor.class, "dd"); // porta 0
            double distance = dd.getDistance(DistanceUnit.CM);


            AR.setDirection(DcMotor.Direction.FORWARD);


            boolean prevRightBumper = false;

// dentro do loop principal
            float lt = gamepad1.left_trigger;   // 0.0 .. 1.0
            boolean rb = gamepad1.right_bumper;
            float threshold = 0.1f;             // evita ruído do trigger

// 1) se identificou, bloqueia e garante motor parado
            if (distance <= 10) {
                ch = "cheio";
                AR.setPower(0f);
            } else {
                // 2) evento: pressionou o right bumper agora (borda de subida)
                if (rb && !prevRightBumper) {
                    ch = "vazio";         // libera o uso do trigger novamente
                    AR.setPower(0.5f);    // gira ao clicar no RB (um pulso)
                }
                // 3) se trigger está sendo pressionado e estado permite, roda proporcional
                else if (lt > threshold && ch.equals("vazio")) {
                    AR.setPower(0.5f);
                }
                // 4) caso contrário, garante que esteja parado
                else {
                    AR.setPower(0f);
                }
            }

// atualizar estado anterior do botão
            prevRightBumper = rb;







            telemetry.addData("AR", AR.getPower());
            telemetry.addData("distance", dd.getDistance(DistanceUnit.CM) );
            telemetry.addData("bola", ch );


            telemetry.update();


            }







        }


    }

package opmode.testes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;

@TeleOp(name = "PIDF Tuner", group = "Tuning")
public class PIDFTuner extends LinearOpMode {

    private DcMotor motor;
    private PIDFController controller;

    // PIDF Coeficientes (iniciais)
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    // Posição alvo
    public static int targetPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotor.class, "motor"); // nome do motor

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         controller = new PIDFController(kP, kI, kD, new StaticFeedforward(kF));

        telemetry.addLine("Pronto para tunar PIDF");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Atualiza coeficientes


            // Posição atual do encoder
            int currentPosition = motor.getCurrentPosition();

            // Calcula a saída do controlador
            double output = controller.calculate(currentPosition, targetPosition);

            // Aplica no motor
            motor.setPower(output);

            // Mostra no telemetry
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Current", currentPosition);
            telemetry.addData("Output", output);
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);
            telemetry.update();
        }
    }
}
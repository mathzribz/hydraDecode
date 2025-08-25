package opmode.testes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import config.pedro.constants.FConstants;
import config.pedro.constants.LConstants;
@Config
@TeleOp
public class feirasenai extends LinearOpMode {
    private DcMotor AR, RMF, RMB, LMF, LMB;
    private Servo servoG, slide;
    private CRServo intakeL, intakeR;
    double speed = 0.8;

    public static double posOpen = 0;
    public static double posClose = 0.58;

    boolean lastShareState = false;

    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();
        while (opModeIsActive()) {
            initt();
            loc();


            /** This method is call once when init is played, it initializes the follower **/

            if (gamepad1.left_trigger > 0.1) {
                intakeL.setPower(-1);
                intakeR.setPower(1);

            }

            else if (gamepad1.right_trigger > 0.1 ) {
                intakeL.setPower(1);
                intakeR.setPower(-1);
            }
            else {
                intakeL.setPower(0);
                intakeR.setPower(0);
            }



            if (gamepad1.dpad_up ) {
                AR.setPower(1);

            }

            else if (gamepad1.dpad_down ) {
                AR.setPower(-1);

            }
            else {
                AR.setPower(0);

            }





            if (gamepad1.left_bumper) {
                slide.setPosition(1);
                servoG.setPosition(0.58);


            }
            if (gamepad1.right_bumper) {
                servoG.setPosition(0);
                slide.setPosition(0);

            }



            telemetry.addData("AR", AR.getPower());
            telemetry.addData("garra", servoG.getPosition());
            telemetry.addData("speed", speed);


            telemetry.update();

        }



    }

    public void initt() {
        RMF = hardwareMap.get(DcMotorEx.class, "RMF"); // porta 1 expension
        RMB = hardwareMap.get(DcMotorEx.class, "RMB"); // porta 2 expension
        LMF = hardwareMap.get(DcMotorEx.class, "LMF"); // porta 1 control
        LMB = hardwareMap.get(DcMotorEx.class, "LMB"); // porta 2 control

        AR = hardwareMap.get(DcMotor.class, "Kit"); // porta 0
        servoG = hardwareMap.get(Servo.class, "pulse"); // porta 0
        slide = hardwareMap.get(Servo.class, "slide"); // porta 0
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");

        RMF.setDirection(DcMotor.Direction.FORWARD);
        RMB.setDirection(DcMotor.Direction.FORWARD);
        LMF.setDirection(DcMotor.Direction.REVERSE);
        LMB.setDirection(DcMotor.Direction.REVERSE);

        AR.setDirection(DcMotor.Direction.FORWARD);
    }


    public void loc() {

        // Leitura dos controles do joystick
        double drive = -gamepad1.left_stick_y; // Movimento para frente/trás
        double strafe = gamepad1.left_stick_x; // Movimento lateral (strafe)
        double turn = gamepad1.right_stick_x; // Rotação


        // Fator de compensação para o strafe
        double strafeCompensation = 1.1; // Ajuste para melhorar a resposta do strafe
        strafe *= strafeCompensation;

        // Zona morta para evitar pequenos movimentos no joystick
        double deadzone = 0.05;
        if (Math.abs(turn) < deadzone) turn = 0;
        if (Math.abs(strafe) < deadzone) strafe = 0;
        if (Math.abs(drive) < deadzone) drive = 0;

        // Mudar angulo com base na aliança




        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);

        double powerLMF = (drive + strafe + turn) / denominator;
        double powerLMB = (drive - strafe + turn) / denominator;
        double powerRMF = (drive - strafe - turn) / denominator;
        double powerRMB = (drive + strafe - turn) / denominator;

        // Alternar velocidade
        boolean fastMode = false; // Variável global
        if (gamepad1.share && !lastShareState) { // Detecta apenas a transição do botão
            fastMode = !fastMode; // Alterna entre rápido e lento
        }
        speed = fastMode ? 1.0 : 0.75; // Define a velocidade

        // Aplicação das potências com ajuste de velocidade
        RMF.setPower(powerRMF * speed);
        RMB.setPower(powerRMB * speed);
        LMF.setPower(powerLMF * speed);
        LMB.setPower(powerLMB * speed);
    }
}
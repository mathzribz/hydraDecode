package opmode.testes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class feirasenai extends LinearOpMode {

    private DcMotorEx RMF, RMB, LMF, LMB;
    private DcMotor viper;
    private Servo pulse, slide, claw, outtake;
    private CRServo intakeL, intakeR;

    private double speed = 0.75;

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
            handleIntake();
            handleOuttake();

            telemetry.addData("Viper Power", viper.getPower());
            telemetry.addData("Pulse", pulse.getPosition());
            telemetry.addData("Slide", slide.getPosition());
            telemetry.addData("Outtake", outtake.getPosition());
            telemetry.addData("Claw", claw.getPosition());
            telemetry.addData("Speed", speed);
            telemetry.update();
        }
    }

    private void initHardware() {
        RMF = hardwareMap.get(DcMotorEx.class, "RMF");
        RMB = hardwareMap.get(DcMotorEx.class, "RMB");
        LMF = hardwareMap.get(DcMotorEx.class, "LMF");
        LMB = hardwareMap.get(DcMotorEx.class, "LMB");

        viper = hardwareMap.get(DcMotor.class, "Kit");

        pulse = hardwareMap.get(Servo.class, "pulse");
        slide = hardwareMap.get(Servo.class, "slide");
        outtake = hardwareMap.get(Servo.class, "outtake");
        claw = hardwareMap.get(Servo.class, "claw");

        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");

        RMF.setDirection(DcMotor.Direction.FORWARD);
        RMB.setDirection(DcMotor.Direction.FORWARD);
        LMF.setDirection(DcMotor.Direction.REVERSE);
        LMB.setDirection(DcMotor.Direction.REVERSE);

        viper.setDirection(DcMotor.Direction.FORWARD);
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
        double powerLMB = (drive - strafe + turn) / denominator;
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

    private void handleIntake() {
        if (gamepad1.left_trigger > 0.4) {
            slide.setPosition(posSlideClose);
            pulse.setPosition(posPulseClose);
        } else {
            slide.setPosition(posSlideOpen);
            pulse.setPosition(posPulseOpen);
        }

        if (gamepad1.right_trigger > 0.1) {
            intakeL.setPower(-0.8);
            intakeR.setPower(0.8);
        } else if (gamepad1.left_bumper) {
            intakeL.setPower(0.8);
            intakeR.setPower(-0.8);
        } else {
            intakeL.setPower(0);
            intakeR.setPower(0);
        }
    }

    private void handleOuttake() {
        if (gamepad1.dpad_up) {
            viper.setPower(1);
        } else if (gamepad1.dpad_down) {
            viper.setPower(-1);
        } else {
            viper.setPower(0);
        }

        if (gamepad1.a) outtake.setPosition(posOuttakeTransfer);
        else if (gamepad1.b) outtake.setPosition(posOuttakeBasket);
        else if (gamepad1.x) outtake.setPosition(posOuttakeSpecimen);
        else if (gamepad1.y) outtake.setPosition(posOuttakeChamber);

        boolean buttonPressed = gamepad1.right_bumper;
        if (buttonPressed && !lastButtonState) {
            clawOpen = !clawOpen;
            claw.setPosition(clawOpen ? posClawOpen : posClawClose);
        }
        lastButtonState = buttonPressed;
    }
}
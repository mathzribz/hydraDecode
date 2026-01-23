
package all.Main.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp
public class DECODAO_BLUE_ENGAJA extends LinearOpMode {

    private DcMotor RMF, RMB, LMF, LMB;
    private DcMotor Intake;
    private DcMotorEx ShooterR, ShooterL;
    private Servo porta, capuz;
    private DistanceSensor distanceSensor;
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;
    private IMU imu;
    private VoltageSensor vs;

    // VALORES
    private double driveSpeed = 0.85;
    private double headingOffset = 0.0;
    private static final double DEAD_ZONE = 0.25;

    public static double kP = 0.00065;
    public static double kI = 0.0;
    public static double kD = 0.00001;
    public static double kF = 0.0015;

    public static double TICKS_PER_REV = 28;
    public static double targetRPM = 1200;
    public static double targetTPS ;
    public static double finalPower;


    // =============== LIMELIGHT TRACKER VARIÁVEIS ===============
    public static double LL_Kp = 0.075;
    public static double LL_Ki = 0.0;
    public static double LL_Kd = 0.004;

    private double llIntegral = 0;
    private double llLastError = 0;
    private ElapsedTime llTimer = new ElapsedTime();
    private Limelight3A limelightLL;

    double shooterS;

    private final PIDFController pidf = new PIDFController(kP, kI, kD, kF);


    @Override
    public void runOpMode() {


        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        initHardware();

        waitForStart();

        while (opModeIsActive()) {


            loc();
            intake();
            shooter();

            telemetry.addData("Drive Speed", driveSpeed);

            telemetry.update();
        }

    }

    private void initHardware() {

        // MOTORS
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");
        LMF = hardwareMap.get(DcMotor.class, "LMF");
        LMB = hardwareMap.get(DcMotor.class, "LMB");

        Intake = hardwareMap.get(DcMotor.class, "intake");

        ShooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        ShooterL = hardwareMap.get(DcMotorEx.class, "shooterL");

        porta = hardwareMap.get(Servo.class, "gate");
        capuz = hardwareMap.get(Servo.class, "capuz");

        // SENSORES

//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        vs = hardwareMap.voltageSensor.iterator().next();

        // DIRECTIONS
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);
        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        ShooterR.setDirection(DcMotorSimple.Direction.FORWARD);
        ShooterL.setDirection(DcMotorSimple.Direction.REVERSE);



        limelightLL = hardwareMap.get(Limelight3A.class, "limelight");
//        limelightLL.pipelineSwitch(0);
//        limelightLL.start();
    }

    // DEADZONE
    private double applyDeadZone(double v) {
        return Math.abs(v) > DEAD_ZONE ? v : 0;
    }

    // FIELD CENTRIC
    public void loc() {

        double currentHeadingRad = pinpoint.getHeading(AngleUnit.RADIANS);
        double botHeading = currentHeadingRad - headingOffset;

        double x  = applyDeadZone(gamepad1.left_stick_x);   // x
        double y  = applyDeadZone(-gamepad1.left_stick_y);   // FORWARD/BACKWARD
        double rx = applyDeadZone(-gamepad1.right_stick_x);   // ROTATION

        // --- LIMELIGHT TRACKING (opcional) ---
//        if (gamepad1.right_trigger > 0.15) {
//            LLResult res = limelightLL.getLatestResult();
//            if (res != null && res.isValid()) {
//                double tx = res.getTx();
//                rx = LL_PID(tx, -3);
//            }
//        } else {
//            llIntegral = 0;
//            llLastError = 0;
//        }

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double powerLMF = (rotY + rotX + rx) / denominator;
        double powerLMB = (rotY - rotX + rx) / denominator;
        double powerRMF = (rotY - rotX - rx) / denominator;
        double powerRMB = (rotY + rotX - rx) / denominator;


        LMF.setPower(powerLMF * driveSpeed);
        LMB.setPower(powerLMB * driveSpeed);
        RMF.setPower(powerRMF * driveSpeed);
        RMB.setPower(powerRMB * driveSpeed);

        telemetry.addData("LMF power", LMF.getPower());
        telemetry.addData("RMF power", RMF.getPower());
        telemetry.addData("LMB power", LMB.getPower());
        telemetry.addData("RMB power",RMB.getPower());
        telemetry.addData("pionpoint YAW ",pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("pionpoint Y ",pinpoint.getEncoderY());
        telemetry.addData("pionpoint scalar ",pinpoint.getYawScalar());

        if (gamepad1.left_stick_button) driveSpeed = 0.9;

        if (gamepad1.right_stick_button) driveSpeed = 0.6;

        if (gamepad1.dpad_right){
            headingOffset = currentHeadingRad;

        }
    }

    public void intake() {

        // INTAKE GAMEPAD
        if (gamepad1.left_trigger > 0.1) {
            Intake.setPower(0.85);
        } else if (gamepad1.right_bumper) {
            Intake.setPower(-0.75);
        } else {
            Intake.setPower(0);
        }


        if(gamepad1.left_bumper){
            Intake.setPower(1);

        }


        if(gamepad1.dpad_left){
            porta.setPosition(0);
        }
        else if(gamepad1.dpad_right){
            double pos = 0.5;
            porta.setPosition(pos);
        }

    }




    public void shooter() {

        double vl = ShooterL.getVelocity();


        targetTPS = (targetRPM / 60.0) * TICKS_PER_REV;


        if (gamepad1.x || gamepad2.x) {
            targetRPM = 1200;
            shooterS = 0.35;

        }
        if (gamepad1.a || gamepad2.a) {
            targetRPM = 1350;
            shooterS = 0.55;

        } if (gamepad1.b || gamepad2.b) {
            targetRPM = 1600;
            shooterS = 0.75;
        }
        double pidPower = pidf.calculate(vl, targetTPS);


        // ajustar feedforward com compensação de voltagem
        double voltage = vs.getVoltage();
        // double compensatedFF = ffPower * (12.0 / Math.max(10.0, voltage));

        finalPower = pidPower;

        finalPower = Math.max(-1.0, Math.min(1.0, finalPower));

        if ( gamepad1.right_trigger > 0.1) {
            ShooterR.setPower(shooterS);
            ShooterL.setPower(shooterS);
        } else {
            ShooterR.setPower(0);
            ShooterL.setPower(0);
//            pidf.reset();
        }

        if (gamepad1.y) {
            ShooterR.setPower(-finalPower );
            ShooterL.setPower(-finalPower  );
            Intake.setPower(0.3);
        }

        if(gamepad1.dpad_down){
            capuz.setPosition(0);
        }
        else if(gamepad1.dpad_up){
            double pos2 = 0.5;
            capuz.setPosition(pos2);
        }

        telemetry.addData("RPM ", vl);
        telemetry.addData("targetRPM", targetRPM);
        telemetry.addData("PID", pidPower);

        telemetry.addData("Voltagem", voltage);
        telemetry.addData("FinalPower", finalPower);


    }

    private double LL_PID(double target, double state) {
        double error = target - state;
        double dt = Math.max(llTimer.seconds(), 0.001);

        llIntegral += error * dt;
        double derivative = (error - llLastError) / dt;

        llLastError = error;
        llTimer.reset();

        double output = (LL_Kp * error) + (LL_Ki * llIntegral) + (LL_Kd * derivative);
        return clamp(output, -0.5, 0.5);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

}

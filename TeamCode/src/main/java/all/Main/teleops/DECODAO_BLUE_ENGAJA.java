
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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    private Servo porta, capuz, servo_teste;
    private DistanceSensor up, down;
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;
    private IMU imu;
    private VoltageSensor vs;

    // VALORES
    private double driveSpeed = 0.85;
    private double headingOffset = 0.0;
    private static final double DEAD_ZONE = 0.25;

    public static double kP = 0.0007;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0003455;

    public static double TICKS_PER_REV = 28;
    public static double targetRPM = 1200;
    public static double targetTPS ;
    public static double finalPower;
    public static double servo1pos = 0.2;
    public static double servoPos = 0.3;


    // =============== LIMELIGHT TRACKER VARIÁVEIS ===============
    public static double LL_Kp = 0.075;
    public static double LL_Ki = 0.0;
    public static double LL_Kd = 0.004;

    private double llIntegral = 0;
    private double llLastError = 0;


    private Limelight3A limelightLL;

    double shooterS;

    boolean EnabledTransfer = true;
    ElapsedTime fullTimer = new ElapsedTime();
    boolean countingFull = false;


    private final PIDFController pidf = new PIDFController(kP, kI, kD, kF);


    @Override
    public void runOpMode() {


        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        initHardware();



        waitForStart();

        while (opModeIsActive()) {
        pinpoint.update();

            telemetry.addData("pionpoint YAW ", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("pionpoint Y ", pinpoint.getEncoderY());
            telemetry.addData("pionpoint X ", pinpoint.getEncoderX());

            shooter();

            telemetry.update();

        }

    }

    private void initHardware() {

        // MOTORS


        ShooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        ShooterL = hardwareMap.get(DcMotorEx.class, "shooterL");



        // SENSORES

//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        vs = hardwareMap.voltageSensor.iterator().next();

        // DIRECTIONS


        ShooterR.setDirection(DcMotorSimple.Direction.FORWARD);
        ShooterL.setDirection(DcMotorSimple.Direction.REVERSE);


        ShooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);





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



        if (gamepad1.left_stick_button) driveSpeed = 0.9;

        if (gamepad1.right_stick_button) driveSpeed = 0.6;

        if (gamepad1.dpad_right){
            headingOffset = currentHeadingRad;

        }
    }

    public void intake() {

        double distanceUp   = up.getDistance(DistanceUnit.CM);
        double distanceDown = down.getDistance(DistanceUnit.CM);

        boolean upBlocked   = distanceUp < 8;
        boolean downBlocked = distanceDown < 8;

        double intakePower = 0.0;

        if (gamepad1.right_bumper) {
            intakePower = -0.75;
        }

        if (upBlocked && downBlocked) {

            if (!countingFull) {
                fullTimer.reset();
                countingFull = true;
            }

            if (fullTimer.seconds() >= 0.1) {
                EnabledTransfer = false;
            }

        } else {
            countingFull = false;
            fullTimer.reset();
        }

        if (!gamepad1.right_bumper) {
            if (gamepad1.left_trigger > 0.08 && EnabledTransfer) {
                intakePower = 0.9;
            }
        }

        if (gamepad1.left_bumper) {
            EnabledTransfer = true;
            countingFull = false;
            fullTimer.reset();
            intakePower = 1.0;
        }

        if (gamepad1.dpad_left) {
            porta.setPosition(0);
        } else if (gamepad1.dpad_right) {
            porta.setPosition(0.5);
        }

        Intake.setPower(intakePower);


    }


    public void shooter() {

        pidf.setPIDF(kP, kI, kD, kF);

        double currentTPS = ShooterL.getVelocity();
        targetTPS = (targetRPM / 60.0) * TICKS_PER_REV;

        double ff = targetTPS * kF;
        double pid = pidf.calculate(currentTPS, targetTPS);

        finalPower = ff + pid;
        finalPower = clamp(finalPower, -1.0, 1.0);

        if (gamepad1.right_trigger > 0.1) {
            ShooterR.setPower(finalPower);
            ShooterL.setPower(finalPower);
          ;
        } else {
            ShooterR.setPower(0);
            ShooterL.setPower(0);
            pidf.reset();

        }

        double rpm = (currentTPS / TICKS_PER_REV) * 60.0;

        telemetry.addData("RPM", rpm);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("PID", pid);
        telemetry.addData("FF", ff);
        telemetry.addData("Final Power", finalPower);
    }


    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

}

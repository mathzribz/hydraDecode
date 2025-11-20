
package all.Main.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
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
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp
public class DECODAO extends LinearOpMode {

    private DcMotor RMF, RMB, LMF, LMB;
    private DcMotor Intake, Transfer;
    private DcMotorEx ShooterR, ShooterL;

    private Limelight3A limelight;
    private IMU imu;
    private DistanceSensor dd;
    private VoltageSensor vs;

    private double driveSpeed = 0.8;
    public static double shooterSpeed = 12.0;

    private static final double DEAD_ZONE = 0.2;

    // PIDF DE DASHBOARD
    public static double kP = 0.8;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double kS = 0.1;
    public static double kV = 0.0022;
    public static double kA = 0.0;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    private PIDController pid = new PIDController(kP, kI, kD);

    @Override
    public void runOpMode() {
        initHardware();

        waitForStart();

        while (opModeIsActive()) {
            loc();
            intake();
            transfer();
            shooter();

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

        Transfer = hardwareMap.get(DcMotor.class, "transfer");

        dd = hardwareMap.get(DistanceSensor.class, "dd");
        vs = hardwareMap.voltageSensor.iterator().next();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // DIRECTIONS
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);
        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake.setDirection(DcMotorSimple.Direction.FORWARD);

        ShooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterL.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(params);

        // RESET YAW
        imu.resetYaw();
    }

    // DEADZONE
    private double applyDeadZone(double v) {
        return Math.abs(v) > DEAD_ZONE ? v : 0;
    }

    // FIELD CENTRIC
    public void loc() {

        if (gamepad1.dpad_right) imu.resetYaw();

        double x = applyDeadZone(gamepad1.left_stick_x);
        double y = -applyDeadZone(gamepad1.left_stick_y);
        double turn = -applyDeadZone(gamepad1.right_stick_x);

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double fieldX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double fieldY = x * Math.sin(-heading) + y * Math.cos(-heading);

        fieldX *= 1.1; // compensate strafe

        double denom = Math.max(Math.abs(fieldY) + Math.abs(fieldX) + Math.abs(turn), 1);

        double fl = (fieldY + fieldX + turn) / denom;
        double bl = (fieldY - fieldX + turn) / denom;
        double fr = (fieldY - fieldX - turn) / denom;
        double br = (fieldY + fieldX - turn) / denom;

        LMF.setPower(fl * driveSpeed);
        LMB.setPower(bl * driveSpeed);
        RMF.setPower(fr * driveSpeed);
        RMB.setPower(br * driveSpeed);

        // DRIVE SPEED TOGGLE
        if (gamepad1.dpad_left) driveSpeed = 0.75;
        if (gamepad1.x) driveSpeed = 0.9;

        telemetry.addData("Yaw", heading);
    }

    private boolean transferEnabled = true;

    public void intake() {

        // Intake normal
        if (gamepad1.left_trigger > 0.1) {
            Intake.setPower(0.6);
        } else if (gamepad1.left_bumper) {
            Intake.setPower(-0.6);
        } else {
            Intake.setPower(0);
        }
    }

    public void transfer() {

        double distance = dd.getDistance(DistanceUnit.CM);
        boolean ballDetected = (distance <= 1.0);

        // Se detectou bola → trava o transfer controlado pelo trigger
        if (ballDetected) {
            transferEnabled = false;
        }


        // 1) RB → libera e roda ao mesmo tempo
        if (gamepad1.right_bumper) {
            transferEnabled = true;
            Transfer.setPower(1);
            return;
        }

        // 2) LT → roda o transfer apenas se estiver liberado e sem bola
        if (gamepad1.left_trigger > 0.1 && transferEnabled && !ballDetected) {
            Transfer.setPower(0.7);
        }
        else {
            Transfer.setPower(0);
        }
    }


    public double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(v, max));
    }

    public void shooter() {

        // Atualiza PID e Feedforward do dashboard
        pid.setPID(kP, kI, kD);
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);

        // Velocidade alvo → shooterSpeed * 100 é só exemplo
        double targetVelocity = shooterSpeed;

        // Velocidade atual dos encoders (ticks/s)
        double currentVelocity =
                (ShooterR.getVelocity() + ShooterL.getVelocity()) / 2.0;

        // PID
        double pidOut = pid.calculate(currentVelocity, targetVelocity);

        // Feedforward
        double ff = feedforward.calculate(targetVelocity);



        double output = clamp(pidOut + ff, 0, 1);

        if (gamepad1.right_trigger > 0.1) {
            ShooterR.setPower(output);
            ShooterL.setPower(output);
        } else {
            ShooterR.setPower(0);
            ShooterL.setPower(0);
        }

        // MUDAR RPM
        if (gamepad1.a) shooterSpeed = 14;
        if (gamepad1.b) shooterSpeed = 10;


        if (gamepad1.y) {
            ShooterR.setPower(-0.6);
            ShooterL.setPower(-0.6);
            Transfer.setPower(-0.5);
            Intake.setPower(0.3);
        }

        // COMPENSAR BATERIA
        double voltage = vs.getVoltage();
        if (voltage > 12.5) shooterSpeed -= 10;

        telemetry.addData("Velocity", currentVelocity);
        telemetry.addData("Target", targetVelocity);
    }

    // public void ll() {
    //
    // YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
    // limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
    //
    // LLResult llResult = limelight.getLatestResult();
    // if (llResult != null && llResult.isValid()) {
    //
    // Pose3D botpose = llResult.getBotpose();
    // /
    // telemetry.addData("target X", llResult.getTx());
    //  telemetry.addData("target Y", llResult.getTy());
    // telemetry.addData("target Area", llResult.getTa());
    // telemetry.addData(" botpose ", botpose.toString());
    // }
}

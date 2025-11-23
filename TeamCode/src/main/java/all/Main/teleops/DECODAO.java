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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
@TeleOp
public class DECODAO extends LinearOpMode {

    private DcMotor RMF, RMB, LMF, LMB;
    private DcMotor Intake, Transfer;
    private DcMotorEx ShooterR, ShooterL;
    private DistanceSensor distanceSensor;
    private Limelight3A limelight;
    private IMU imu;
    private VoltageSensor vs;

    // VALORES
    private double driveSpeed = 0.8;
    public static double shooterSpeed = 10;
    private static final double DEAD_ZONE = 0.2;

    // PIDF DE DASHBOARD
    public static double kP = 0.8;
    public static double kD = 0.0;
    public static double kI = 5;
    public static double kV = 10.0;
    public static double kA = 10.0;
    public static double kS = 10.0;

    SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(kS, kV, kA);

    PIDController pid =
            new PIDController(kP, kI, kD);

    // VALORES TRANSFER SYSTEM
    ElapsedTime rbTimer = new ElapsedTime();
    boolean rbAtivo = false;
    boolean rbRodando = false;
    double tempoRodar = 0.5;
    double tempoParar = 0.3;
    boolean transferEnabled = true;

    @Override
    public void runOpMode() {
        initHardware();

        waitForStart();

        while (opModeIsActive()) {
            loc();
            intake();
            transfer();
            shooter();

            telemetry.addData("Shooter Speed", shooterSpeed);
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

        Transfer = hardwareMap.get(DcMotor.class, "transfer");

        // SENSORES
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        vs = hardwareMap.voltageSensor.iterator().next();

        // DIRECTIONS
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);
        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

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

        //INIT IMU
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

        // GET HEADING
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // RESET PLAYER DIRECTION
        if (gamepad1.dpad_right) imu.resetYaw();

        // JOYSTICKS
        double strafe = applyDeadZone(gamepad1.left_stick_x);
        double drive = -applyDeadZone(gamepad1.left_stick_y);
        double turn = -applyDeadZone(gamepad1.right_stick_x);

        // MECANO ANGLES
        double fieldX = strafe * Math.cos(-heading) - drive * Math.sin(-heading);
        double fieldY = strafe * Math.sin(-heading) + drive * Math.cos(-heading);

        // STRAFE ERROR
        fieldX *= 1.1;

        // DENOMINATOR
        double denominator = Math.max(Math.abs(fieldY) + Math.abs(fieldX) + Math.abs(turn), 1);

        // POWERS
        double LMFpower = (fieldY + fieldX + turn) / denominator;
        double LMBpower = (fieldY - fieldX + turn) / denominator;
        double RMFpower = (fieldY - fieldX - turn) / denominator;
        double RMBpower = (fieldY + fieldX - turn) / denominator;

        LMF.setPower(LMFpower * driveSpeed);
        LMB.setPower(LMBpower * driveSpeed);
        RMF.setPower(RMFpower * driveSpeed);
        RMB.setPower(RMBpower * driveSpeed);

        telemetry.addData("Yaw", heading);
    }

    public void intake() {

        // INTAKE GAMEPAD
        if (gamepad1.left_trigger > 0.1 || gamepad1.right_bumper) {
            Intake.setPower(0.8);
        } else if (gamepad1.left_bumper) {
            Intake.setPower(-0.8);
        } else {
            Intake.setPower(0);
        }

    }

    public void transfer() {

        // VARIAVEL DISTANCE OF SENSOR
        double distance = distanceSensor.getDistance(DistanceUnit.CM);

        // VARIALVEL BALL DETECTED
        boolean ballDetected = (distance < 9 );

        // INTAKE

        // Se detectou bola -> trava apenas o trigger
        if (ballDetected) {
            transferEnabled = false;
        }

        if (gamepad1.left_trigger > 0.1 && transferEnabled && !ballDetected) {
            Transfer.setPower(0.5);
        } else {
            Transfer.setPower(0);
        }

        //TRANSFER

        // RB inicia ciclo temporizado e libera o trigger
        if (gamepad1.right_bumper && !rbAtivo) {
            rbAtivo = true;
            rbRodando = true;

            // RESET RB TIMER
            rbTimer.reset();

            // RB também reabilita o LT para poder usar depois
            transferEnabled = true;
        }

        // Se o ciclo do RB estiver ativo, executa o ciclo (RB tem prioridade)
        if (rbAtivo) {

            if (rbRodando) {
                Transfer.setPower(0.5);

                if (rbTimer.seconds() >= tempoRodar) {
                    rbRodando = false;

                }

            } else { // período parado
                Transfer.setPower(0);

                if (rbTimer.seconds() >= tempoParar) {
                    rbRodando = true;

                }

            }

            // Se quiser que o ciclo pare quando soltar o RB:
            if (!gamepad1.right_bumper) {
                rbAtivo = false;
                rbRodando = false;
                Transfer.setPower(0);

            }

            telemetry.addData("rbCycle", "active"); // CYCLE ACTIVE
            return;   // RB tem prioridade sobre o LT

        }

        telemetry.addData("distance", distance);
        telemetry.addData("transferEnabled", transferEnabled);
        telemetry.addData("rbCycle", "inactive"); // CYCLE INACTIVE
    }

    public void shooter() {

        // VARIAVEL VOLTAGEM
        double voltage = vs.getVoltage();

        // CURRENT VELOCITY
        double currentVelocity = (ShooterR.getVelocity() + ShooterL.getVelocity()) / 2.0;

        // TARGET VELOCITY
        double targetVelocity = shooterSpeed;

        // ATUALIZA PID E FF DASHBOARD
        pid.setPID(kP, kI, kD);
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);

        // FF
        double ff = feedforward.calculate(5,5);

        // PID OUT
        double pidOut = pid.calculate(currentVelocity, targetVelocity);

        // OUTPUT
        double output = pidOut + ff;

        // SHOOTER GAMEPAD
        if (gamepad1.right_trigger > 0.1) {
            ShooterR.setVelocity(output * shooterSpeed);
            ShooterL.setVelocity(output * shooterSpeed);
        } else {
            ShooterR.setPower(0);
            ShooterL.setPower(0);
        }

        if (gamepad1.y) {
            ShooterR.setPower(0.6);
            ShooterL.setPower(-0.6);
            Transfer.setPower(-0.6);
            Intake.setPower(0.3);
        }

        // TROCAR VELOCITY
        if (voltage > 13) {
            if (gamepad1.a) shooterSpeed = 6;
            if (gamepad1.b) shooterSpeed = 9;
        }

        if (voltage > 12 && voltage < 13) {
            if (gamepad1.a) shooterSpeed = 8;
            if (gamepad1.b) shooterSpeed = 10;
        }

        if (voltage < 11.5) {
            if (gamepad1.a) shooterSpeed = 12;
            if (gamepad1.b) shooterSpeed = 14;
        }

        telemetry.addData("VR ", ShooterR.getVelocity());
        telemetry.addData("VL ", ShooterL.getVelocity());
        telemetry.addData("voltage", voltage);

    }

//     public void ll() {
//
//         // VARIAVEL ORIENTATION
//         YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//
//         // LL ORIENTATION
//         limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
//
//         // LL ON
//         LLResult llResult = limelight.getLatestResult();
//         if (llResult != null && llResult.isValid()) {
//
//             Pose3D botpose = llResult.getBotpose();
//             telemetry.addData("target X", llResult.getTx());
//             telemetry.addData("target Y", llResult.getTy());
//             telemetry.addData("target Area", llResult.getTa());
//             telemetry.addData(" botpose ", botpose.toString());
//         }
//
//     }

}
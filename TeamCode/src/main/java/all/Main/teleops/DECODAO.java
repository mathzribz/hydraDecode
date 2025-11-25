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

    private static final double DEAD_ZONE = 0.2;

    public static double kP = 0.02;
    public static double kI = 0.0;
    public static double kD = 0.00005;

    // Feedforward constants
    public static double kS = 0.1;
    public static double kV = 0.00125;
    public static double kA = 0.00008;

    public static double TICKS_PER_REV = 28;
    public static double targetRPM = 3000;
    public static double targetTPS ;
    public static double shSpeed = 0.5 ;



    private final PIDController pid = new PIDController(kP, kI, kD);
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

    // VALORES TRANSFER SYSTEM
    ElapsedTime rbTimer = new ElapsedTime();
    boolean rbAtivo = false;
    boolean rbRodando = false;
    double tempoRodar = 0.05;
    double tempoParar = 0.5;
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
        Transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        ShooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterL.setDirection(DcMotorSimple.Direction.REVERSE);

        ShooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
                Transfer.setPower(1);

                if (rbTimer.seconds() >= tempoRodar) {
                    rbRodando = false;

                    rbTimer.reset();

                }

            } else { // período parado
                Transfer.setPower(0);

                if (rbTimer.seconds() >= tempoParar) {
                    rbRodando = true;
                    rbTimer.reset();

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


        double vr = ShooterR.getVelocity();
        double vl = ShooterL.getVelocity();
        double vAvg = (vr + vl) / 2.0;

        targetTPS = (targetRPM / 60.0) * TICKS_PER_REV;


        if (gamepad1.a) {
            targetRPM = 2000;
        }
        if (gamepad1.b) {
            targetRPM = 1000;

        } if (gamepad1.x) {
            targetRPM = 500;
        }
        double pidPower = pid.calculate(vAvg, targetTPS);
        double ffPower = ff.calculate(targetTPS);

        // ajustar feedforward com compensação de voltagem
        double voltage = vs.getVoltage();
        double compensatedFF = ffPower * (12.0 / Math.max(10.0, voltage));

        double finalPower = pidPower + compensatedFF;

        finalPower = Math.max(-1.0, Math.min(1.0, finalPower));

        if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) {
            ShooterR.setPower(shSpeed);
            ShooterL.setPower(shSpeed);
        } else {
            ShooterR.setPower(0);
            ShooterL.setPower(0);
            pid.reset();
        }

        if (gamepad1.y) {
            ShooterR.setPower(0.6);
            ShooterL.setPower(-0.6);
            Transfer.setPower(-0.6);
            Intake.setPower(0.3);
        }




        telemetry.addData("RPM avg", vAvg);
        telemetry.addData("targetRPM", targetRPM);
        telemetry.addData("PID", pidPower);
        telemetry.addData("FF", ffPower);
        telemetry.addData("FF compensado", compensatedFF);
        telemetry.addData("Voltagem", voltage);
        telemetry.addData("FinalPower", finalPower);
        telemetry.addData("RPM R", vr);
        telemetry.addData("RPM L", vl);

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
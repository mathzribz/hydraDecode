
package all.Main.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
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

    public static double kP = 0.0006;
    public static double kI = 0.0;
    public static double kD = 0.00001;
    public static double kF = 0.0015;

    public static double TICKS_PER_REV = 28;
    public static double targetRPM = 1300;
    public static double targetAlvo = 1250;
    public static double targetTPS ;
    public static double shSpeed = 0.5 ;

    public static double finalPower;


    // =============== LIMELIGHT TRACKER VARIÁVEIS ===============
    public static double LL_Kp = 0.065;
    public static double LL_Ki = 0.0;
    public static double LL_Kd = 0.0001;

    private double llIntegral = 0;
    private double llLastError = 0;
    private ElapsedTime llTimer = new ElapsedTime();
    private Limelight3A limelightLL;


    private final PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    ;
    // private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

    // VALORES TRANSFER SYSTEM
    ElapsedTime rbTimer = new ElapsedTime();
    boolean rbAtivo = false;
    boolean rbRodando = false;
    double tempoRodar = 0.2; ///0.05
    double tempoParar = 0.5;
    boolean transferEnabled = true;

    boolean shooterSolo = false;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
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



        limelightLL = hardwareMap.get(Limelight3A.class, "limelight");
        limelightLL.pipelineSwitch(0);
        limelightLL.start();
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
        double turnInput = -applyDeadZone(gamepad1.right_stick_x);
        double turn = turnInput;

        // ================== LIMELIGHT AIM ASSIST ==================
        if (gamepad1.right_trigger > 0.15) {   // segurou RT → tracking ativado
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelightLL.updateRobotOrientation(
                    orientation.getYaw(AngleUnit.DEGREES)
            );

            LLResult res = limelightLL.getLatestResult();

            if (res != null && res.isValid()) {
                double tx = res.getTx();
                 turn = LL_PID(tx, 0);   // substitui turn pelo PID de correção
                telemetry.addData("LL Tracking", "ATIVO");
                telemetry.addData("tx", tx);
            } else {
                telemetry.addData("LL Tracking", "SEM TAG");
            }
        } else {
            llIntegral = 0;  // reset quando soltar
            llLastError = 0;
        }

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
        if (gamepad1.left_trigger > 0.1) {
            Intake.setPower(1);
        } else if (gamepad1.dpad_left) {
            Intake.setPower(-0.8);
        } else {
            Intake.setPower(0);
        }




        if(gamepad1.left_bumper){
            Intake.setPower(0.9);
            Transfer.setPower(0.9);

        }
    }

    public void transfer() {

        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        boolean ballDetected = (distance < 12);
        boolean ltPressed = gamepad1.left_trigger > 0.1;
        boolean rbPressed = gamepad1.right_bumper;

        // ---------------------------------------------------------
        // 1) TRAVAMENTO DO LT
        //    O Transfer trava SE:
        //       - LT está sendo pressionado
        //       - E uma bola já foi detectada
        //    → RB só destrava ENQUANTO for segurado
        // ---------------------------------------------------------
        boolean ltTravado = ltPressed && ballDetected;
        boolean transferPodeRodarLT = ltPressed && !ltTravado;

        // ---------------------------------------------------------
        // 2) CICLO DO RB (PRIORITÁRIO)
        // ---------------------------------------------------------
        if (rbPressed && !rbAtivo) {
            rbAtivo = true;
            rbRodando = true;
            rbTimer.reset();
        }

        if (rbAtivo) {

            // Fase rodando
            if (rbRodando) {
                Transfer.setPower(0.8);
                Intake.setPower(0.7);

                if (rbTimer.seconds() >= tempoRodar) {
                    rbRodando = false;
                    rbTimer.reset();
                }

            } else { // fase parado
                Transfer.setPower(0);
                Intake.setPower(0);

                if (rbTimer.seconds() >= tempoParar) {
                    rbRodando = true;
                    rbTimer.reset();
                }
            }

            // Soltou RB → encerra ciclo na hora
            if (!rbPressed) {
                rbAtivo = false;
                rbRodando = false;
                Transfer.setPower(0);
            }

            return; // RB domina totalmente
        }

        // ---------------------------------------------------------
        // 3) LT CONTROLANDO TRANSFER (somente se não estiver travado)
        // ---------------------------------------------------------
        if (transferPodeRodarLT) {
            Transfer.setPower(0.5);
        } else {
            Transfer.setPower(0);
        }

        telemetry.addData("distance", distance);
        telemetry.addData("ltTravado", ltTravado);
        telemetry.addData("transferPodeRodarLT", transferPodeRodarLT);
        telemetry.addData("rbAtivo", rbAtivo);
    }

    public void shooter() {
        PIDFController pidf = new PIDFController(kP, kI, kD, kF);


        double vr = ShooterR.getVelocity();
        double vl = ShooterL.getVelocity();
        double vAvg = vl;

        targetTPS = (targetRPM / 60.0) * TICKS_PER_REV;


        if (gamepad1.x || gamepad2.x) {
            targetRPM = 1100;
        }
        if (gamepad1.a || gamepad2.a) {
            targetRPM = 1250;

        } if (gamepad1.b || gamepad2.b) {
            targetRPM = 2200;
        }
        double pidPower = pidf.calculate(vAvg, targetTPS);


        // ajustar feedforward com compensação de voltagem
        double voltage = vs.getVoltage();
        // double compensatedFF = ffPower * (12.0 / Math.max(10.0, voltage));

     finalPower = pidPower;

        finalPower = Math.max(-1.0, Math.min(1.0, finalPower));

        if ( gamepad2.right_trigger > 0.1) {
            ShooterR.setPower(finalPower);
            ShooterL.setPower(finalPower);
        } else {
            ShooterR.setPower(0);
            ShooterL.setPower(0);
            pidf.reset();
        }

        if (gamepad1.y) {
            ShooterR.setPower(0.6);
            ShooterL.setPower(-0.6);
            Transfer.setPower(-0.6);
            Intake.setPower(0.3);
        }


        if (gamepad1.dpad_down){
            shooterSolo = false;

        }
        if (gamepad1.dpad_up){
            shooterSolo = true;

        }

        if(shooterSolo){
            if ( gamepad2.right_trigger > 0.1 || gamepad1.right_trigger > 0.1 ) {
                ShooterR.setPower(finalPower);
                ShooterL.setPower(finalPower);
            } else {
                ShooterR.setPower(0);
                ShooterL.setPower(0);
                pidf.reset();
            }
        }




        telemetry.addData("RPM avg", vAvg);
        telemetry.addData("targetRPM", targetRPM);
        telemetry.addData("PID", pidPower);

        telemetry.addData("Voltagem", voltage);
        telemetry.addData("FinalPower", finalPower);
        telemetry.addData("RPM R", vr);
        telemetry.addData("RPM L", vl);

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
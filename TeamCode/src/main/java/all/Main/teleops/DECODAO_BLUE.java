
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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
@TeleOp
public class DECODAO_BLUE extends LinearOpMode {

    private DcMotor RMF, RMB, LMF, LMB;
    private DcMotor Intake, Transfer;
    private DcMotorEx ShooterR, ShooterL;
    private DistanceSensor distanceSensor;
    private Limelight3A limelight;
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;
    private VoltageSensor vs;

    // VALORES
    private double driveSpeed = 0.85;
    double headingOffset = 0;


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
    public static double LL_Kp = 0.065;
    public static double LL_Ki = 0.0;
    public static double LL_Kd = 0.0001;

    private double llIntegral = 0;
    private double llLastError = 0;
    private ElapsedTime llTimer = new ElapsedTime();
    private Limelight3A limelightLL;


    private final PIDFController pidf = new PIDFController(kP, kI, kD, kF);


    // VALORES TRANSFER SYSTEM
    private boolean rbAtivo = false;
    private boolean rbRodando = false;
    private boolean rbEsperando = false;

    private ElapsedTime rbTimer = new ElapsedTime();

    public double tempoParar = 0.3;
    boolean transferEnabled;

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
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
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
        pinpoint.update();


        // RESET PLAYER DIRECTION
        if (gamepad1.dpad_right){ pinpoint.recalibrateIMU();
            }


        double heading = pinpoint.getHeading(AngleUnit.DEGREES);

        // JOYSTICKS
        double strafe = applyDeadZone(gamepad1.left_stick_x);
        double drive = -applyDeadZone(gamepad1.left_stick_y);
        double turn = -applyDeadZone(gamepad1.right_stick_x);

        // ================== LIMELIGHT AIM ASSIST ==================
        if (gamepad1.right_trigger > 0.15) {   // segurou RT → tracking ativado
            limelightLL.updateRobotOrientation(
                    pinpoint.getHeading(AngleUnit.DEGREES)
            );


            LLResult res = limelightLL.getLatestResult();

            if (res != null && res.isValid()) {
                double tx = res.getTx();
                 turn = LL_PID(tx, -3);   // substitui turn pelo PID de correção
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



        telemetry.addData("Pinpoint Heading", heading);
        telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
    }

    public void intake() {

        // INTAKE GAMEPAD
        if (gamepad1.left_trigger > 0.1) {
            Intake.setPower(0.75);
        } else if (gamepad1.dpad_left) {
            Intake.setPower(-0.75);
        } else {
            Intake.setPower(0);
        }


        if(gamepad1.left_bumper){
            Intake.setPower(1);
            Transfer.setPower(1);

        }
    }

    public void transfer() {

        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        boolean ballDetected = (distance < 12);
        boolean ballDetectedRB = (distance < 11.5);
        boolean ltPressed = gamepad1.left_trigger > 0.1;
        boolean rbPressed = gamepad1.right_bumper;


        boolean ltTravado = ltPressed && ballDetected;
        boolean transferPodeRodarLT = ltPressed && !ltTravado;


        // ---- ESTADOS ----
        // rbAtivo  → RB está sendo segurado
        // rbRodando → true = motor rodando; false = aguardando delay
        // rbEsperando → true = esperando delay após detectar bola
        // rbTimer → controla o tempo

        // Início ao apertar RB
        if (rbPressed && !rbAtivo) {
            rbAtivo = true;
            rbRodando = true;
            rbEsperando = false;
            rbTimer.reset();
        }

        if (rbAtivo) {

            // ======================================================
            // 1. ESTADO: MOTOR RODANDO ATÉ DETECTAR A BOLA NO SENSOR
            // ======================================================
            if (rbRodando) {

                Transfer.setPower(0.4);
                Intake.setPower(0.8);

                // Se bola detectada → muda para estado de ESPERA
                if (ballDetectedRB) {
                    rbRodando = false;
                    rbEsperando = true;
                    rbTimer.reset();
                }
            }

            // ======================================
            // 2. ESTADO: ESPERA APÓS DETECTAR A BOLA
            // ======================================
            else if (rbEsperando) {

                Transfer.setPower(0);
                Intake.setPower(0);

                // Espera X segundos
                if (rbTimer.seconds() >= tempoParar) {
                    rbEsperando = false;
                    rbRodando = true;  // volta a rodar
                }
            }

            // ========================================
            // 3. SOLTOU RB → PARA TUDO AUTOMATICAMENTE
            // ========================================
            if (!rbPressed) {
                rbAtivo = false;
                rbRodando = false;
                rbEsperando = false;

                Transfer.setPower(0);
                Intake.setPower(0);
            }

            return; // RB domina totalmente
        }

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

        double vl = ShooterL.getVelocity();


        targetTPS = (targetRPM / 60.0) * TICKS_PER_REV;


        if (gamepad1.x || gamepad2.x) {
            targetRPM = 1200;
        }
        if (gamepad1.a || gamepad2.a) {
            targetRPM = 1350;

        } if (gamepad1.b || gamepad2.b) {
            targetRPM = 1500;
        }
        double pidPower = pidf.calculate(vl, targetTPS);


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
            ShooterR.setPower(-finalPower );
            ShooterL.setPower(-finalPower  );
            Transfer.setPower(-0.7);
            Intake.setPower(0.3);
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

package all.Main.teleops;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.Supplier;

import all.configPedro.Constants;

@Configurable
@TeleOp
public class Pedro_TELEOP extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(0, 0, Math.toRadians(90)); //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private DcMotor Intake, Transfer;
    private DcMotorEx ShooterR, ShooterL;
    private DistanceSensor distanceSensor;
    private Limelight3A limelight;
    private IMU imu;
    private VoltageSensor vs;

    // VALORES
    private double driveSpeed = 0.85;

    private static final double DEAD_ZONE = 0.25;

    public static double kP = 0.00055;
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
    public static double LL_Kd = 0.005;

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
    ElapsedTime timer = new ElapsedTime();

    public double tempoParar = 0.5;
    boolean transferEnabled;
    boolean bolaRegistrada = false;




    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(follower.getPose().withHeading(180));
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))

                .build();

        initHardware();

    }
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        double turn  =  -gamepad1.right_stick_x;

        if (gamepad1.right_trigger > 0.15) {
            LLResult res = limelightLL.getLatestResult();
            if (res != null && res.isValid()) {
                double tx = res.getTx();
                turn = LL_PID(tx, -3);
            }
        } else {
            llIntegral = 0;
            llLastError = 0;
        }

        //Call this once per loop
        follower.update();
        telemetryM.update();
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    turn,
                    false // Robot Centric
            );


        }
        //Automated PathFollowing


        timer.reset();

        intake();
        transfer();
        shooter();




        if(gamepad1.dpad_right) {
            follower.setPose(follower.getPose().withHeading(0));

        }
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }


    private void initHardware() {



        Intake = hardwareMap.get(DcMotor.class, "intake");

        ShooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        ShooterL = hardwareMap.get(DcMotorEx.class, "shooterL");

        Transfer = hardwareMap.get(DcMotor.class, "transfer");

        // SENSORES
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        vs = hardwareMap.voltageSensor.iterator().next();



        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        ShooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterL.setDirection(DcMotorSimple.Direction.REVERSE);



        // IMU


        limelightLL = hardwareMap.get(Limelight3A.class, "limelight");
        limelightLL.pipelineSwitch(0);
        limelightLL.start();
    }


    public void intake() {

        // INTAKE GAMEPAD
        if (gamepad1.left_trigger > 0.1) {
            Intake.setPower(0.85);
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
        boolean ltPressed = gamepad1.left_trigger > 0.1;
        boolean rbPressed = gamepad1.right_bumper;

        // ================================
        // DETECÇÃO DE NOVA BOLA (EDGE)
        // ================================
        boolean novaBola = false;

        if (ballDetected && !bolaRegistrada) {
            novaBola = true;         // só dispara 1 vez por bola
            bolaRegistrada = true;   // marca como registrada
        }

        if (!ballDetected) {
            bolaRegistrada = false;  // libera para próxima bola
        }

        // ================================
        // LÓGICA DO RB
        // ================================

        if (rbPressed && !rbAtivo) {
            rbAtivo = true;
            rbRodando = true;
            rbEsperando = false;
            rbTimer.reset();
        }

        if (rbAtivo) {

            if (rbRodando) {

                Transfer.setPower(0.4);
                Intake.setPower(0.8);

                // AGORA A BOLA SÓ ATIVA UMA VEZ
                if (novaBola) {
                    rbRodando = false;
                    rbEsperando = true;
                    rbTimer.reset();
                }
            }

            else if (rbEsperando) {

                Transfer.setPower(0);
                Intake.setPower(0);

                if (rbTimer.seconds() >= tempoParar) {
                    rbEsperando = false;
                    rbRodando = true;
                }
            }

            if (!rbPressed) {
                rbAtivo = false;
                rbRodando = false;
                rbEsperando = false;

                Transfer.setPower(0);
                Intake.setPower(0);
            }

            return; // RB domina
        }

        // ================================
        // LÓGICA DO LT
        // ================================
        boolean ltTravado = ltPressed && ballDetected;
        boolean transferPodeRodarLT = ltPressed && !ltTravado;

        if (transferPodeRodarLT) {
            Transfer.setPower(0.5);
        } else {
            Transfer.setPower(0);
        }
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
            targetRPM = 1600;
        }
        double pidPower = pidf.calculate(vl, targetTPS);


        // ajustar feedforward com compensação de voltagem
        double voltage = vs.getVoltage();
        // double compensatedFF = ffPower * (12.0 / Math.max(10.0, voltage));

        finalPower = pidPower;

        finalPower = Math.max(-1.0, Math.min(1.0, finalPower));

        if ( gamepad1.right_trigger > 0.1) {
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
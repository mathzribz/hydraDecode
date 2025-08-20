package opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;


import config.pedro.constants.FConstants;
import config.pedro.constants.LConstants;
import config.subsystems.Claw;
import config.subsystems.Intake;
import config.subsystems.Lift;
import config.subsystems.Pulse;
import config.subsystems.Slide;

@TeleOp
public class AuraEgo extends PedroOpMode {


    public AuraEgo() {
        super( Pulse.INSTANCE, Slide.INSTANCE);
    }

    private CRServo intakeL, intakeR;
    public String frontLeftName = "LMF";
    public String frontRightName = "RMF";
    public String backLeftName = "LMB";
    public String backRightName = "RMB";


    public MotorEx frontLeftMotor;
    public MotorEx frontRightMotor;
    public MotorEx backLeftMotor;
    public MotorEx backRightMotor;


    public MotorEx[] motors;

    public Command driverControlled;

    @Override
    public void onInit() {
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");

        frontLeftMotor = new MotorEx(frontLeftName);
        backLeftMotor = new MotorEx(backLeftName);
        backRightMotor = new MotorEx(backRightName);
        frontRightMotor = new MotorEx(frontRightName);

        // Change the motor directions to suit your robot.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        motors = new MotorEx[] {frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};

    }

    @Override
    public void onStartButtonPressed() {

        driverControlled = new MecanumDriverControlled(motors, gamepadManager.getGamepad1());
        driverControlled.invoke();
        modoIntake();


    }


   public void modoIntake(){
       if (gamepad1.dpad_up ) {
           intakeL.setPower(1);
           intakeR.setPower(-1);

       }

       else if (gamepad1.dpad_down ) {
           intakeL.setPower(-1);
           intakeR.setPower(-1);

       }
       else {
           intakeL.setPower(0);
           intakeR.setPower(0);
       }






        }
   }





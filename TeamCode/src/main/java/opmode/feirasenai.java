package opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import config.pedro.constants.FConstants;
import config.pedro.constants.LConstants;
import config.subsystems.Claw;
import config.subsystems.Lift;

@TeleOp
public class feirasenai extends PedroOpMode {
    public feirasenai(){
        super(Claw.INSTANCE, Lift.INSTANCE);
    }

    public String frontLeftName = "LMF";
    public String frontRightName = "LMB";
    public String backLeftName = "LMB";
    public String backRightName = "RMB";
    public MotorEx RMF, RMB, LMF, LMB;
    public MotorEx[] motors;


    public Command driverControlled;

    @Override
    public void onInit() {
        LMF = new MotorEx(frontLeftName);
        LMB = new MotorEx(backLeftName);
        RMB = new MotorEx(backRightName);
        RMF = new MotorEx(frontRightName);

        // Change the motor directions to suit your robot.
        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);

        motors = new MotorEx[] {LMF, RMF, LMB, RMB};
    }

    @Override
    public void onStartButtonPressed() {
        driverControlled = new MecanumDriverControlled(motors, gamepadManager.getGamepad1());
        driverControlled.invoke();
    }
}


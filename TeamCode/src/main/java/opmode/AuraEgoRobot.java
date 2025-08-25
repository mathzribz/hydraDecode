
package opmode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.CommandManager;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadManager;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.pedro.DriverControlled;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;


import config.subsystems.Claw;
import config.subsystems.Outtake;
import config.subsystems.Pulse;
import config.subsystems.Lift;
import config.subsystems.Slide;

@TeleOp
public class AuraEgoRobot  extends PedroOpMode {
    public AuraEgoRobot() {
        super(Claw.INSTANCE, Lift.INSTANCE, Slide.INSTANCE, Pulse.INSTANCE, Outtake.INSTANCE);
    }

    CRServo intakeL, intakeR;
    DcMotorEx Kit;
    public IMU imu;
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
        frontLeftMotor = new MotorEx(frontLeftName);
        backLeftMotor = new MotorEx(backLeftName);
        backRightMotor = new MotorEx(backRightName);
        frontRightMotor = new MotorEx(frontRightName);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        motors = new MotorEx[]{frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};
        imu = hardwareMap.get(IMU. class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");

        Kit = hardwareMap.get(DcMotorEx.class, "Kit");

        Slide.INSTANCE.close();



    }


    @Override
    public void onStartButtonPressed() {
       // CommandManager.INSTANCE.scheduleCommand(new DriverControlled(gamepadManager.getGamepad1(), false));
        driverControlled = new MecanumDriverControlled(motors, gamepadManager.getGamepad1());
        driverControlled.invoke();

        gamepadManager.getGamepad2().getLeftBumper().setPressedCommand(
                () -> new SequentialGroup(
                        Slide.INSTANCE.open(),
                        Pulse.INSTANCE.toLow()
                )
        );

        gamepadManager.getGamepad2().getRightBumper().setPressedCommand(
                () -> new SequentialGroup(
                        Slide.INSTANCE.close(),
                        Pulse.INSTANCE.toUp()
                )
        );



        //transfer
        gamepadManager.getGamepad1().getA().setPressedCommand(Outtake.INSTANCE::transfer);

        //basket
        gamepadManager.getGamepad1().getB().setPressedCommand(Outtake.INSTANCE::basket);

        //specimen
        gamepadManager.getGamepad1().getX().setPressedCommand(Outtake.INSTANCE::specimen);

        //chamber
        gamepadManager.getGamepad1().getY().setPressedCommand(Outtake.INSTANCE::chamber);


        //Garra
        gamepadManager.getGamepad1().getDpadLeft().setPressedCommand(Claw.INSTANCE::close);

        gamepadManager.getGamepad1().getDpadRight().setPressedCommand(Claw.INSTANCE::open);
    }

    @Override
    public void onUpdate() {
        super.onUpdate();
        gamepadManager.updateGamepads();
        intake();
        outtake();

        if (gamepad1.options){
            imu.resetYaw();
        }
    }

    public void intake(){
        if (gamepad1.left_trigger > 0.1) {
            intakeL.setPower(0.8);
            intakeR.setPower(-0.8);

        }

        else if (gamepad1.right_trigger > 0.1) {
            intakeL.setPower(-0.8);
            intakeR.setPower(0.8);

        }
        else {
            intakeL.setPower(0);
            intakeR.setPower(0);
        }



    }

    public void outtake(){
        if (gamepad1.dpad_up ) {
            Kit.setPower(1);

        }

        else if (gamepad1.dpad_down) {
            Kit.setPower(-1);
        }
        else {
            Kit.setPower(0);
        }




    }


}
package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.CommandManager;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.pedro.DriverControlled;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import config.subsystems.Claw;
import config.subsystems.Lift;

@TeleOp
public class testeOP extends PedroOpMode {


    public testeOP(){
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
        CommandManager.INSTANCE.scheduleCommand(new DriverControlled(gamepadManager.getGamepad1(), false));
        if(gamepad1.dpad_down) {
            CommandManager.INSTANCE.scheduleCommand(new DriverControlled(gamepadManager.getGamepad1(), true));
        }
        else if (gamepad1.dpad_up) {
            CommandManager.INSTANCE.scheduleCommand(new DriverControlled(gamepadManager.getGamepad1(), false));
        }

        driverControlled.invoke();

    }
}


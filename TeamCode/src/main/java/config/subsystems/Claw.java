package config.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class Claw extends Subsystem {
    public static final Claw INSTANCE = new Claw();
    private Claw() { }
    public Servo servoG;
    public String name = "servoG";
    Double openPos = 0.0;
    Double closePos = 0.5;

    public Command open() {
        return new ServoToPosition(servoG, // SERVO TO MOVE
                openPos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command close() {
        return new ServoToPosition(servoG, // SERVO TO MOVE
                closePos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }





    @Override
    public void initialize() {
        servoG = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
    }


}

package config.subsystems;


import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class Slide extends Subsystem {
    public static final Slide INSTANCE = new Slide();
    private Slide() { }
    public Servo servoS;
    public String name = "slide";
    Double openPos = 0.0;
    Double closePos = 1.0;

    public Command open() {
        return new ServoToPosition(servoS, // SERVO TO MOVE
                openPos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command close() {
        return new ServoToPosition(servoS, // SERVO TO MOVE
                closePos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }





    @Override
    public void initialize() {
        servoS = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
    }



}

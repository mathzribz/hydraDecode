
package config.subsystems;


import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class IntakePulse extends Subsystem {
    public static final IntakePulse INSTANCE = new IntakePulse();
    private IntakePulse() { }
    public Servo IntakePulse;
    public String name = "IntakePulse";
    Double openPos = 0.0;
    Double closePos = 0.5;

    public Command open() {
        return new ServoToPosition(IntakePulse, // SERVO TO MOVE
                openPos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command close() {
        return new ServoToPosition(IntakePulse, // SERVO TO MOVE
                closePos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }





    @Override
    public void initialize() {
        IntakePulse = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
    }


}
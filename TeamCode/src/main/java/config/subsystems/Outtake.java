
package config.subsystems;


import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class Outtake extends Subsystem {
    public static final Outtake INSTANCE = new Outtake();
    private Outtake() { }
    public Servo outtake;
    public String name = "outtake";
    Double transferPos = 0.0;
    Double chamberPos = 0.3;
    Double basketPos = 0.6;
    Double specPos = 0.8;

    public Command transfer() {
        return new ServoToPosition(outtake, // SERVO TO MOVE
                transferPos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command basket() {
        return new ServoToPosition(outtake, // SERVO TO MOVE
                basketPos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command specimen() {
        return new ServoToPosition(outtake, // SERVO TO MOVE
                specPos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command chamber() {
        return new ServoToPosition(outtake, // SERVO TO MOVE
                chamberPos, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }





    @Override
    public void initialize() {
        outtake = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
    }


}

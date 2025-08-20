
package config.subsystems;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.SetPower;

public class Intake extends Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() { }
    public CRServo intakeL, intakeR;
    public String name = "intakeL";
    public String name2 = "intakeR";
    Double openSpeed = 1.0;
    Double closeSpeed = -1.0;

    public Command open() {
        intakeL.setPower(openSpeed);
        intakeL.setPower(openSpeed); // IMPLEMENTED SUBSYSTEM
        return null;
    }

    public Command close() {
        intakeL.setPower(closeSpeed);
        intakeL.setPower(closeSpeed); // IMPLEMENTED SUBSYSTEM
        return null;
    }





    @Override
    public void initialize() {
        intakeL = OpModeData.INSTANCE.getHardwareMap().get(CRServo.class, name);
        intakeR = OpModeData.INSTANCE.getHardwareMap().get(CRServo.class, name2);
    }


}
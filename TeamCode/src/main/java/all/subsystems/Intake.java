
package all.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Intake extends SubsystemBase {

    public Intake(final HardwareMap hwMap, String name) {
        motor = new MotorEx(hwMap, "intake");
        servo = hwMap.get(Servo.class, "gate");
    }

    private MotorEx motor;
    private final Servo servo;

    public double intakeSpeed = 0.8;
    public double transferSpeed = 1;

    public void IntakeOn() {motor.set(intakeSpeed);}

    public void Transfer() {motor.set(transferSpeed);}

    public void IntakeOff() {motor.set(0);}
    public void IntakeOut() {motor.set(-intakeSpeed);}

    public void GateOpen() {
        servo.setPosition(0.2   );
    }

    public void GateClose() {
        servo.setPosition(0.5);
    }

    @Override
    public void periodic() {

    }

}

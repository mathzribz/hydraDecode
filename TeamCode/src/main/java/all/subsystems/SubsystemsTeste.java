
package all.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SubsystemsTeste extends SubsystemBase {

    public SubsystemsTeste(final HardwareMap hwMap, String name) {
        motor = new MotorEx(hwMap, "motor");
        servo = hwMap.get(Servo.class, "servo");
    }

    private MotorEx motor;
    private final Servo servo;

    public double motorSpeed = 0.9;

    public void IntakeOn() {
        motor.set(motorSpeed);
    }

    public void IntakeOff() {
        motor.set(0);
    }

    public void servoLowest() {
        servo.setPosition(0);
    }

    public void servoMidest() {
        servo.setPosition(0.5);
    }

    public void servoHighest() {
        servo.setPosition(1);
    }

    public void lowMotorSpeed() {
        motorSpeed = 0.55;
    }

    public void midMotorSpeed() {
        motorSpeed = 0.7;
    }

    public void highMotorSpeed() {
        motorSpeed = 0.9;
    }

    @Override
    public void periodic() {

    }

}

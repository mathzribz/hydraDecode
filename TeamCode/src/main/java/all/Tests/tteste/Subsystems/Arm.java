package all.Tests.tteste.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends SubsystemBase {
    // 1 motor, 2 servos
    private MotorEx baseMotor, motor2;
    private Servo servo, claw;

    public double turnSpeed = 0.2;
    public double up = 0;
    public double down = 0.7;
    public double open = 0.5;
    public double close = 0;
    public double motor2Speed = 0.2;

    public Arm (final HardwareMap hMap, String name) {
        baseMotor = hMap.get(MotorEx.class, "baseMotor");
        motor2 = hMap.get(MotorEx.class, "motor2");
        servo = hMap.get(Servo.class, "servo1");
        claw = hMap.get(Servo.class, "gripServo");
    }

    public void turnRight() { baseMotor.set(-turnSpeed);}
    public void turnLeft() { baseMotor.set(turnSpeed);}
    public void front() { motor2.set(motor2Speed);}
    public void back() { motor2.set(-motor2Speed);}
    public void openClaw() { claw.setPosition(open);}
    public void closeClaw() { claw.setPosition(close);}
    public void upArm() { servo.setPosition(up);}
    public void downArm() { servo.setPosition(down);}

    @Override
    public void periodic() {

    }
}

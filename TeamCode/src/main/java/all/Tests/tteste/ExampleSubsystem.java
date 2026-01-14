package all.Tests.tteste;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ExampleSubsystem extends SubsystemBase {
    private MotorEx intakeMotor;
    public double intakeSpeed = 1;

    public ExampleSubsystem (final HardwareMap hMap, String name) {
        intakeMotor = hMap.get(MotorEx.class, "intakeMotor");
    }

    public void input () {
        intakeMotor.set(intakeSpeed);
    }

    public void output () {
        intakeMotor.set(-intakeSpeed);
    }

    public void intakeOff () { intakeMotor.set(0); }

    @Override
    public void periodic() {

    }
}

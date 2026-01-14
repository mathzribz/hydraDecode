package all.Tests.tteste.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import all.Tests.tteste.Subsystems.Arm;

public class UpArm extends CommandBase {

    private final Arm arm;

    public UpArm(Arm subsystem) {
        arm = subsystem;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.upArm();


    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

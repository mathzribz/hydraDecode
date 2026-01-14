package all.Tests.tteste.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import all.Tests.tteste.Subsystems.Arm;

public class DownArm extends CommandBase {
    private final Arm arm;

    public DownArm(Arm subsystem) {
        arm = subsystem;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.downArm();


    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

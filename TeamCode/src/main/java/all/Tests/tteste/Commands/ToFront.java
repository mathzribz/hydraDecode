package all.Tests.tteste.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import all.Tests.tteste.Subsystems.Arm;

public class ToFront extends CommandBase {

    private final Arm arm;

    public ToFront(Arm subsystem) {
        arm = subsystem;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.front();


    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

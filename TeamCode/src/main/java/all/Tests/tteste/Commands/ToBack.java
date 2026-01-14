package all.Tests.tteste.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import all.Tests.tteste.Subsystems.Arm;

public class ToBack extends CommandBase {

    private final Arm arm;

    public ToBack(Arm subsystem) {
        arm = subsystem;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.back();


    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

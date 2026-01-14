package all.Tests.tteste.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import all.Tests.tteste.Subsystems.Arm;

public class TurnRight extends CommandBase {

    private final Arm arm;

    public TurnRight(Arm subsystem) {
        arm = subsystem;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.turnRight();


    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

package all.Tests.tteste.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import all.Tests.tteste.Subsystems.Arm;

public class CloseClaw extends CommandBase {

    private final Arm claw;

    public CloseClaw(Arm subsystem) {
        claw = subsystem;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.closeClaw();


    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

package all.subsystems.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import all.subsystems.SubsystemsTeste;

public class IntakeOn extends CommandBase {

    private final SubsystemsTeste subsystemsTeste;

    public IntakeOn(SubsystemsTeste subsystem) {
        subsystemsTeste = subsystem;
        addRequirements(subsystemsTeste);
    }

    @Override
    public void initialize() {
        subsystemsTeste.IntakeOn();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

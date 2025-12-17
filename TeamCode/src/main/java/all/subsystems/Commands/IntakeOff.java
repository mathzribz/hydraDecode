package all.subsystems.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import all.subsystems.SubsystemsTeste;

public class IntakeOff extends CommandBase {

    private final SubsystemsTeste subsystemsTeste;

    public IntakeOff(SubsystemsTeste subsystem) {
        subsystemsTeste = subsystem;
        addRequirements(subsystemsTeste);
    }

    @Override
    public void initialize() {
        subsystemsTeste.IntakeOff();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

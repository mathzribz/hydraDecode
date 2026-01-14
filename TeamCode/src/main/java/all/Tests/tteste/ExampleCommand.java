package all.Tests.tteste;

import com.arcrobotics.ftclib.command.CommandBase;

public class ExampleCommand extends CommandBase {

    //Ligar intake
    private ExampleSubsystem m_subsystem;

    public ExampleCommand(ExampleSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.input();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

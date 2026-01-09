
package all.Commands.Gate;

import com.arcrobotics.ftclib.command.CommandBase;

import all.Subsystems.IntakeSubsystem;

public class GateOpen extends CommandBase {

    private final IntakeSubsystem intakeTransfer;

    public GateOpen(IntakeSubsystem subsystem) {
        intakeTransfer = subsystem;
        addRequirements(intakeTransfer);
    }

    @Override
    public void initialize() {
        intakeTransfer.GateOpen();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}


package all.Commands.Gate;

import com.arcrobotics.ftclib.command.CommandBase;

import all.subsystems.IntakeSubsystem;

public class GateClose extends CommandBase {

    private final IntakeSubsystem intakeTransfer;

    public GateClose(IntakeSubsystem subsystem) {
        intakeTransfer = subsystem;
        addRequirements(intakeTransfer);
    }

    @Override
    public void initialize() {
        intakeTransfer.GateClose();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

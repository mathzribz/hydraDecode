
package all.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;

import all.Subsystems.IntakeSubsystem;

public class Transfer extends CommandBase {

    private final IntakeSubsystem intakeTransfer;

    public Transfer(IntakeSubsystem subsystem) {
        intakeTransfer = subsystem;
        addRequirements(intakeTransfer);
    }

    @Override
    public void initialize() {
        intakeTransfer.Transfer();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

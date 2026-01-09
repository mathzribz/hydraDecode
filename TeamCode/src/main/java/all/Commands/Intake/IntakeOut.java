
package all.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;

import all.Subsystems.IntakeSubsystem;

public class IntakeOut extends CommandBase {

    private final IntakeSubsystem intakeTransfer;

    public IntakeOut(IntakeSubsystem subsystem) {
        intakeTransfer = subsystem;
        addRequirements(intakeTransfer);
    }

    @Override
    public void initialize() {
        intakeTransfer.IntakeOut();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

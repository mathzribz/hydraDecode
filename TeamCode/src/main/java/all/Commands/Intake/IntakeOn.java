
package all.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;

import all.subsystems.IntakeSubsystem;

public class IntakeOn extends CommandBase {

    private final IntakeSubsystem intakeTransfer;

    public IntakeOn(IntakeSubsystem subsystem) {
        intakeTransfer = subsystem;
        addRequirements(intakeTransfer);
    }

    @Override
    public void initialize() {
        intakeTransfer.IntakeOn();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

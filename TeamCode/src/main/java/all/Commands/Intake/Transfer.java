
package all.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;

import all.subsystems.Intake;

public class Transfer extends CommandBase {

    private final Intake intakeTransfer;

    public Transfer(Intake subsystem) {
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


package all.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;

import all.subsystems.Intake;

public class TransferAuto extends CommandBase {

    private final Intake intakeTransfer;

    public TransferAuto(Intake subsystem) {
        intakeTransfer = subsystem;
        addRequirements(intakeTransfer);
    }

    @Override
    public void initialize() {
        intakeTransfer.TransferAuto();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

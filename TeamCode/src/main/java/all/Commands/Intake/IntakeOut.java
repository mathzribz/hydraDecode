
package all.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;

import all.subsystems.Intake;

public class IntakeOut extends CommandBase {

    private final Intake intakeTransfer;

    public IntakeOut(Intake subsystem) {
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

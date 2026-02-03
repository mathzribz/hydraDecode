
package all.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;

import all.subsystems.Intake;

public class IntakeOn extends CommandBase {

    private final Intake intakeTransfer;

    public IntakeOn(Intake subsystem) {
        intakeTransfer = subsystem;
        addRequirements(intakeTransfer);
    }

    @Override
    public void initialize() {
        intakeTransfer.intakeOn();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

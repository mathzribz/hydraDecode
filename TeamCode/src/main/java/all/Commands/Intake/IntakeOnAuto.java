
package all.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;

import all.subsystems.Intake;

public class IntakeOnAuto extends CommandBase {

    private final Intake intakeTransfer;

    public IntakeOnAuto(Intake subsystem) {
        intakeTransfer = subsystem;
        addRequirements(intakeTransfer);
    }

    @Override
    public void initialize() {
        intakeTransfer.intakeOnAuto();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

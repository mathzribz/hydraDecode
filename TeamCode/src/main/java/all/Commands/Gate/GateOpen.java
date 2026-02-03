
package all.Commands.Gate;

import com.arcrobotics.ftclib.command.CommandBase;

import all.subsystems.Intake;

public class GateOpen extends CommandBase {

    private final Intake intakeTransfer;

    public GateOpen(Intake subsystem) {
        intakeTransfer = subsystem;
        addRequirements(intakeTransfer);
    }

    @Override
    public void initialize() {
        intakeTransfer.gateOpen();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}


package all.Commands.Gate;

import com.arcrobotics.ftclib.command.CommandBase;

import all.subsystems.Intake;

public class GateClose extends CommandBase {

    private final Intake intakeTransfer;

    public GateClose(Intake subsystem) {
        intakeTransfer = subsystem;
        addRequirements(intakeTransfer);
    }

    @Override
    public void initialize() {
        intakeTransfer.gateClose();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

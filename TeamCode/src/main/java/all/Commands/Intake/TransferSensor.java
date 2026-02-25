
package all.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;

import all.subsystems.Intake;

public class TransferSensor extends CommandBase {

    private final Intake intakeTransfer;

    public TransferSensor(Intake subsystem) {
        intakeTransfer = subsystem;
        addRequirements(intakeTransfer);
    }

    @Override
    public void initialize() {
        intakeTransfer.transferSensor();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

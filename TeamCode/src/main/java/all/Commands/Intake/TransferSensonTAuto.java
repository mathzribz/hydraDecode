
package all.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;

import all.subsystems.Intake;

public class TransferSensonTAuto extends CommandBase {

    private final Intake intakeTransfer;

    public TransferSensonTAuto(Intake subsystem) {
        intakeTransfer = subsystem;
        addRequirements(intakeTransfer);
    }

    @Override
    public void initialize() {
        intakeTransfer.TransferSensorTAuto();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

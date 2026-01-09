package all.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;

import all.Subsystems.IntakeSubsystem;

public class IntakeOff extends CommandBase {

    private final IntakeSubsystem intakeTransfer;

    public IntakeOff(IntakeSubsystem subsystem) {
        intakeTransfer = subsystem;
        addRequirements(intakeTransfer);
    }

    @Override
    public void initialize() {
        intakeTransfer.IntakeOff();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

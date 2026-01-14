package all.Tests.tteste.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import all.Tests.tteste.Subsystems.Arm;

public class OpenClaw extends CommandBase {

    private final Arm claw;

    public OpenClaw(Arm subsystem) {
        claw = subsystem;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.openClaw();


    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

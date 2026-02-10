
package all.Commands.Hood;

import com.arcrobotics.ftclib.command.CommandBase;

import all.subsystems.Intake;
import all.subsystems.Shooter;

public class HoodLow extends CommandBase {

    private final Shooter shooter;

    public HoodLow(Shooter subsystem) {
        shooter = subsystem;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.HoodLow();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}


package all.Commands.Hood;

import com.arcrobotics.ftclib.command.CommandBase;

import all.subsystems.Intake;
import all.subsystems.Shooter;

public class HoodHigh extends CommandBase {

    private final Shooter shooter;

    public HoodHigh(Shooter subsystem) {
        shooter = subsystem;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.HoodHigh();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

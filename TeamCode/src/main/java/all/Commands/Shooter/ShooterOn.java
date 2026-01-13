
package all.Commands.Shooter;

import com.arcrobotics.ftclib.command.CommandBase;

import all.subsystems.Intake;
import all.subsystems.Shooter;

public class ShooterOn extends CommandBase {

    private final Shooter shoot;

    public ShooterOn(Shooter subsystem) {
        shoot = subsystem;
        addRequirements(shoot);
    }

    @Override
    public void initialize() {
        shoot.ShooterOn();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

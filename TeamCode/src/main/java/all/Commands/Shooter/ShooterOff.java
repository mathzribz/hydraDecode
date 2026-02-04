
package all.Commands.Shooter;

import com.arcrobotics.ftclib.command.CommandBase;

import all.subsystems.Intake;
import all.subsystems.Shooter;

public class ShooterOff extends CommandBase {

    private final Shooter shoot;

    public ShooterOff(Shooter subsystem) {
        shoot = subsystem;
        addRequirements(shoot);
    }

    @Override
    public void initialize() {
        shoot.shooterOff();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}


package all.Commands.Shooter;

import com.arcrobotics.ftclib.command.CommandBase;

import all.subsystems.Shooter;

public class ShooterOnAuto extends CommandBase {

    private final Shooter shoot;

    public ShooterOnAuto(Shooter subsystem) {
        shoot = subsystem;
        addRequirements(shoot);
    }

    @Override
    public void initialize() {
        shoot.shooterOnAuto();


    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

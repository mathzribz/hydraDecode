package all.Commands.Turret;

import com.arcrobotics.ftclib.command.CommandBase;
import all.Subsystems.Turret;

import java.util.function.DoubleSupplier;

public class ManualTurret extends CommandBase {

    private final Turret turret;
    private final DoubleSupplier input;

    public ManualTurret(Turret t, DoubleSupplier in) {
        turret = t;
        input = in;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        turret.setTarget(turret.getAngle() + input.getAsDouble() * 2);
    }
}

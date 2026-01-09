package all.Commands.Turret;

import com.arcrobotics.ftclib.command.InstantCommand;
import all.Subsystems.Turret;

public class ResetTurret extends InstantCommand {

    public ResetTurret(Turret turret) {
        super(() -> turret.setTarget(0), turret);
    }
}

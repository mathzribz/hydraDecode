package all.Commands.Loc;

import com.arcrobotics.ftclib.command.InstantCommand;
import all.subsystems.Drive;

public class SetDriveSpeed extends InstantCommand {

    public SetDriveSpeed(Drive drive, double speed) {
        super(() -> drive.setDriveSpeed(speed), drive);
    }
}

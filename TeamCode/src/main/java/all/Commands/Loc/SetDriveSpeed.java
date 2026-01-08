package all.Commands.Loc;

import com.arcrobotics.ftclib.command.InstantCommand;
import all.subsystems.DriveSubsystem;

public class SetDriveSpeed extends InstantCommand {

    public SetDriveSpeed(DriveSubsystem drive, double speed) {
        super(() -> drive.setDriveSpeed(speed), drive);
    }
}

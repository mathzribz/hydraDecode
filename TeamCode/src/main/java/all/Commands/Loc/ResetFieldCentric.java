package all.Commands.Loc;

import com.arcrobotics.ftclib.command.InstantCommand;

import all.subsystems.DriveSubsystem;

public class ResetFieldCentric extends InstantCommand {

    public ResetFieldCentric(DriveSubsystem drive) {
        super(drive::resetFieldOrientation, drive);
    }
}

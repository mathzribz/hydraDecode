package all.Commands.Loc;

import com.arcrobotics.ftclib.command.InstantCommand;

import all.subsystems.Drive;

public class ResetFieldCentric extends InstantCommand {

    public ResetFieldCentric(Drive drive) {
        super(drive::resetFieldOrientation, drive);
    }
}

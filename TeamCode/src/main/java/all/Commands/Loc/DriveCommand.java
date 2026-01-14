package all.Commands.Loc;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import all.subsystems.Drive;

public class DriveCommand extends CommandBase {

    private final Drive drive;
    private final GamepadEx gamepad;

    public DriveCommand(Drive drive, GamepadEx gamepad) {
        this.drive = drive;
        this.gamepad = gamepad;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double x  = -gamepad.getLeftX();
        double y  = -gamepad.getLeftY();
        double rx = -gamepad.getRightX();

        drive.drive(x, y, rx);
    }


}

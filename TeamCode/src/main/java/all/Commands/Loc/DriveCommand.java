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

    private double applyDeadZone(double value) {
        return Math.abs(value) > 0.05 ? value : 0;
    }

    @Override
    public void execute() {

        double x  = applyDeadZone(-gamepad.getLeftX());
        double y  = applyDeadZone(gamepad.getLeftY());
        double rx = applyDeadZone(-gamepad.getRightX());

        drive.drive(x, y, rx);
    }

    @Override
    public boolean isFinished() {
        return false; // comando default
    }
}

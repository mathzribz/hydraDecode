package all.Commands.Loc;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import all.Subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final GamepadEx gamepad;

    public DriveCommand(DriveSubsystem drive, GamepadEx gamepad) {
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

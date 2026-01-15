
package all.Tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import all.Commands.Loc.DriveCommand;
import all.Commands.Loc.ResetFieldCentric;
import all.Commands.Loc.SetDriveSpeed;
import all.subsystems.Drive;
import all.subsystems.LLturret;

@TeleOp
public class TeleOpCommandBased extends CommandOpMode {

    private Drive drive;
    private LLturret ll;
    private GamepadEx gamepad1Ex;

    @Override
    public void initialize() {

        drive = new Drive(hardwareMap);
        ll = new LLturret(hardwareMap);
        gamepad1Ex = new GamepadEx(gamepad1);

        ll.switchPipeline(0);
        ll.start();

        // DEFAULT COMMAND (ESSENCIAL)
        drive.setDefaultCommand(
                new DriveCommand(drive, gamepad1Ex)
        );

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SetDriveSpeed(drive, 0.65));

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SetDriveSpeed(drive, 1.0));

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new ResetFieldCentric(drive));
    }

    @Override
    public void run() {

        drive.updatePinpoint();
        super.run();

        telemetry.addData("Heading (deg)", "%.2f", drive.getHeadingDeg());
        telemetry.addData("Drive Speed", "%.2f", drive.getDriveSpeed());
        telemetry.addData("cood pedro", ll.getPedroRobotPose());

        telemetry.update();
    }
}

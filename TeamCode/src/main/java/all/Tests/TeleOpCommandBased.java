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

@TeleOp
public class TeleOpCommandBased extends CommandOpMode {

    private Drive drive;
    private GamepadEx gamepad1Ex;

    @Override
    public void initialize() {

        gamepad1Ex = new GamepadEx(gamepad1);
        drive = new Drive(hardwareMap);



        double x  = gamepad1Ex.getLeftX();
        double y  = gamepad1Ex.getLeftY();
        double rx = gamepad1Ex.getRightX();

        drive.drive(x, y, rx);

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SetDriveSpeed(drive, 0.65));

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SetDriveSpeed(drive, 1.0));


        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new ResetFieldCentric(drive));

    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {

        }
        }
    }


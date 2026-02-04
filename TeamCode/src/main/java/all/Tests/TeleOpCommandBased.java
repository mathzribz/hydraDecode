
package all.Tests;

import static all.Configs.Turret.FieldConstants.BLUE_GOAL;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import all.Commands.Loc.DriveCommand;
import all.Commands.Loc.ResetFieldCentric;
import all.Commands.Loc.SetDriveSpeed;
import all.subsystems.Drive;
import all.subsystems.Intake;
import all.subsystems.LLturret;
import all.subsystems.Turret;
@TeleOp
public class TeleOpCommandBased extends CommandOpMode {
    private Drive drive;
    private LLturret ll;
    private Turret turret;
    private Intake intake;
    private GamepadEx gamepad1Ex;

    @Override
    public void initialize() {

        drive = new Drive(hardwareMap);
        ll = new LLturret(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        gamepad1Ex = new GamepadEx(gamepad1);

        turret.resetEncoder();

        Pose startPos = new Pose(0, 0,90 );

        drive.setStartingPose(startPos);

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
        super.run();

        turret.followPose(BLUE_GOAL,drive.getPose());

        intakeWorking();
        gateWorking();

        telemetry.addData("Heading (deg)", "%.2f", drive.getHeadingDeg());
        telemetry.addData("Drive Speed", "%.2f", drive.getDriveSpeed());
        telemetry.addData("Up (cm)", Intake.upBlocked);
        telemetry.addData("Down (cm)", Intake.downBlocked);
        telemetry.addData("Full Timer", intake.getFullTime());
        telemetry.addData("cood pedro",drive.getPose());
        // telemetry.addData("cood LL", ll.getPedroRobotPose());

        telemetry.update();
    }

    public void intakeWorking(){

        if(gamepad1Ex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
            intake.intakeOn();
        }

        else if(gamepad1Ex.getButton(GamepadKeys.Button.RIGHT_BUMPER) ) {
            intake.intakeOut();
        }

        else if(gamepad1Ex.getButton(GamepadKeys.Button.LEFT_BUMPER) ) {
            intake.Transfer();
        }
        else { intake.intakeStop();}
    }

    public void gateWorking(){

        if(gamepad1Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
            intake.gateOpen();
        }

        else { intake.gateClose();}
    }
}

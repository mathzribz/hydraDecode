
package all.Tests;

import static all.Configs.Pedro.Tuning.draw;
import static all.Configs.Pedro.Tuning.drawOnlyCurrent;
import static all.Configs.Pedro.Tuning.follower;
import static all.Configs.Turret.FieldConstants.BLUE_GOAL;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import all.Commands.Loc.DriveCommand;
import all.Commands.Loc.ResetFieldCentric;
import all.Commands.Loc.SetDriveSpeed;
import all.Configs.Panels.Drawing;
import all.subsystems.Drive;
import all.subsystems.Intake;

import all.subsystems.LLMegatag;
import all.subsystems.Shooter;
import all.subsystems.Turret;
@TeleOp
public class TeleOpCommandBased extends CommandOpMode {
    private Drive drive;
    private Turret turret;
    private Intake intake;
    private Shooter shooter;
    private LLMegatag ll;
    private GamepadEx gamepad1Ex;

    @Override
    public void initialize() {
        Drawing.init();

        drive = new Drive(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        ll = new LLMegatag(hardwareMap);
        gamepad1Ex = new GamepadEx(gamepad1);
        intake.useSensors = true;



        ll.switchPipeline(0);
        ll.start();

        Pose startPos = new Pose(33, 111, Math.toRadians(180) );

        drive.setStartingPose(startPos);

        drive.setDefaultCommand(
                new DriveCommand(drive, gamepad1Ex)
        );


        gamepad1Ex.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new SetDriveSpeed(drive, 0.9));

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new ResetFieldCentric(drive));



    }

    @Override
    public void run() {

        waitForStart();
        drive.updatePinpoint();
        super.run();

        try {
            Drawing.drawDebug(drive.follower);
        } catch (Exception e) {
            telemetry.addLine("drawing failed");
        }


            turret.followPose(BLUE_GOAL, drive.getPose(), drive.getHeadingRad());

        if (gamepad1Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1 && ll.isPoseReliable()) {
            drive.relocalizeWithLimelight(ll.getPedroRobotPose());

           // turret.setRelocalizationOffset(Math.toRadians(0));
        }


        intakeWorking();
        shooterWorking();


        telemetry.addData("Heading (deg)", "%.2f", drive.getHeadingDeg());
        telemetry.addData("Drive Speed", "%.2f", drive.getDriveSpeed());
        telemetry.addData("Up (cm)", Intake.upBlocked);
        telemetry.addData("Down (cm)", Intake.downBlocked);
        telemetry.addData("Mid (cm)", Intake.midBlocked);
        telemetry.addData("cood pedro",drive.getPose());
        telemetry.addData("target RPM",shooter.getTargetRPM());
        telemetry.addData("current RPM",shooter.getCurrentRPM());
        telemetry.addData("cood LL",ll.getPedroRobotPose());


        telemetry.update();

    }

    public void intakeWorking(){

        if(gamepad1Ex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
            intake.intakeOn();
        }

        else if(gamepad1Ex.getButton(GamepadKeys.Button.LEFT_BUMPER) ) {
            intake.intakeOut();
        }

        else if(gamepad1Ex.getButton(GamepadKeys.Button.RIGHT_BUMPER) ) {
            intake.TransferTeleop();
        }
        else { intake.intakeStop();}
    }

    public void shooterWorking() {

        if (gamepad1Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
            intake.gateOpen();
            shooter.shooterOn();
        } else {
            intake.gateClose();
            shooter.shooterOff();
        }

        if (gamepad1Ex.getButton(GamepadKeys.Button.X)) {
            shooter.setTargetRPM(1900);
        }
        if (gamepad1Ex.getButton(GamepadKeys.Button.Y)) {
            shooter.setTargetRPM(2300);
        }
        if (gamepad1Ex.getButton(GamepadKeys.Button.B)) {
            shooter.setTargetRPM(2900);
        }

        if (gamepad1Ex.getButton(GamepadKeys.Button.DPAD_UP)) {
            shooter.HoodHigh();
        }
        if (gamepad1Ex.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            shooter.HoodLow();
        }



    }

}

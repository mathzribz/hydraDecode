
package all.Main.teleops.NACIONAL;

import static all.Configs.Turret.FieldConstants.RED_GOAL;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import all.Commands.Loc.DriveCommand;
import all.Commands.Loc.ResetFieldCentric;
import all.Commands.Loc.SetDriveSpeed;
import all.Configs.Auto.PoseStorage;
import all.Configs.Panels.Drawing;
import all.subsystems.Drive;
import all.subsystems.Intake;

import all.subsystems.LLMegatag;
import all.subsystems.Shooter;
import all.subsystems.Turret;
@TeleOp
public class DECODAO_RED extends CommandOpMode {
    private Drive drive;
    private Turret turret;
    private Intake intake;
    private Shooter shooter;
    private LLMegatag ll;
    private GamepadEx gamepad1Ex;
    private double shooterRPM = 2300 ;

    @Override
    public void initialize() {
        Drawing.init();

        drive = new Drive(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        ll = new LLMegatag(hardwareMap);
        gamepad1Ex = new GamepadEx(gamepad1);

        ll.switchPipeline(0);
        ll.start();


        drive.setStartingPose(PoseStorage.currentPose);

        drive.setDefaultCommand(
                new DriveCommand(drive, gamepad1Ex)
        );


        gamepad1Ex.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SetDriveSpeed(drive, 0.95));

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

        turret.followPose(RED_GOAL, drive.getPose(), drive.getHeadingRad());

        if (gamepad1Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1 && ll.isPoseReliable()) {
//            turret.applyVisionCorrection(ll.getTx());

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

        else if(gamepad1Ex.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON) ) {
            intake.intakeOut();
        }

        else if(gamepad1Ex.getButton(GamepadKeys.Button.RIGHT_BUMPER) ) {
            intake.transferTeleop();
        }
        else if(gamepad1Ex.getButton(GamepadKeys.Button.LEFT_BUMPER) ) {

        }
        else { intake.intakeStop();}
    }

    public void shooterWorking() {
        shooter.shooterOn();

        if (gamepad1Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
            intake.gateOpen();
            shooter.setTargetRPM(shooterRPM);
        } else {
            intake.gateClose();
            shooter.setTargetRPM(1000);

        }

        if (gamepad1Ex.getButton(GamepadKeys.Button.X)) {
            shooterRPM = 2300;
        }
        if (gamepad1Ex.getButton(GamepadKeys.Button.Y)) {
            shooterRPM = 3000;
        }

        if (gamepad1Ex.getButton(GamepadKeys.Button.DPAD_UP)) {
            shooter.HoodHigh();
        }
        if (gamepad1Ex.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            shooter.HoodLow();
        }



    }

}

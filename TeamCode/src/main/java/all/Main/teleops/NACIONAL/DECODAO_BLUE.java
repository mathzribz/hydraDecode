
package all.Main.teleops.NACIONAL;

import static all.Configs.Turret.FieldConstants.BLUE_GOAL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import all.Commands.Loc.DriveCommand;
import all.Commands.Loc.ResetFieldCentric;
import all.Commands.Loc.SetDriveSpeed;
import all.Configs.Auto.PoseStorage;
import all.Configs.Panels.Drawing;
import all.Configs.Teleop.TeleopLogic;
import all.subsystems.BlinkinLED;
import all.subsystems.Drive;
import all.subsystems.Intake;
import all.subsystems.LLMegatag;
import all.subsystems.Shooter;
import all.subsystems.Turret;
@Config
@TeleOp
public class DECODAO_BLUE extends CommandOpMode {
    private Drive drive;
    private Turret turret;
    private Intake intake;
    private Shooter shooter;
    private LLMegatag ll;
    private BlinkinLED blink;
    private GamepadEx gamepad1Ex;
    public static double offset = 5;
    public ElapsedTime elapsedtime = new ElapsedTime();

    @Override
    public void initialize() {
        Drawing.init();

        drive = new Drive(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        ll = new LLMegatag(hardwareMap);
        gamepad1Ex = new GamepadEx(gamepad1);
        blink = new BlinkinLED(hardwareMap);
        elapsedtime.reset();


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

        turret.followPose(BLUE_GOAL, drive.getPose(), drive.getHeadingRad());

        if (gamepad1Ex.getButton(GamepadKeys.Button.Y) && ll.isPoseReliable()) {
            turret.applyVisionCorrection(ll.getTx(), offset);

        }

        intakeWorking();
        shooterWorking();
        led();

        telemetry.addData("Drive Speed", "%.2f", drive.getDriveSpeed());
        telemetry.addData("Up (cm)", Intake.upBlocked);
        telemetry.addData("Down (cm)", Intake.downBlocked);
        telemetry.addData("Mid (cm)", Intake.midBlocked);
        telemetry.addData("cood pedro",drive.getPose());
        telemetry.addData("target RPM",shooter.getTargetRPM());
        telemetry.addData("current RPM",shooter.getCurrentRPM());
        telemetry.addData("cood LL",ll.isPoseReliable());
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();

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
            intake.transferSensor();
        }
        else { intake.intakeStop();}
    }

    public void shooterWorking() {
        shooter.shooterOn();


        if (gamepad1Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
            shooter.putRpm(drive.getDistanceInInches(BLUE_GOAL,drive.getPose()));
            intake.gateOpen();
        } else {
            intake.gateClose();
            shooter.setTargetRPM(500);

        }

        shooter.putHood(drive.getDistanceInInches(BLUE_GOAL,drive.getPose()));


    }


    public void led(){
        double tolerance = 50.0;
        double atSpeed = Math.abs(shooter.getTargetRPM() - shooter.getCurrentRPM());


        if (Intake.allblocked) {
            blink.red();
        }

        else if (atSpeed < tolerance) {
            blink.violet();

        }
        else {
            blink.black();
        }

    }

}

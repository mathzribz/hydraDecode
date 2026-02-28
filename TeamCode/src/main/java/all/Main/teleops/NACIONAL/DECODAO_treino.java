
package all.Main.teleops.NACIONAL;

import static all.Configs.Turret.FieldConstants.BLUE_GOAL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import all.Commands.Loc.DriveCommand;
import all.Commands.Loc.ResetFieldCentric;
import all.Commands.Loc.SetDriveSpeed;
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
public class DECODAO_treino extends CommandOpMode {
    private Drive drive;
    private Turret turret;
    private Intake intake;
    private Shooter shooter;
    private LLMegatag ll;
    private BlinkinLED blink;
    private GamepadEx gamepad1Ex, gamepad2Ex;
    public static double offset = 5;
    private List<LynxModule> allHubs;
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
        gamepad2Ex = new GamepadEx(gamepad2);
        blink = new BlinkinLED(hardwareMap);
        elapsedtime.reset();


        ll.switchPipeline(0);
        ll.start();

        Pose startPos = new Pose(33, 111, Math.toRadians(180) );

        drive.setStartingPose(startPos);

        drive.setDefaultCommand(
                new DriveCommand(drive, gamepad1Ex)
        );


        gamepad1Ex.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SetDriveSpeed(drive, 0.95));

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new ResetFieldCentric(drive));

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


    }
    @Override
    public void run() {



        super.run();

        waitForStart();
        drive.updatePinpoint();
        turret.followPose(BLUE_GOAL, drive.getPose(), drive.getHeadingRad());


        try {
            Drawing.drawDebug(drive.follower);
        } catch (Exception e) {
            telemetry.addLine("drawing failed");
        }


        if (gamepad1Ex.getButton(GamepadKeys.Button.Y) && ll.isPoseReliable()) {
            turret.applyVisionCorrection(ll.getTx(), offset);

        }

        intakeWorking();
        shooterWorking();
        led();


        telemetry.addData("Drive Speed", "%.2f", drive.getDriveSpeed());

        telemetry.addData("cood pedro",drive.getPose());
        telemetry.addData("target RPM",shooter.getTargetRPM());
        telemetry.addData("current RPM",shooter.getCurrentRPM());
        telemetry.addData("cood LL",ll.isPoseReliable());
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        telemetry.addData("current intake", intake.getCurrentAmps());
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



        if (gamepad1Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1 || gamepad2Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
            shooter.putRpm(drive.getDistanceInInches(BLUE_GOAL,drive.getPose()));
            shooter.shooterOn();

        } else {

            shooter.shooterOff();

        }


        if (gamepad1Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1 ) {

            intake.gateOpen();
        } else {
            intake.gateClose();
        }

        shooter.putHood(drive.getDistanceInInches(BLUE_GOAL,drive.getPose()));


    }


    public void led(){
        double tolerance = 50.0;
        double atSpeed = Math.abs(shooter.getTargetRPM() - shooter.getCurrentRPM());

        boolean rpmled = atSpeed < tolerance;
        boolean fullin = intake.getCurrentAmps() >= 6;



        if (fullin && !rpmled) {
            blink.red();

        }   else if (rpmled  ) {
            blink.violet();

        }
        else {
            blink.black();
        }

    }

}

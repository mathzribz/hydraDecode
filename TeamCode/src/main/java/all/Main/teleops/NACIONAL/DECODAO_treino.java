
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
    private GamepadEx gamepad1Ex;
    public static double shooterRPM = 2300;
    public static double pos = 0.2;
    public static double offset = 5;
    private final TeleopLogic teleopLogic = new TeleopLogic();

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
//        teleopLogic.init(hardwareMap);
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


        telemetry.addData("Heading (deg)", "%.2f", drive.getHeadingDeg());
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
            intake.gateOpen();
            shooter.setTargetRPM(shooterRPM);
        } else {
            intake.gateClose();
            shooter.setTargetRPM(0);

        }

        if (gamepad1Ex.getButton(GamepadKeys.Button.A)) {
            shooterRPM = 2300;
        }
        if (gamepad1Ex.getButton(GamepadKeys.Button.X)) {
            shooterRPM = 3000;
        }

        if (gamepad1Ex.getButton(GamepadKeys.Button.DPAD_UP)) {
            shooter.HoodPos(pos);
        }
        if (gamepad1Ex.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            shooter.HoodLow();
        }
    }


    public void led(){
        double targetRPMUp = shooter.getTargetRPM() + 100;
        double targetRPMDown = shooter.getTargetRPM() - 100;

//        if (targetRPMDown < shooter.getCurrentRPM() && targetRPMUp > shooter.getCurrentRPM() ){
//            targetAprox = true;
//        }else{targetAprox = false;}

        if (Intake.allblocked) {
            blink.red();
        }
//    else if (targetAprox){
//        blink.orange();
//    }
        else {
            blink.black();
        }

    }

}

package all.Configs.Teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static all.Configs.Turret.FieldConstants.BLUE_GOAL;
import static all.Configs.Turret.FieldConstants.RED_GOAL;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

import all.Configs.Auto.PoseStorage;
import all.Configs.Panels.Drawing;
import all.subsystems.BlinkinLED;
import all.subsystems.Drive;
import all.subsystems.Intake;
import all.subsystems.LLMegatag;
import all.subsystems.Shooter;
import all.subsystems.Turret;

public class TeleopLogic {
    private Drive drive;
    private Turret turret;
    private Intake intake;
    private Shooter shooter;
    private LLMegatag ll;
    private BlinkinLED blink;
    private GamepadEx gamepad1Ex;
    private List<LynxModule> allHubs;

    public static double shooterRPM = 2300;
    boolean targetAprox = false;


    public void init(HardwareMap hw) {
        Drawing.init();

        drive = new Drive(hw);
        turret = new Turret(hw);
        intake = new Intake(hw);
        shooter = new Shooter(hw);
        ll = new LLMegatag(hw);
        blink = new BlinkinLED(hw);




//        drive.setDefaultCommand(
//                new DriveCommand(drive, gamepad1Ex)
//        );
//
//
//        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//                .whenPressed(new ResetFieldCentric(drive));

//        allHubs = hardwareMap.getAll(LynxModule.class);
//
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
    }



    public void bulkRead(){
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public int startLL(int pipeline){
        ll.switchPipeline(pipeline);
        ll.start();
        return pipeline;
    }

    public void update() {
        drive.updatePinpoint();

        led();
        telemetryUpdate();

        try {
            Drawing.drawDebug(drive.follower);
        } catch (Exception e) {
            telemetry.addLine("drawing failed");
        }

    }

// STARTER POS
    public void startPoseTreino() {
        Pose startPos = new Pose(33, 111, Math.toRadians(180) );

        drive.setStartingPose(startPos);

        }

    public void startPosePosAuto() {
        drive.setStartingPose(PoseStorage.currentPose);
    }


// TURRET
    public void turretWorking(String alliance, GamepadEx g){

        if (alliance.equals("BLUE")) {
            turret.followPose(BLUE_GOAL, drive.getPose(), drive.getHeadingRad());
        }

        if (alliance.equals("RED")) {
            turret.followPose(RED_GOAL, drive.getPose(), drive.getHeadingRad());
        }

            if (g.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1 && ll.isPoseReliable()) {
//                turret.applyVisionCorrection(ll.getTx());

            } else {
                turret.setRelocalizationOffset(0.0);
            }

    }

// SHOOTER
public void shooterWorking(GamepadEx g) {

    shooter.shooterOn();

    if (g.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
        intake.gateOpen();
        shooter.setTargetRPM(shooterRPM);
    } else {
        intake.gateClose();
        shooter.setTargetRPM(1000);

    }

    if (g.getButton(GamepadKeys.Button.X)) {
        shooterRPM = 2300;
    }
    if (g.getButton(GamepadKeys.Button.Y)) {
        shooterRPM = 3000;
    }

    if (g.getButton(GamepadKeys.Button.DPAD_UP)) {
        shooter.HoodHigh();
    }
    if (g.getButton(GamepadKeys.Button.DPAD_DOWN)) {
        shooter.HoodLow();
    }
}

// INTAKE
public void intakeWorking(GamepadEx g) {

    if (g.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
        intake.intakeOn();
    } else if (g.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
        intake.intakeOut();
    } else if (g.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
        intake.transferSensor();
    } else if (g.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
        intake.transferTeleop();
    } else {
        intake.intakeStop();
    }

}



// TELEMETRY
public void telemetryUpdate(){
    telemetry.addData("Heading (deg)", "%.2f", drive.getHeadingDeg());
    telemetry.addData("Drive Speed", "%.2f", drive.getDriveSpeed());
    telemetry.addData("Up (cm)", Intake.upBlocked);
    telemetry.addData("Down (cm)", Intake.downBlocked);
    telemetry.addData("Mid (cm)", Intake.midBlocked);
    telemetry.addData("cood pedro",drive.getPose());
    telemetry.addData("target RPM",shooter.getTargetRPM());
    telemetry.addData("current RPM",shooter.getCurrentRPM());
    telemetry.addData("cood LL",ll.isPoseReliable());

}

public void led(){
     double targetRPMUp = shooter.getTargetRPM() + 100;
     double targetRPMDown = shooter.getTargetRPM() - 100;

     if (targetRPMDown < shooter.getCurrentRPM() && targetRPMUp > shooter.getCurrentRPM() ){
         targetAprox = true;
     }else{targetAprox = false;}

    if (Intake.allblocked && !targetAprox){
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

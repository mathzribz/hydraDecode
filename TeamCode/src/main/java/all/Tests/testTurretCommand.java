
package all.Tests;

import static all.Configs.Turret.FieldConstants.BLUE_GOAL;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import all.Commands.t;
import all.Configs.Pedro.Constants;
import all.subsystems.Drive;
import all.subsystems.Turret;
@Config
@TeleOp
public class testTurretCommand extends CommandOpMode {

    private Follower follower;
    private GamepadEx gamepads1;
    private Turret turret;
    private Drive drive;



    Pose startPos = new Pose(0, 0);
    @Override
    public void initialize() {


        gamepads1 = new GamepadEx(gamepad1);

        turret = new Turret(hardwareMap);
        drive = new Drive(hardwareMap);
        follower = Constants.createFollower(hardwareMap);




        follower.setStartingPose(startPos);


    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();


           turret.seguirPose(BLUE_GOAL,follower.getPose());

            telemetry.addData("cood", follower.getPose());




            telemetry.update();
            follower.update();
        }
        reset();
    }



}

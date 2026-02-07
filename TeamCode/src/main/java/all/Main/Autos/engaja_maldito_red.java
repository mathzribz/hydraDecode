
package all.Main.Autos;

import static all.Configs.Turret.FieldConstants.BLUE_GOAL;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import all.Commands.t;
import all.Configs.Pedro.Constants;
import all.subsystems.Drive;
import all.subsystems.Intake;
import all.subsystems.LLturret;
import all.subsystems.Shooter;
import all.subsystems.Turret;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;

@Config
@TeleOp
public class engaja_maldito_red extends CommandOpMode {

    private Follower follower;
    private Drive drive;
    private Turret turret;
    private Intake intake;
    private Shooter shooter;

    ElapsedTime timer = new ElapsedTime();


    public PathChain score1;
    public PathChain repo1;



    Pose startPos = new Pose(127, 120, 35);


    private final Pose scorePose = new Pose(101.5, 100, Math.toRadians(45));
    private final Pose repoPose1 = new Pose(86, 66 , Math.toRadians(0));

    @Override
    public void initialize() {



        drive = new Drive(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        follower = Constants.createFollower(hardwareMap);




        follower.setStartingPose(startPos);

    }

    private void buildPaths() {

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(startPos, scorePose))
                .setLinearHeadingInterpolation(startPos.getHeading(), scorePose.getHeading())
                .build();

//------------------------------------------------------------------------------------------------------------------
        repo1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, repoPose1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), repoPose1.getHeading())
                .build();
    }

    private void autonomousRoutine() {
        timer.reset();

        turret.resetEncoder();
        turret.setInitialAngle(Math.PI);
        turret.setTarget(0);

        shooter.HoodHigh();

        shooter.setTargetRPM(2000);
        shooter.shooterOn();
        intake.gateClose();

        if (timer.seconds() > 2){
            new FollowPath(score1, true);
        }

        if (timer.seconds() > 5){
            intake.gateOpen();
            intake.Transfer();
        }


        if (timer.seconds() > 8){
            shooter.shooterOff();
            intake.gateClose();
            intake.intakeStop();
        }


        if (timer.seconds() > 14){
            new FollowPath(repo1, true);
        }


    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();


            autonomousRoutine();

            telemetry.addData("FullTimer", timer.seconds());
            telemetry.update();
            follower.update();
        }
        reset();
    }

}

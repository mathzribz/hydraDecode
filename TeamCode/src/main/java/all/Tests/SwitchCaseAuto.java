package all.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import all.Commands.Intake.IntakeOff;
import all.Commands.Intake.IntakeOn;
import all.Configs.Pedro.Constants;
import all.subsystems.Intake;
import all.subsystems.Shooter;

@Autonomous
public class SwitchCaseAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private Shooter shooterSubsystem;

    public enum PathState {

        DRIVE_STARTPOSE_SCOREPOSE,
        SCORE,
        DRIVE_SCOREPOSE_REPOPOSE1

    }

    PathState pathState;

    private final Pose starterPose = new Pose(20, 123, 138);
    private final Pose scorePose = new Pose(46, 98, 138);
    private final Pose repoPose1 = new Pose(18, 87, 90);

    PathChain drive_start_score, drive_score_repo1;

    public void buildPaths() {

        //START TO SCORE
        drive_start_score = follower.pathBuilder()
                .addPath(new BezierLine(starterPose, scorePose))
                .setLinearHeadingInterpolation(starterPose.getHeading(), scorePose.getHeading())
                .build();

        //SCORE TO REPOPOSE1
        drive_score_repo1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, repoPose1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), repoPose1.getHeading())
                .build();

    }

    public void statePathUpdate () {

        switch (pathState) {

            case DRIVE_STARTPOSE_SCOREPOSE:
                follower.followPath(drive_start_score, true);
                setPathState(PathState.SCORE);
                break;

            case SCORE:
                if (!follower.isBusy()) {

                    setPathState(PathState.DRIVE_SCOREPOSE_REPOPOSE1);
                }
                break;

            case DRIVE_SCOREPOSE_REPOPOSE1:
                if (!follower.isBusy()) {
                    follower.followPath(drive_score_repo1, true);
                }
                break;

            default:
                telemetry.addLine("END OF AUTO");
                break;
        }

    }

    public void setPathState (PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOSE_SCOREPOSE;

        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(starterPose);
    }

    public void start () {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
    }

}

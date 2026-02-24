
package all.Main.Autos;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import all.Configs.Auto.AutoLogic;
import all.Configs.Auto.PoseStorage;
import all.Configs.Pedro.Constants;
import all.subsystems.Turret;


@Autonomous
public class SwitchCaseAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private AutoLogic autologic = new AutoLogic();

    private Turret turret;



    public enum PathState {

        DRIVE_STARTPOSE_SCOREPOSE,
        START_SCORE,
        DRIVE_SCOREPOSE_REPOPOSE1,
        INTAKE,
        END

    }

    PathState pathState;

    private final Pose starterPose = new Pose(17.537, 119.723, Math.toRadians(142));
    private final Pose scorePose = new Pose(38.170418006430864, 102.26366559485528, Math.toRadians(142));
    private final Pose repoPose1 = new Pose(46, 65, Math.toRadians(180));


    PathChain drive_start_score, drive_score_repo1, intake;

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

                autologic.preSpin();
                if (opModeTimer.getElapsedTimeSeconds() > 0.25) {
                    follower.followPath(drive_start_score, true);
                    setPathState(PathState.START_SCORE);
                }
                pathTimer.resetTimer();

                break;

            case START_SCORE:

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    autologic.burstFire();
                }

                if (!autologic.isBusy()) {
                    setPathState(PathState.DRIVE_SCOREPOSE_REPOPOSE1);
                    follower.followPath(drive_score_repo1, true);
                }

                break;
            case DRIVE_SCOREPOSE_REPOPOSE1:
                if(!follower.isBusy()){
                    setPathState(PathState.INTAKE);


                }

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
        autologic.init(hardwareMap);

        buildPaths();
        follower.setPose(starterPose);
    }

    public void start () {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        follower.update();
        autologic.update();
        PoseStorage.currentPose = follower.getPose();


        statePathUpdate();

        telemetry.addData(String.valueOf(autologic.timer), "timer auto");


    }

}
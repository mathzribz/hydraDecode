package all.Main.autos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import all.configPedro.Constants;
import all.subsystems.Flywheel;
import all.subsystems.Intake;
import all.subsystems.Shooter;
import all.subsystems.Transfer;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous
public class testeAuto extends NextFTCOpMode {

    public testeAuto() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Transfer.INSTANCE, Flywheel.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    public PathChain score1;
    public PathChain repo1;
    public PathChain intake1;
    public PathChain score2;
    public PathChain repo2;
    public PathChain intake2;
    public PathChain score3;
    public PathChain repo3;
    public PathChain intake3;
    public PathChain score4;

    @Override
    public void onInit() {
        buildPaths();
        follower().setStartingPose(startPose);
    }

    private final Pose startPose = new Pose(26, 130, Math.toRadians(140));
    private final Pose scorePose = new Pose(50, 108, Math.toRadians(140));
    private final Pose repoPose1 = new Pose(50, 84 , Math.toRadians(180));
    private final Pose intakePose1 = new Pose(15, 84, Math.toRadians(180));
    private final Pose repoPose2 = new Pose(15, 60, Math.toRadians(180));
    private final Pose intakePose2 = new Pose(15, 60, Math.toRadians(180));
    private final Pose repoPose3 = new Pose(15, 36, Math.toRadians(180));
    private final Pose intakePose3 = new Pose(15, 36, Math.toRadians(180));





    private void buildPaths() {

        score1 = follower().pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

//------------------------------------------------------------------------------------------------------------------
        repo1 = follower().pathBuilder()
                .addPath(new BezierLine(scorePose, repoPose1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), repoPose1.getHeading())
                .build();


        intake1 = follower().pathBuilder()
                .addPath(new BezierLine(repoPose1, intakePose1))
                .setLinearHeadingInterpolation(repoPose1.getHeading(), intakePose1.getHeading())
                .build();

        score2 = follower().pathBuilder()
                .addPath(new BezierLine(intakePose1, scorePose))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), scorePose.getHeading())
                .build();

//------------------------------------------------------------------------------------------------------------------

        repo2 = follower().pathBuilder()
                .addPath(new BezierLine(scorePose, repoPose2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), repoPose2.getHeading())
                .build();

        intake2 = follower().pathBuilder()
                .addPath(new BezierLine(repoPose2, intakePose2))
                .setLinearHeadingInterpolation(repoPose2.getHeading(), intakePose2.getHeading())
                .build();

        score3 = follower().pathBuilder()
                .addPath(new BezierLine(intakePose1, scorePose))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), scorePose.getHeading())
                .build();

//------------------------------------------------------------------------------------------------------------------

        repo3 = follower().pathBuilder()
                .addPath(new BezierLine(scorePose, repoPose3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), repoPose3.getHeading())
                .build();

        intake3 = follower().pathBuilder()
                .addPath(new BezierLine(repoPose3, intakePose3))
                .setLinearHeadingInterpolation(repoPose3.getHeading(), intakePose3.getHeading())
                .build();

        score4 = follower().pathBuilder()
                .addPath(new BezierLine(intakePose3, scorePose))
                .setLinearHeadingInterpolation(intakePose3.getHeading(), scorePose.getHeading())
                .build();


    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                new FollowPath(score1, true),
                new FollowPath(repo1, true),
                new FollowPath(intake1, true)



        );
    }
}
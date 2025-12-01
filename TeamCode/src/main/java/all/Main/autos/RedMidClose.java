package all.Main.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import all.configPedro.Constants;
import all.subsystems.Flywheel;
import all.subsystems.Intake;
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
public class RedMidClose extends NextFTCOpMode {
    public RedMidClose() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intake.INSTANCE, Transfer.INSTANCE, Flywheel.INSTANCE),
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
    public PathChain repogg;

    @Override
    public void onInit() {

        buildPaths();
        follower().setStartingPose(startPose);
    }

    private final Pose startPose = new Pose(87, 135, Math.toRadians(0));
    private final Pose scorePose = new Pose(95, 97, Math.toRadians(44));
    private final Pose repoPose1 = new Pose(84, 83.5, Math.toRadians(0));
    private final Pose intakePose1 = new Pose(129, 83.5, Math.toRadians(0));
    private final Pose repoPose2 = new Pose(85, 59.5, Math.toRadians(0));
    private final Pose intakePose2 = new Pose(129, 59.5, Math.toRadians(0));
    private final Pose repoG = new Pose(119,58 , Math.toRadians(0));

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

        repogg = follower().pathBuilder()
                .addPath(new BezierLine(intakePose1, repoG))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), repoG.getHeading())
                .build();

        score3 = follower().pathBuilder()
                .addPath(new BezierLine(repoG, scorePose))
                .setLinearHeadingInterpolation(repoG.getHeading(), scorePose.getHeading())
                .build();

    }
    @Override
    public void onStartButtonPressed() {

        autonomousRoutine().schedule();
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(

                // SCORE 1
                Flywheel.INSTANCE.on,
                Flywheel.INSTANCE.onin,

                new FollowPath(score1, true),
                new Delay(0.65),
                Transfer.INSTANCE.on,
                new Delay(0.4),
                Intake.INSTANCE.onin,
                new Delay(2),
                Transfer.INSTANCE.off,

                Flywheel.INSTANCE.off,
                Flywheel.INSTANCE.off2,

                // INTAKE 1
                new FollowPath(repo1, true,0.95),
                Transfer.INSTANCE.onin,
                new FollowPath(intake1, true,0.65),
                new Delay(0.05),
                Transfer.INSTANCE.off,
                new Delay(0.9),
                Intake.INSTANCE.onkeep,

                // SCORE 2
                Flywheel.INSTANCE.on,
                Flywheel.INSTANCE.onin,

                new FollowPath(score2, true),
                Intake.INSTANCE.off,
                new Delay(0.6),
                Transfer.INSTANCE.on,
                new Delay(0.4),
                Intake.INSTANCE.onin,
                new Delay(2),

                Flywheel.INSTANCE.off,
                Flywheel.INSTANCE.off2,
                Transfer.INSTANCE.off,

                // INTAKE 2
                new FollowPath(repo2, true,0.95),
                Transfer.INSTANCE.onin,
                new FollowPath(intake2, true,0.5),
                new Delay(0.01),
                Transfer.INSTANCE.off,
                new Delay(0.9),


                new FollowPath(repogg,true),

                // SCORE 3
                Flywheel.INSTANCE.on,
                Flywheel.INSTANCE.onin,

                new FollowPath(score3, true),
                Intake.INSTANCE.off,
                new Delay(0.6),
                Transfer.INSTANCE.on,
                new Delay(0.4),
                Intake.INSTANCE.onin,
                new Delay(2.5),

                Flywheel.INSTANCE.off,
                Flywheel.INSTANCE.off2
        );

    }

}
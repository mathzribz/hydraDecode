
package all.Main.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
public class BlueGOATClose extends NextFTCOpMode {

    public BlueGOATClose() {
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
    public PathChain repo3;
    public PathChain intake3;
    public PathChain finalPose;
    public PathChain gate;

    @Override
    public void onInit() {

        buildPaths();
        follower().setStartingPose(startPose);
    }

    private final Pose startPose = new Pose(17, 115, Math.toRadians(89));
    private final Pose scorePose = new Pose(41.5, 102, Math.toRadians(134));
    private final Pose scorePose2 = new Pose(41.5, 102, Math.toRadians(135));
    private final Pose repoPose1 = new Pose(59, 85 , Math.toRadians(180));
    private final Pose intakePose1 = new Pose(18.5, 85, Math.toRadians(180));
    private final Pose repoPose2 = new Pose(57, 60.5, Math.toRadians(180));
    private final Pose intakePose2 = new Pose(16, 60.5, Math.toRadians(180));
    private final Pose repoG = new Pose(23,61 , Math.toRadians(180));
    private final Pose repoPose3 = new Pose(57, 38, Math.toRadians(180));

    private final Pose intakePose3 = new Pose(17, 38, Math.toRadians(180));
    private final Pose gatepose  = new Pose(37, 68.5, Math.toRadians(0));
    private final Pose parkpose  = new Pose(19, 68.5, Math.toRadians(0));

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
                .addPath(new BezierLine(intakePose1, scorePose2))
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
                .addPath(new BezierLine(repoG, scorePose2))
                .setLinearHeadingInterpolation(repoG.getHeading(), scorePose.getHeading())
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

        gate = follower().pathBuilder()
                .addPath(new BezierLine(intakePose3, gatepose))
                .setLinearHeadingInterpolation(intakePose3.getHeading(), gatepose.getHeading())
                .build();

        finalPose = follower().pathBuilder()
                .addPath(new BezierLine(gatepose, parkpose))
                .setLinearHeadingInterpolation(gatepose.getHeading(), parkpose.getHeading())
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
                new Delay(0.75),
                Transfer.INSTANCE.on,
                new Delay(0.5),
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
                new Delay(0.75),
                Transfer.INSTANCE.on,
                new Delay(0.5),
                Intake.INSTANCE.onin,
                new Delay(2),
                Transfer.INSTANCE.off,

                Flywheel.INSTANCE.off,
                Flywheel.INSTANCE.off2,
                Transfer.INSTANCE.off,

                // INTAKE 2
                new FollowPath(repo2, true,0.95),
                Transfer.INSTANCE.onin,
                new FollowPath(intake2, true,0.65),
                new Delay(0.07),
                Transfer.INSTANCE.off,
                new Delay(0.9),
                Intake.INSTANCE.onkeep,




                // SCORE 3
                Flywheel.INSTANCE.on,
                Flywheel.INSTANCE.onin,

                new FollowPath(score3, true),
                new Delay(0.75),
                Transfer.INSTANCE.on,
                new Delay(0.6),
                Intake.INSTANCE.onin,
                new Delay(2),
                Transfer.INSTANCE.off,

                Flywheel.INSTANCE.off,
                Flywheel.INSTANCE.off2,

                // INTAKE 3 + GATE
                new FollowPath(repo3, true,0.95),
                Transfer.INSTANCE.onin,
                new FollowPath(intake3, true,0.48),
                new Delay(0.03),
                Transfer.INSTANCE.off,
                new Delay(0.9),
                Intake.INSTANCE.onkeep,

                new FollowPath(finalPose, true),
                new FollowPath(gate, true)

        );

    }

}
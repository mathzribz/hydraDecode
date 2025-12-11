
package all.Main.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
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
public class RedFarGOAT extends NextFTCOpMode {

    public RedFarGOAT() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intake.INSTANCE, Transfer.INSTANCE, Flywheel.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    private Path scorePreload, turn1;

    @Override
    public void onInit() {
        buildPaths();
        follower().setStartingPose(startPose);
    }

    private final Pose startPose = new Pose(79, 8, Math.toRadians(90));
    private final Pose scorePose1 = new Pose(78, 14.5, Math.toRadians(3));
    private final Pose turnPose1 = new Pose(90, 30.5, Math.toRadians(90));

    private final Pose repoPose3 = new Pose(85, 35.5, Math.toRadians(0));
    private final Pose intakePose3 = new Pose(125, 35, Math.toRadians(0));

    public PathChain repo3;
    public PathChain intake3;

    private void buildPaths() {

//------------------------------------------------------------------------------------------------------------------
        scorePreload = new Path(new BezierLine(startPose, scorePose1));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading());

        turn1 = new Path(new BezierLine(scorePose1, turnPose1));
        turn1.setLinearHeadingInterpolation(scorePose1.getHeading(), turnPose1.getHeading());

        repo3 = follower().pathBuilder()
                .addPath(new BezierLine(scorePose1, repoPose3))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), repoPose3.getHeading())
                .build();

        intake3 = follower().pathBuilder()
                .addPath(new BezierLine(repoPose3, intakePose3))
                .setLinearHeadingInterpolation(repoPose3.getHeading(), intakePose3.getHeading())
                .build();

    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(

                // SCORE 1
                Flywheel.INSTANCE.onfar,
                Flywheel.INSTANCE.oninfar,

                new FollowPath(scorePreload, true),
                new Delay(1.4),
                Transfer.INSTANCE.on,
                new Delay(0.1),
                Transfer.INSTANCE.off,
                new Delay(0.55),
                Transfer.INSTANCE.on,
                new Delay(0.33),
                Intake.INSTANCE.onin,
                new Delay(2),
                Transfer.INSTANCE.off,
                new Delay(2),
                Intake.INSTANCE.off,
                Flywheel.INSTANCE.off,
                Flywheel.INSTANCE.off2,
                new Delay(1),

                new FollowPath(repo3, false,0.95),
                Transfer.INSTANCE.onin,
                new FollowPath(intake3, true,0.48),
                new Delay(0.01),
                Transfer.INSTANCE.off,
                new Delay(0.7),
                Intake.INSTANCE.onkeep,

                new FollowPath(scorePreload, true),
                new Delay(1.4),
                Transfer.INSTANCE.on,
                new Delay(0.1),
                Transfer.INSTANCE.off,
                new Delay(0.55),
                Transfer.INSTANCE.on,
                new Delay(0.33),
                Intake.INSTANCE.onin,
                new Delay(2),
                Transfer.INSTANCE.off,
                new Delay(2),
                Intake.INSTANCE.off,
                Flywheel.INSTANCE.off,
                Flywheel.INSTANCE.off2,
                new Delay(5),


                new FollowPath(turn1, true)


        );

    }

}
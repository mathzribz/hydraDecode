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
public class BlueGoatFar extends NextFTCOpMode {

    public BlueGoatFar() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intake.INSTANCE, Transfer.INSTANCE, Flywheel.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    private Path scorePreload , turn1;
    private PathChain intake1,scorePreload2;

    @Override
    public void onInit() {
        buildPaths();
        follower().setStartingPose(startPose);
    }

    private final Pose startPose = new Pose(57, 8, Math.toRadians(90));
    private final Pose scorePose1 = new Pose(60.5, 18.5, Math.toRadians(166));
    private final Pose turnPose1 = new Pose(68, 35.5, Math.toRadians(185));
    private final Pose intakePose1 = new Pose(18, 35.5, Math.toRadians(185));
    private final Pose scorePose2 = new Pose(60.5, 18.5, Math.toRadians(117));
    private void buildPaths() {

//------------------------------------------------------------------------------------------------------------------
        scorePreload = new Path(new BezierLine(startPose, scorePose1));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading());

        turn1 = new Path(new BezierLine(scorePose1, turnPose1));
        turn1.setLinearHeadingInterpolation(scorePose1.getHeading(), turnPose1.getHeading());

//------------------------------------------------------------------------------------------------------------------
        intake1 = follower().pathBuilder()
                .addPath(new BezierLine(turnPose1, intakePose1))
                .setLinearHeadingInterpolation(turnPose1.getHeading(), intakePose1.getHeading())
                .build();

        scorePreload2 = follower().pathBuilder()
                .addPath(new BezierLine(intakePose1, scorePose2))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), scorePose2.getHeading())
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

                // INTAKE 1
                Flywheel.INSTANCE.off,
                Flywheel.INSTANCE.off2,

                new FollowPath(turn1, true),
                Transfer.INSTANCE.onin,
                new FollowPath(intake1, true,0.65),
                new Delay(0.09),
                Transfer.INSTANCE.off,
                new Delay(1.15),
                Intake.INSTANCE.off,

                // SCORE 2
                Flywheel.INSTANCE.onfar,
                Flywheel.INSTANCE.oninfar,

                new FollowPath(scorePreload2, true),
                new Delay(1.2),
                Transfer.INSTANCE.on,
                new Delay(0.1),
                Transfer.INSTANCE.off,
                new Delay(0.4),
                Transfer.INSTANCE.on,
                new Delay(0.24),
                Intake.INSTANCE.onin,
                new Delay(2),
                Transfer.INSTANCE.off,

                Flywheel.INSTANCE.off,
                Flywheel.INSTANCE.off2,
                Intake.INSTANCE.off
        );

    }

}
package all.Main.autos;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import all.configPedro.Constants;
import all.subsystems.Flywheel;
import all.subsystems.Intake;
import all.subsystems.Transfer;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

public class RedGoatFar extends NextFTCOpMode {

    public RedGoatFar() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intake.INSTANCE, Transfer.INSTANCE, Flywheel.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    private Path score, turn1;
    private PathChain intake1;

    @Override
    public void onInit() {
        buildPaths();
        follower().setStartingPose(startPose);
    }

    private final Pose startPose = new Pose(79, 8, Math.toRadians(-90));
    private final Pose scorePose1 = new Pose(83.5, 17.5, Math.toRadians(-123));
    private final Pose turnPose1 = new Pose(89.5, 35.5, Math.toRadians(0));
    private final Pose intakePose1 = new Pose(128, 35.5, Math.toRadians(0));

    private void buildPaths() {

        //------------------------------------------------------------------------------------------------------------------
        score = new Path(new BezierLine(startPose, scorePose1));
        score.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading());

        turn1 = new Path(new BezierLine(scorePose1, turnPose1));
        turn1.setLinearHeadingInterpolation(scorePose1.getHeading(), turnPose1.getHeading());

        //------------------------------------------------------------------------------------------------------------------
        intake1 = follower().pathBuilder()
                .addPath(new BezierLine(turnPose1, intakePose1))
                .setLinearHeadingInterpolation(turnPose1.getHeading(), intakePose1.getHeading())
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

                new FollowPath(score, true),
                new Delay(1.3),
                Transfer.INSTANCE.on,
                new Delay(0.8),
                Intake.INSTANCE.onin,
                new Delay(2.2),
                Transfer.INSTANCE.off,

                Flywheel.INSTANCE.off,
                Flywheel.INSTANCE.off2,

                // INTAKE 1
                new FollowPath(turn1, true),
                Transfer.INSTANCE.onin,
                new FollowPath(intake1, true,0.55),
                new Delay(0.05),
                Transfer.INSTANCE.off,
                new Delay(0.9),
                Intake.INSTANCE.off,

                // SCORE 2
                Flywheel.INSTANCE.onfar,
                Flywheel.INSTANCE.oninfar,

                new FollowPath(score, true),
                new Delay(1),
                Transfer.INSTANCE.on,
                new Delay(0.8),
                Intake.INSTANCE.onin,
                new Delay(2.2),
                Transfer.INSTANCE.off,

                Flywheel.INSTANCE.off,
                Flywheel.INSTANCE.off2
        );
    }

}
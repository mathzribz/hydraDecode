package all.Main.autos;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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

@Autonomous
public class RedFar extends NextFTCOpMode {

    public RedFar() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intake.INSTANCE, Transfer.INSTANCE, Flywheel.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    private Path score, turn1;

    @Override
    public void onInit() {
        buildPaths();
        follower().setStartingPose(startPose);
    }

    private final Pose startPose = new Pose(79, 8, Math.toRadians(90));
    private final Pose scorePose1 = new Pose(78, 14.5, Math.toRadians(3));
    private final Pose turnPose1 = new Pose(90, 30.5, Math.toRadians(90));

    private void buildPaths() {

        //------------------------------------------------------------------------------------------------------------------
        score = new Path(new BezierLine(startPose, scorePose1));
        score.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading());

        turn1 = new Path(new BezierLine(scorePose1, turnPose1));
        turn1.setLinearHeadingInterpolation(scorePose1.getHeading(), turnPose1.getHeading());
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
                new Delay(15),


                new FollowPath(turn1, true)

        );
    }

}
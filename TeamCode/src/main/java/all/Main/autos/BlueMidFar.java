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
public class BlueMidFar extends NextFTCOpMode {

    public BlueMidFar() {
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

    private final Pose startPose = new Pose(57, 8, Math.toRadians(90));
    private final Pose scorePose1 = new Pose(61.5, 17.5, Math.toRadians(178));
    private final Pose turnPose1 = new Pose(68, 35.5, Math.toRadians(185));

    private void buildPaths() {

//------------------------------------------------------------------------------------------------------------------
        scorePreload = new Path(new BezierLine(startPose, scorePose1));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading());

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

                new FollowPath(scorePreload, true),
                new Delay(1.3),
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
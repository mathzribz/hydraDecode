package all.Main.autos;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

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

public class RedMidFar extends NextFTCOpMode {

    public RedMidFar() {
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

    private final Pose startPose = new Pose(79, 8, Math.toRadians(-90));
    private final Pose scorePose1 = new Pose(83.5, 17.5, Math.toRadians(-123));

    private void buildPaths() {

        //------------------------------------------------------------------------------------------------------------------
        score = new Path(new BezierLine(startPose, scorePose1));
        score.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading());
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
                Flywheel.INSTANCE.off2
        );
    }

}
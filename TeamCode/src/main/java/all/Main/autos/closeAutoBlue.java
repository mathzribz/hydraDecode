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
public class closeAutoBlue extends NextFTCOpMode {

    public closeAutoBlue() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Transfer.INSTANCE, Flywheel.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    private Path scorePreload, turn1;
    private PathChain  intake1,Score2;

    @Override
    public void onInit() {
        buildPaths();
        follower().setStartingPose(startPose);
    }

    private final Pose startPose = new Pose(27, 130, Math.toRadians(142));
    private final Pose scorePose1 = new Pose(50, 108, Math.toRadians(140));
    private final Pose turnPose1 = new Pose(60, 101 , Math.toRadians(186));
    private final Pose intakePose1 = new Pose(19, 101, Math.toRadians(186));
    private final Pose scorePose2 = new Pose(50, 108, Math.toRadians(130));


    private void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, scorePose1));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading());



        turn1 = new Path(new BezierCurve(scorePose1, turnPose1));
        turn1.setLinearHeadingInterpolation(scorePose1.getHeading(), turnPose1.getHeading());



        intake1 = follower().pathBuilder()
                .addPath(new BezierLine(turnPose1, intakePose1))
                .setLinearHeadingInterpolation(turnPose1.getHeading(), intakePose1.getHeading())
                .build();

        Score2 = follower().pathBuilder()
                .addPath(new BezierLine(intakePose1, scorePose1))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), scorePose1.getHeading())
                .build();
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(

                Flywheel.INSTANCE.on,
               new FollowPath(scorePreload, true),
                new Delay(3),
                Transfer.INSTANCE.on,
                new Delay(0.1),
                Intake.INSTANCE.on,
                new Delay(4),
                Flywheel.INSTANCE.off,
                Transfer.INSTANCE.off,

                Intake.INSTANCE.onin,
                new FollowPath(turn1, true),
                Transfer.INSTANCE.onin,
                new FollowPath(intake1, true,0.38),
                new Delay(0.5),
                Transfer.INSTANCE.off,
                new Delay(0.6),
                Intake.INSTANCE.off,
                new Delay(1),

                Flywheel.INSTANCE.on,
                new FollowPath(Score2, true),
                new Delay(3),
                Transfer.INSTANCE.on,
                Intake.INSTANCE.on,
                new Delay(4),
                Flywheel.INSTANCE.off,
                Transfer.INSTANCE.off,
                Intake.INSTANCE.off




              //  Transfer.INSTANCE.on,
           //     Intake.INSTANCE.on,
              //
                //  new FollowPath(intake1, true)


//
//                new FollowPath(turn1, true),
//
//                new Delay(1),
//
//                new FollowPath(intake1, true)


        );
    }
}
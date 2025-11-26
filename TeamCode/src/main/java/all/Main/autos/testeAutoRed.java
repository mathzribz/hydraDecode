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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous
public class testeAutoRed extends NextFTCOpMode {
    private DistanceSensor dd;

    public testeAutoRed() {
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
    public PathChain repo3;
    public PathChain intake3;
    public PathChain score4;

    @Override
    public void onInit() {
        buildPaths();
        follower().setStartingPose(startPose);


    }

    private final Pose startPose = new Pose(57, 134, Math.toRadians(0));
    private final Pose scorePose = new Pose(87, 100, Math.toRadians(40));
    private final Pose repoPose1 = new Pose(75, 88, Math.toRadians(0));
    private final Pose intakePose1 = new Pose(123, 88, Math.toRadians(0));
    private final Pose repoPose2 = new Pose(57, 60, Math.toRadians(180));
    private final Pose intakePose2 = new Pose(17.5, 60, Math.toRadians(180));
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
        // VARIAVEL DISTANCE OF SENSOR


        // VARIALVEL BALL DETECTED


        return new SequentialGroup(
                Flywheel.INSTANCE.on,
                Flywheel.INSTANCE.onin,

                new FollowPath(score1, true),
                new Delay(0.5),
                Transfer.INSTANCE.on,
                new Delay(0.35),
                Intake.INSTANCE.onin,
                new Delay(2.8),
                Transfer.INSTANCE.off,

                Flywheel.INSTANCE.off,
                Flywheel.INSTANCE.off2,

                new FollowPath(repo1, true),
                Transfer.INSTANCE.onin,
                new FollowPath(intake1, true,0.6),
                new Delay(0.35),
                Transfer.INSTANCE.off,
                new Delay(0.8),
                Intake.INSTANCE.off,

                Flywheel.INSTANCE.on,
                Flywheel.INSTANCE.onin,

                new FollowPath(score2, true),
                new Delay(1),
                Transfer.INSTANCE.on,
                new Delay(0.35),
                Intake.INSTANCE.onin,
                new Delay(3),

                Flywheel.INSTANCE.off,
                Flywheel.INSTANCE.off2,
                Transfer.INSTANCE.off







        );
    }

}

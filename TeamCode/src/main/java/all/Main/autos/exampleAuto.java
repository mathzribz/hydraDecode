package all.Main.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import all.configPedro.Constants;
import all.subsystems.FlyWheelTester;
import all.subsystems.Intake;
import all.subsystems.Shooter;
import all.subsystems.Transfer;




import org.threeten.bp.Duration;

import all.subsystems.transfers;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

@Autonomous(name = "AutoExampleNextFTC", group = "NextFTC")
public class exampleAuto extends NextFTCOpMode {

    public exampleAuto() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Transfer.INSTANCE, FlyWheelTester.INSTANCE, transfers.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1;

    @Override
    public void onInit() {
        buildPaths();
        follower().breakFollowing();
        follower().setStartingPose(startPose);

    }

    private final Pose startPose = new Pose(16, 111, Math.toRadians(90));
    private final Pose scorePose = new Pose(53, 95, Math.toRadians(135));
    private final Pose pickupPose = new Pose(16, 86, Math.toRadians(190));

    private void buildPaths() {
        // Caminho direto até o ponto de lançamento

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Caminho de volta para pegar um artefato
        grabPickup1 = follower().pathBuilder()
                .addPath(new BezierLine(scorePose, pickupPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPose.getHeading())
                .build();

        // Caminho para retornar e lançar novamente
        scorePickup1 = follower().pathBuilder()
                .addPath(new BezierLine(pickupPose, scorePose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
                .build();
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                new ParallelGroup(

                 new FollowPath(scorePreload, true),
                FlyWheelTester.INSTANCE.intakeOn2
                        ),
                transfers.INSTANCE.intakeOn2,

               // new FollowPath(grabPickup1, true, 0.8),
               // Intake.INSTANCE.intakeOn,
                new Delay(1),
              //  Intake.INSTANCE.intakeOn,

               // new FollowPath(scorePickup1, true),
              //  Shooter.INSTANCE.shooterOn,
                new Delay(5)
              //  Shooter.INSTANCE.shooterOn
        );
    }
}
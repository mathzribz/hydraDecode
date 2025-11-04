package all.Main.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import all.configPedro.Constants;
import all.subsystems.Intake;
import all.subsystems.Shooter;
import all.subsystems.Transfer;

import static all.configPedro.Tuning.follower;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import org.threeten.bp.Duration;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "AutoExampleNextFTC", group = "NextFTC")
public class exampleAuto extends NextFTCOpMode {

    public exampleAuto() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Transfer.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1;

    @Override
    public void onInit() {
        buildPaths();
        follower.setStartingPose(startPose);
    }

    private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180));
    private final Pose scorePose = new Pose(60, 85, Math.toRadians(135));
    private final Pose pickupPose = new Pose(37, 121, Math.toRadians(0));

    private void buildPaths() {
        // Caminho direto até o ponto de lançamento
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Caminho de volta para pegar um artefato
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPose.getHeading())
                .build();

        // Caminho para retornar e lançar novamente
        scorePickup1 = follower.pathBuilder()
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
                // Ativa o shooter antes de começar
                Shooter.INSTANCE.shooterOn,
                // Segue até a zona de lançamento
                new FollowPath(scorePreload, true),
                // Pausa breve para garantir estabilidade e lançar
                new Delay(5),
                Shooter.INSTANCE.shooterOn,
                // Caminho até o pickup
                new FollowPath(grabPickup1, true, 0.8),
                Intake.INSTANCE.intakeOn,
                new Delay(1),
                Intake.INSTANCE.intakeOn,
                // Volta para lançar novamente
                new FollowPath(scorePickup1, true),
                Shooter.INSTANCE.shooterOn,
                new Delay(5),
                Shooter.INSTANCE.shooterOn
        );
    }
}
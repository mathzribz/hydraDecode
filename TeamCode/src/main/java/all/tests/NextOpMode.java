package all.tests;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import all.subsystems.Shooter;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp
@Config
@Configurable
public class NextOpMode extends NextFTCOpMode {

    public NextOpMode() {
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {

    }

    @Override
    public void onUpdate() {

        Gamepads.gamepad1().a().whenBecomesTrue(Shooter.INSTANCE.on);
        Gamepads.gamepad1().b().whenBecomesTrue(Shooter.INSTANCE.off);

        ActiveOpMode.telemetry().update();
    }

}

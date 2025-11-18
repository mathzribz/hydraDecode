package all.tests;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import all.subsystems.Flywheel;
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
                new SubsystemComponent(Flywheel.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        Gamepads.gamepad1().a().whenBecomesTrue(Flywheel.INSTANCE.lowR);
        Gamepads.gamepad1().a().whenBecomesTrue(Flywheel.INSTANCE.lowL);




//        Gamepads.gamepad1().b().whenBecomesTrue(Flywheel.INSTANCE.mid);
//        Gamepads.gamepad1().y().whenBecomesTrue(Flywheel.INSTANCE.high);
//        Gamepads.gamepad1().x().whenBecomesTrue(Flywheel.INSTANCE.offShooter);

    }

    @Override
    public void onUpdate() {

        ActiveOpMode.telemetry().update();
    }

}

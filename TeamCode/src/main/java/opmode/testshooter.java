
package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import subsystems.shooter;
@TeleOp
public class testshooter extends NextFTCOpMode {
    public testshooter() {
        addComponents(
                new SubsystemComponent(shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }


    @Override
    public void onStartButtonPressed() {
        Gamepads.gamepad1().y().whenBecomesTrue(shooter.INSTANCE.spinUp); //MotorPower
        Gamepads.gamepad1().x().whenBecomesTrue(shooter.INSTANCE.spinDown);
    }

    @Override
    public void onUpdate() {
        ActiveOpMode.telemetry().update();
    }
}

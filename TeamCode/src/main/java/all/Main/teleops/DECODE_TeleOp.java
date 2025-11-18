package all.Main.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import all.subsystems.Flywheel;
import all.subsystems.Intake;
import all.subsystems.Transfer;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
@TeleOp
public class DECODE_TeleOp extends NextFTCOpMode {

    public DECODE_TeleOp() {
        addComponents(

                new SubsystemComponent(Intake.INSTANCE,Transfer.INSTANCE, Flywheel.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {

    }

    @Override
    public void onUpdate() {



        //Shooter gamepad2
//        Gamepads.gamepad2().rightTrigger().greaterThan(0.1).whenBecomesTrue(Flywheel.INSTANCE.on);
//        Gamepads.gamepad1().rightTrigger().lessThan(0.1).whenBecomesTrue(Flywheel.INSTANCE.off);

        ActiveOpMode.telemetry().update();
}

}
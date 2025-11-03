package all.Main;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import all.configPedro.Constants;
import all.subsystems.Intake;
import all.subsystems.Shooter;
import all.subsystems.Transfer;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
@TeleOp
public class DECODE_TeleOp extends NextFTCOpMode {

    public DECODE_TeleOp() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intake.INSTANCE,Transfer.INSTANCE,Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {

    }



    @Override
    public void onUpdate() {


        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                false
        );
        driverControlled.schedule();

        Gamepads.gamepad1().leftTrigger().greaterThan(0.1).whenBecomesTrue(Intake.INSTANCE.intakeOn);
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(Intake.INSTANCE.intakeReverse);

        Gamepads.gamepad1().rightTrigger().greaterThan(0.1).whenBecomesTrue(Shooter.INSTANCE.shooterOn);
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(Transfer.INSTANCE.transferOn);
        Gamepads.gamepad1().y().whenBecomesTrue(Transfer.INSTANCE.transferReverse);

}
}

package all.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;


public class Flywheel implements Subsystem {
    public static final Flywheel INSTANCE = new Flywheel();
    private Flywheel() { }

    private final MotorEx motor = new MotorEx("shooter");


    private final ControlSystem controller = ControlSystem.builder()
            .velPid(0.005, 0, 0)
            .basicFF(0.01, 0.02, 0.03)
            .build();

   // public final Command off = new RunToVelocity(controller, 0.0).requires(this).named("FlywheelOff");
 //   public final Command on = new RunToVelocity(controller, 500.0).requires(this).named("FlywheelOn");
    public final Command on = new SetPower(motor,-0.75);
    public final Command onfar = new SetPower(motor,-1);
    public final Command off = new SetPower(motor,0);

    @Override
    public void initialize() {
        motor.brakeMode();
    }

    @Override
    public void periodic() {


       // motor.setPower(controller.calculate(motor.getState()));
        motor.setPower(motor.getPower());
    }
}
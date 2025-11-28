package all.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

/**
 * The flywheel subsystem implemented from
 * <a href="https://v1.nextftc.dev/guide/subsystems/flywheel">docs</a>.
 */
public class FlywheelPID implements Subsystem {
    public static final FlywheelPID INSTANCE = new FlywheelPID();
    private FlywheelPID() { }

    private final MotorEx Flywheel = new MotorEx("shooterL");
    private final MotorEx Flywheel2 = new MotorEx("shooterR");
    private final ControlSystem controller = ControlSystem.builder()
            .velPid(0.0006, 0, 0.00001)
            .basicFF(0.0015, 0., 0.0)
            .build();

    public final Command off = new RunToVelocity(controller, 0.0).requires(this).named("shooterL");
    public final Command on = new RunToVelocity(controller, 550).requires(this).named("shooterL");


    @Override
    public void initialize() {



        Flywheel.reverse();
        Flywheel2.reverse();

    }
    @Override
    public void periodic() {

        double output = controller.calculate(Flywheel.getState());
        Flywheel.setPower(output);
        Flywheel2.setPower(output);

    }
}
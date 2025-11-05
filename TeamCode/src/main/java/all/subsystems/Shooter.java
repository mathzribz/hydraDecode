package all.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() { }

    private final MotorEx shooterMotor = new MotorEx("shooter");

    private final ControlSystem controller = ControlSystem.builder()
            .velPid(0.005, 0, 0)
            .basicFF(0.01, 0.02, 0.03)
            .build();

    public final Command off = new RunToVelocity(controller, 0.0).requires(this).named("ShooterOff");
    public final Command on = new RunToVelocity(controller, 500.0).requires(this).named("ShooterOn");

    @Override
    public void initialize() {
        shooterMotor.reverse();

    }

    @Override
    public void periodic() {
        shooterMotor.setPower(controller.calculate(shooterMotor.getState()));

        ActiveOpMode.telemetry().addData("shooter State", shooterMotor.getState());
}

}
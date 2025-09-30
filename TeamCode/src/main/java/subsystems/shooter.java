
package subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Configurable
public class shooter implements Subsystem {
    public static final shooter INSTANCE = new shooter();
    private shooter() { }

    private final MotorEx motor = new MotorEx("shooter")
            .floatMode()
            .zeroed();

    // PIDF ajustes
    public static PIDCoefficients coefficients = new PIDCoefficients(0.0, 0.0, 0.0);
    public static BasicFeedforwardParameters feedforward =
            new BasicFeedforwardParameters(1.0/11200.0, 0, 0.05);

    private final ControlSystem controlSystem = ControlSystem.builder()
            .velPid(coefficients)
            .basicFF(feedforward)
            .build();

    // Comandos
    public final Command spinUp = new RunToVelocity(controlSystem, 5600).requires(motor);
    public final Command spinDown = new RunToVelocity(controlSystem, 0).requires(motor);

    @Override
    public void periodic() {
        motor.setPower(controlSystem.calculate(motor.getState()));

        // Debug
        ActiveOpMode.telemetry().addData("Shooter Vel", motor.getState().getVelocity());

    }
}
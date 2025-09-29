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
    public shooter() { }


    public static int a = 0;

    private boolean enabled = false;

    private MotorEx motor = new MotorEx("shooter")
            .floatMode();

    public static PIDCoefficients coefficients = new PIDCoefficients(0.1, 0, 0);
    public static BasicFeedforwardParameters feedforward = new BasicFeedforwardParameters(1, 0, 0);

    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(coefficients)
            .basicFF(feedforward)
            .build();

        public Command spinUp = new RunToVelocity(controlSystem, 2500, new KineticState(Double.POSITIVE_INFINITY, 500, Double.POSITIVE_INFINITY)).requires(motor);
    public Command spinDown = new RunToVelocity(controlSystem, 0).requires(motor);


    @Override
    public void periodic() {
        motor.setPower(controlSystem.calculate(motor.getState()));
    }

}
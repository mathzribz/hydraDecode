package subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
@Configurable
public class shooter implements Subsystem {
    public static final shooter INSTANCE = new shooter();
    public shooter() { }

    private boolean enabled = false;

    private MotorEx motor = new MotorEx("shooter")
            .floatMode();

    public static PIDCoefficients coefficients = new PIDCoefficients(0, 0, 0);
    public static BasicFeedforwardParameters feedforward = new BasicFeedforwardParameters(0, 0, 0);

    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(coefficients)
            .basicFF(feedforward)
            .build();


}
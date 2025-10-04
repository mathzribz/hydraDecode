
package subsystems;

import com.acmerobotics.dashboard.config.Config;
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
@Config
public class shooter implements Subsystem {
    public static final shooter INSTANCE = new shooter();
    public shooter() { }
    private MotorEx motor;
    private ControlSystem controlSystem;
    public static PIDCoefficients coefficients;
    public static BasicFeedforwardParameters feedforward;

    public Command spinUp;
    public Command spinDown;


    @Override
    public void initialize() {


         motor = new MotorEx("shooter")
                .zeroed()
                .floatMode();


 coefficients = new PIDCoefficients(0.1, 0, 0);
      feedforward = new BasicFeedforwardParameters(1, 0, 0);

       controlSystem = ControlSystem.builder()

                .basicFF(feedforward)
                .velPid(coefficients)
                .build();


        spinUp = new RunToVelocity(controlSystem, -100)
                .requires(motor);

        spinDown = new RunToVelocity(controlSystem, 0)
                .requires(motor);
    }



    @Override
    public void periodic() {
        motor.setPower(controlSystem.calculate(motor.getState()));

    }

}
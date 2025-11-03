package all.subsystems;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
@Config
public class FlyWheelTester implements Subsystem {
    public static final FlyWheelTester INSTANCE = new FlyWheelTester();
    private FlyWheelTester() {}
    private MotorEx flyMotor;
    private ControlSystem controlSystem;
    public static PIDCoefficients coefficients;
    public static BasicFeedforwardParameters feedforward;

    public static double kP = 0.0;
    public static double kV = 0.5;
    public static double kD = 0.0;
    public static double kA = 0.0;
    public static double velocity = 200;

    public Command intakeOn1;
    public Command intakeOn2;
    public Command intakeOff;

    @Override
    public void initialize() {
        flyMotor = new MotorEx("flyMotor");

        coefficients = new PIDCoefficients(kP, 0.0, kD);
        feedforward = new BasicFeedforwardParameters(kV, kA, 0.0);

        controlSystem = ControlSystem.builder()
                 .velPid(coefficients)
                .basicFF(feedforward)
                .build();

        intakeOn1 =
                new RunToVelocity(controlSystem, velocity).requires(this);

        intakeOn2 =
                new RunToVelocity(controlSystem, 500).requires(this);

        intakeOff =
                new RunToVelocity(controlSystem, 0).requires(this);

    }

    @Override
    public void periodic() {
        flyMotor.setPower(controlSystem.calculate(flyMotor.getState()));

        ActiveOpMode.telemetry().addData("flyMotor State", flyMotor.getState());
        ActiveOpMode.telemetry().addData("kP", kP);
        ActiveOpMode.telemetry().addData("kD", kD);
        ActiveOpMode.telemetry().addData("kV", kV);
        ActiveOpMode.telemetry().addData("kA", kA);
        ActiveOpMode.telemetry().addData("velocity", velocity   );
    }

}
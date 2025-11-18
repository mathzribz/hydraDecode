package all.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.CommandGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Config
@Configurable
public class Flywheel implements Subsystem {



    public static final Flywheel INSTANCE = new Flywheel();
    private Flywheel() { }

    private final MotorEx motor = new MotorEx("shooterL");
    private final MotorEx motor2 = new MotorEx("shooterR");
    public static double kP = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;
    public static double kS = 0.0;
    SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(kS, kV, kA);
    double result = feedforward.calculate(kV,kA);

    public static double speed = 0.5;

    double output = result * speed;



//    private final ControlSystem controller = ControlSystem.builder()
//            .velPid(kP, 0, kD)
//            .basicFF(kV, kA, kS)
//            .build();

//    public Command offShooter = new RunToVelocity(controller, 0);
    public Command lowR = new SetPower(motor,output );
    public Command lowL = new SetPower(motor2,output );
//    public Command mid = new RunToVelocity(controller, 100);
//    public Command high = new RunToVelocity(controller, 500);

    @Override
    public void initialize() {

    }

    @Override
    public void periodic() {
        motor.setPower(output);
        motor2.setPower(output);

        ActiveOpMode.telemetry().addData("State", motor.getPower());
        ActiveOpMode.telemetry().addData("State 2", motor2.getPower());
        ActiveOpMode.telemetry().addData("kP", kP);
        ActiveOpMode.telemetry().addData("kD", kD);
        ActiveOpMode.telemetry().addData("kV", kV);
        ActiveOpMode.telemetry().addData("kA", kA);
        ActiveOpMode.telemetry().addData("kS", kS);
    }


}
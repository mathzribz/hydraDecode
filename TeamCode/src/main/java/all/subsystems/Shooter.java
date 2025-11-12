package all.subsystems;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() { }

    private final MotorEx shooterMotor = new MotorEx("shooter");
    public static double shooterSpeed = 500.0;
    public static double kP = 0.1;
    public static double kD = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;
    public static double kS = 0.0;

    private final ControlSystem controller = ControlSystem.builder()
            .velPid(kP, 0, kD)
            .basicFF(kV, kA, kS)
            .build();

    public final Command off = new RunToVelocity(controller, 0.0).requires(this).named("ShooterOff");
    public final Command on = new RunToVelocity(controller, shooterSpeed).requires(this).named("ShooterOn");

    @Override
    public void initialize() {
        shooterMotor.reverse();

    }

    @Override
    public void periodic() {
        shooterMotor.setPower(controller.calculate(shooterMotor.getState()));


        ActiveOpMode.telemetry().addData("shooter State", shooterMotor.getState());
        ActiveOpMode.telemetry().addData("shooter Speed", shooterSpeed);
        ActiveOpMode.telemetry().addData("kP", kP);
        ActiveOpMode.telemetry().addData("kD", kD);
        ActiveOpMode.telemetry().addData("kV", kV);
        ActiveOpMode.telemetry().addData("kA", kA);
        ActiveOpMode.telemetry().addData("kS", kS);
}

}
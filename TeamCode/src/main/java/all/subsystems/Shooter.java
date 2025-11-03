package all.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() {}

    private MotorEx shooterMotor;
    public static double shooterSpeed = 0.9;

    public Command shooterOn;
    public Command shooterSpeed_90;
    public Command shooterSpedd_75;

    @Override
    public void initialize() {
        shooterMotor = new MotorEx("shooter");
        shooterMotor.reverse();

        shooterOn =
                new SetPower(shooterMotor, shooterSpeed).requires(this);

    }

    @Override
    public void periodic() {
        shooterMotor.setPower(0);

    }

}

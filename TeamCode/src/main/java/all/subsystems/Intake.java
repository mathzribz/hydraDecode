package all.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() {}

    private MotorEx intakeMotor;

    public Command intakeOn;
    public Command intakeReverse;

    @Override
    public void initialize() {
        intakeMotor = new MotorEx("intake");
        intakeMotor.reverse();

        intakeOn =
                new SetPower(intakeMotor, 1).requires(this);

        intakeReverse =
                new SetPower(intakeMotor, -1).requires(this);

    }

    @Override
    public void periodic() {
        intakeMotor.setPower(0);

    }

}

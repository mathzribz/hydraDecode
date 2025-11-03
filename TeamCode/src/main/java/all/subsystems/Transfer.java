package all.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    private Transfer() {}

    private MotorEx transferMotor;

    public Command transferOn;
    public Command transferReverse;

    @Override
    public void initialize() {
        transferMotor = new MotorEx("transfer");
        transferMotor.reverse();

        transferOn =
                new SetPower(transferMotor, 1).requires(this);

        transferReverse =
                new SetPower(transferMotor, -1).requires(this);

    }

    @Override
    public void periodic() {
        transferMotor.setPower(0);

    }

}

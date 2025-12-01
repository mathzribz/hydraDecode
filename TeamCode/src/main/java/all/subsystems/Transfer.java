package all.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    private Transfer() { }

    private final MotorEx transferMotor = new MotorEx("transfer");

    public final Command on = new SetPower(transferMotor,0.5);
    public final Command onin = new SetPower(transferMotor,0.25);
    public final Command off = new SetPower(transferMotor,0);

    @Override
    public void initialize() {

    }

    @Override
    public void periodic() {
        transferMotor.setPower(transferMotor.getPower());

        ActiveOpMode.telemetry().addData("transfer State", transferMotor.getState());
    }

}
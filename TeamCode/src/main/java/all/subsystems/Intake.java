package all.subsystems;


import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Config
public class  Intake implements Subsystem {

    private double intakespeed = 0.8;
    public static final Intake INSTANCE = new Intake();
    private Intake() { }

    private final MotorEx intakeMotor = new MotorEx("intake");

    public final Command on = new SetPower(intakeMotor,0.3);
    public final Command onin = new SetPower(intakeMotor,-intakespeed);
    public final Command onkeep = new SetPower(intakeMotor,-0.45);
    public final Command off = new SetPower(intakeMotor,0);
    @Override
    public void initialize() {

    }

    @Override
    public void periodic() {

        intakeMotor.setPower(intakeMotor.getPower());

        ActiveOpMode.telemetry().addData("intake State", intakeMotor.getState());

    }

}
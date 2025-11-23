package all.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Config
public class  Intake implements Subsystem {


    private boolean transferEnabled = true;
    private double intakespeed = 0.8;
    public static final Intake INSTANCE = new Intake();
    private Intake() { }

    private final MotorEx intakeMotor = new MotorEx("intake");



    public final Command on = new SetPower(intakeMotor,1);
    public final Command onin = new SetPower(intakeMotor,intakespeed);
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
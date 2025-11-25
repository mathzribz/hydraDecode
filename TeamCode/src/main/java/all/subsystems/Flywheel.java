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
public class  Flywheel implements Subsystem {


    private boolean transferEnabled = true;
    private double FlywheelSpeed = 0.88;
    public static final Flywheel INSTANCE = new Flywheel();
    private Flywheel() { }

    private final MotorEx Flywheel = new MotorEx("shooterL");
    private final MotorEx Flywheel2 = new MotorEx("shooterR");



    public final Command on = new SetPower(Flywheel,FlywheelSpeed);
    public final Command onin = new SetPower(Flywheel2,FlywheelSpeed);
    public final Command off = new SetPower(Flywheel,0);
    public final Command off2 = new SetPower(Flywheel2,0);
//    public final Command off = new SetPower(intakeMotor,0);
    @Override
    public void initialize() {
        Flywheel.reverse();
        Flywheel2.reverse();

    }

    @Override
    public void periodic() {

        Flywheel.setPower(Flywheel.getPower());
        Flywheel2.setPower(Flywheel2.getPower());




        ActiveOpMode.telemetry().addData("Flywheel State", Flywheel.getState());

    }

}
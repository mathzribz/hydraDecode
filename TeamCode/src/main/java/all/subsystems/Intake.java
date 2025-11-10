package all.subsystems;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Config
public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() { }

    private final MotorEx intakeMotor = new MotorEx("intake");
    public static double intakeSpeed = 500.0;
    public static double kP = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;
    public static double kS = 0.0;

    private final ControlSystem controller = ControlSystem.builder()
            .velPid(kP, 0, kD)
            .basicFF(kV, kA, kS)
            .build();

   // public final Command off = new RunToVelocity(controller, 0.0).requires(this).named("IntakeOff");
   // public final Command on = new RunToVelocity(controller, intakeSpeed).requires(this).named("IntakeOn");
    public final Command reverse = new RunToVelocity(controller, -intakeSpeed).requires(this).named("IntakeReverse");
    public final Command on = new SetPower(intakeMotor,0.8);
    public final Command onin = new SetPower(intakeMotor,0.45);
    public final Command off = new SetPower(intakeMotor,0);
    @Override
    public void initialize() {


    }

    @Override
    public void periodic() {
      //  intakeMotor.setPower(controller.calculate(intakeMotor.getState()));
        intakeMotor.setPower(intakeMotor.getPower());

        ActiveOpMode.telemetry().addData("intake State", intakeMotor.getState());
        ActiveOpMode.telemetry().addData("inatke Speed", intakeSpeed);
        ActiveOpMode.telemetry().addData("kP", kP);
        ActiveOpMode.telemetry().addData("kD", kD);
        ActiveOpMode.telemetry().addData("kV", kV);
        ActiveOpMode.telemetry().addData("kA", kA);
        ActiveOpMode.telemetry().addData("kS", kS);
    }

}
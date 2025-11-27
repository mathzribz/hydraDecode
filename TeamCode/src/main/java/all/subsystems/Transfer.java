package all.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    private Transfer() { }




    private final MotorEx transferMotor = new MotorEx("transfer");
    public static double transferSpeed = 500.0;
    public static double kP = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;
    public static double kS = 0.0;

    private final ControlSystem controller = ControlSystem.builder()
            .velPid(kP, 0, kD)
            .basicFF(kV, kA, kS)
            .build();

   // public final Command off = new RunToVelocity(controller, 0.0).requires(this).named("TransferOff");
  //  public final Command on = new RunToVelocity(controller, transferSpeed).requires(this).named("TransferOn");
    public final Command reverse = new RunToVelocity(controller, -transferSpeed).requires(this).named("TransferReverse");

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
        ActiveOpMode.telemetry().addData("Shooter Speed", transferSpeed);
        ActiveOpMode.telemetry().addData("kP", kP);
        ActiveOpMode.telemetry().addData("kD", kD);
        ActiveOpMode.telemetry().addData("kV", kV);
        ActiveOpMode.telemetry().addData("kA", kA);
        ActiveOpMode.telemetry().addData("kS", kS);}

}
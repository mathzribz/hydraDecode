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

    DistanceSensor dd;
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
//        dd = hardwareMap.get(DistanceSensor.class, "dd");

    }

    @Override
    public void periodic() {

        intakeMotor.setPower(intakeMotor.getPower());


//        double distance = dd.getDistance(DistanceUnit.CM);
//        boolean ballDetected = (distance > 7 && distance < 8);
//
//        // Se detectou bola → trava o transfer controlado pelo trigger
//        if (ballDetected) {
//            transferEnabled = false;
//        }
//
//        if (!transferEnabled){
//            intakespeed = 0;
//        }
//        else{
//            intakespeed = 0.8;
//        }


        ActiveOpMode.telemetry().addData("intake State", intakeMotor.getState());

    }

}

package all.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Shooter extends SubsystemBase {

    public Shooter(final HardwareMap hwMap, String name) {
        m1 = new MotorEx(hwMap, "shooterL");
        m2 = new MotorEx(hwMap, "shooterR");
        servo = hwMap.get(Servo.class, "hood");

    }

    private MotorEx m1, m2;
    private final Servo servo;
    public static double kP = 0.00065;
    public static double kI = 0.0;
    public static double kD = 0.00001;
    public static double kF = 0.0015;

    public static double TICKS_PER_REV = 28;
    public static double targetRPM = 1200;
    public static double targetTPS = (targetRPM / 60.0) * TICKS_PER_REV;
    private final PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    double Lp = m1.getVelocity();
    double pidPower = pidf.calculate(Lp, targetTPS);

    public double getPidPower() {
        return  pidPower = Math.max(-1.0, Math.min(1.0, pidPower));
    }

    public void ShooterOn() {
        m1.set(getPidPower());
        m2.set(getPidPower());

    }
    public void ShooterOff() {
        m1.set(0);
        m2.set(0);
        pidf.reset();

    }




    @Override
    public void periodic() {
        telemetry.addData("RPM ", Lp);
        telemetry.addData("targetRPM", targetRPM);
        telemetry.update();
    }

}

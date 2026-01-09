package all.Subsystems;



import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import all.Configs.Turret.TurretConstants;

public class Turret extends SubsystemBase {

    private final DcMotorEx motor;
    private final PIDController pid;

    private double targetDegrees = 0;

    public Turret(DcMotorEx motor) {
        this.motor = motor;
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        pid = new PIDController(
                TurretConstants.kP,
                0,
                TurretConstants.kD
        );
    }

    public double getAngle() {
        return motor.getCurrentPosition() * TurretConstants.DEGREES_PER_TICK;
    }

    public void setTarget(double degrees) {
        targetDegrees = normalize(degrees);
    }

    @Override
    public void periodic() {
        double error = normalize(targetDegrees - getAngle());
        double power = pid.calculate(error);
        motor.setPower(power);
    }

    private double normalize(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}

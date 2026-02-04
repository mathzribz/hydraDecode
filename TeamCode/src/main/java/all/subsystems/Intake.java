
package all.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake extends SubsystemBase {

    private final MotorEx motor;
    private final Servo gate;

    private final DistanceSensor up, down;
    private final ElapsedTime fullTimer = new ElapsedTime();

    private boolean countingFull = false;
    private boolean enabledTransfer = true;

    public static boolean upBlocked, downBlocked;

    private double motorPower = 0;

    public Intake(HardwareMap hw) {
        motor = new MotorEx(hw, "intake");
        gate  = hw.get(Servo.class, "gate");

        up   = hw.get(DistanceSensor.class, "up");
        down = hw.get(DistanceSensor.class, "down");

    }

    public void updateAutoLogic() {

         upBlocked   = up.getDistance(DistanceUnit.CM) < 8;
         downBlocked = down.getDistance(DistanceUnit.CM) < 8;

        if (upBlocked && downBlocked) {
            if (!countingFull) {
                fullTimer.reset();
                countingFull = true;
            }

            if (fullTimer.seconds() >= 0.08) {
                enabledTransfer = false;
                motorPower = 0;
            }

        } else {
            countingFull = false;
            fullTimer.reset();
        }
    }

    // ===============================
    // CONTROLES
    // ===============================
    public void intakeOn() {
        if (enabledTransfer) {
            motorPower = -0.9;
        }
    }

    public void intakeOut() {
        motorPower = 0.75;
    }

    public void intakeStop() {
        motorPower = 0;
    }

    public void Transfer() {
        enabledTransfer = true;
        countingFull = false;
        fullTimer.reset();
        motorPower = -1.0;
    }

    public void gateOpen() {
        gate.setPosition(0.2);
    }

    public void gateClose() {
        gate.setPosition(0.3);
    }

    // ===============================
    // LOOP DO SUBSYSTEM
    // ===============================
    @Override
    public void periodic() {
        updateAutoLogic();
        motor.set(motorPower);
    }

    // ===============================
    // TELEMETRIA (opcional)
    // ===============================
    public boolean isTransferEnabled() {
        return enabledTransfer;
    }

    public double getFullTime() {
        return fullTimer.seconds();
    }
}

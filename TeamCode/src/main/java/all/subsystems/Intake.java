
package all.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake extends SubsystemBase {

    private final MotorEx motor;
    private final Servo gate;

    private final DistanceSensor mid, down;

    private NormalizedColorSensor up;

    private final ElapsedTime fullTimer = new ElapsedTime();

    private boolean countingFull = false;
    private boolean enabledTransfer = true;

    public boolean useSensors = true;

    public static boolean upBlocked, midBlocked, downBlocked;

    private double motorPower = 0;

    public Intake(HardwareMap hw) {
        motor = new MotorEx(hw, "intake");
        gate  = hw.get(Servo.class, "gate");

        mid   = hw.get(DistanceSensor.class, "mid");
        down = hw.get(DistanceSensor.class, "down");
        up = hw.get(NormalizedColorSensor.class, "up");

    }

    public void middateAutoLogic() {
        if (useSensors) {

            midBlocked = mid.getDistance(DistanceUnit.CM) < 9;
            downBlocked = down.getDistance(DistanceUnit.CM) < 9;
            upBlocked = ((DistanceSensor) up).getDistance(DistanceUnit.CM) < 6.5;

            if (upBlocked && midBlocked && downBlocked) {
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
    }

    public void intakeOn() {
        if (enabledTransfer) {
            motorPower = -0.9;
        }
    }

    public void intakeOut() {
        enabledTransfer = true;
        countingFull = false;
        fullTimer.reset();
        motorPower = 0.75;
    }

    public void intakeStop() {
        motorPower = 0;
    }

    public void TransferTeleop() {
        enabledTransfer = true;
        countingFull = false;
        fullTimer.reset();
        motorPower = -1.0;

    }
    public void TransferAuto() {
        motorPower = -1.0;
    }

    public void gateOpen() {
        gate.setPosition(0.16);
    }

    public void gateClose() {
        gate.setPosition(0.33);
    }

    @Override
    public void periodic() {
        middateAutoLogic();
        motor.set(motorPower);
    }

    public boolean isTransferEnabled() {
        return enabledTransfer;
    }

    public double getFullTime() {
        return fullTimer.seconds();
    }
}

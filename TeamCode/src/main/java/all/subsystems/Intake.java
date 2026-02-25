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
    private final NormalizedColorSensor up;

    private final ElapsedTime fullTimer = new ElapsedTime();
    private final ElapsedTime launchTimer = new ElapsedTime();

    public static double MIN_LAUNCH_INTERVAL = 0.3;

    public boolean countingFull = false;
    private boolean launchCooldownActive = false;
    private boolean lastUpBlocked = false;

    public static boolean upBlocked, midBlocked, downBlocked, useSensor;
    public static boolean allblocked = false;

    private double motorPower = 0;

    private enum Mode {
        OFF,
        INTAKE,
        OUTTAKE,
        TRANSFER,
        TRANSFER_SENSOR
    }

    private Mode mode = Mode.OFF;

    public Intake(HardwareMap hw) {
        motor = new MotorEx(hw, "intake");
        gate  = hw.get(Servo.class, "gate");

        mid   = hw.get(DistanceSensor.class, "mid");
        down  = hw.get(DistanceSensor.class, "down");
        up    = hw.get(NormalizedColorSensor.class, "up");
    }

    private void updateSensors() {
        midBlocked  = mid.getDistance(DistanceUnit.CM) < 9;
        downBlocked = down.getDistance(DistanceUnit.CM) < 9;
        upBlocked   = ((DistanceSensor) up).getDistance(DistanceUnit.CM) < 6.5;

    }

    private void runFullDetection() {
        if (upBlocked && midBlocked && downBlocked) {
            if (!countingFull) {
                fullTimer.reset();
                countingFull = true;
                allblocked = true;
            }

            if (fullTimer.seconds() >= 0.5) {
                motorPower = 0;
                return;
            }

        } else {
            countingFull = false;
            fullTimer.reset();
        }
    }

    private void runSensorTransferLogic() {
        if(useSensor) {

            if (lastUpBlocked && !upBlocked) {
                launchCooldownActive = true;
                launchTimer.reset();
            }

            lastUpBlocked = upBlocked;

            if (launchCooldownActive) {
                if (launchTimer.seconds() < MIN_LAUNCH_INTERVAL) {
                    motorPower = 0;
                    return;
                }
                launchCooldownActive = false;
            }

            motorPower = -1.0;
        }
    }

    public void intakeOn() {
        mode = Mode.INTAKE;
    }

    public void intakeOnAuto() {
        mode = Mode.INTAKE;
    }

    public void intakeOut() {
        mode = Mode.OUTTAKE;
    }

    public void transferTeleop() {
        mode = Mode.TRANSFER;
    }
    public void transferAuto() {
        mode = Mode.TRANSFER;
    }

    public void transferSensor() {mode = Mode.TRANSFER_SENSOR;}

    public void transferSensorAuto() {mode = Mode.TRANSFER_SENSOR;}

    public void intakeStop() {
        mode = Mode.OFF;
    }

    public void gateOpen() {
        gate.setPosition(0.16);
    }

    public void gateClose() {
        gate.setPosition(0.33);
    }

    @Override
    public void periodic() {


        updateSensors();

        switch (mode) {

            case OFF:
                motorPower = 0;
                break;

            case INTAKE:
                useSensor = true;
                runFullDetection();
                if (!countingFull) {
                    motorPower = -1.0;
                }
                break;

            case OUTTAKE:
                motorPower = 0.75;
                break;

            case TRANSFER:
                useSensor = false;
                motorPower = -1.0;
                allblocked = false;
                break;

            case TRANSFER_SENSOR:
                useSensor = true;
                runSensorTransferLogic();
                allblocked = false;
                break;
        }

        motor.set(motorPower);
    }

    public double getMotor() {
        return motor.getVelocity();
    }


}
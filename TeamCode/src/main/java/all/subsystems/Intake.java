package all.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake extends SubsystemBase {

    private final MotorEx motor;
    private final Servo gate;

    // ===== SENSOR UP =====
    private final NormalizedColorSensor up;
    private boolean upBlocked = false;
    private boolean lastUpBlocked = false;

    private final ElapsedTime launchTimer = new ElapsedTime();
    public static double MIN_LAUNCH_INTERVAL = 0.7;
    private boolean launchCooldownActive = false;

    private double motorPower = 0;

    // ===== MODOS =====
    private enum Mode {
        OFF,
        INTAKE,
        OUTTAKE,
        TRANSFER,
        TRANSFERFAR,
        TRANSFER_SENSOR
    }

    private Mode mode = Mode.OFF;

    public Intake(HardwareMap hw) {
        motor = new MotorEx(hw, "intake");
        gate  = hw.get(Servo.class, "gate");
        up    = hw.get(NormalizedColorSensor.class, "up");

        motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
    }

    // ==============================
    // ===== SENSOR UPDATE ==========
    // ==============================

    private void updateUpSensor() {
        upBlocked = ((DistanceSensor) up).getDistance(DistanceUnit.CM) < 6.5;
    }

    private void runSensorTransferLogic() {

        // Detecta borda de saída do pixel
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

        motorPower = -0.86;
    }

    // ==============================
    // ===== COMANDOS TELEOP ========
    // ==============================

    public void intakeOn() { mode = Mode.INTAKE; }
    public void intakeStop() { mode = Mode.OFF; }
    public void intakeOut() { mode = Mode.OUTTAKE; }
    public void transferTeleop() { mode = Mode.TRANSFER; }
    public void transferFarTeleop() { mode = Mode.TRANSFERFAR; }
    public void transferSensor() { mode = Mode.TRANSFER_SENSOR; }

    // ==============================
    // ===== COMANDOS AUTO ==========
    // ==============================

    public void intakeOnAuto() { mode = Mode.INTAKE; }
    public void transferAuto() { mode = Mode.TRANSFER; }
    public void transferSensorAuto() { mode = Mode.TRANSFER_SENSOR; }

    // ==============================
    // ===== GATE ===================
    // ==============================

    public void gateOpen() { gate.setPosition(0.16); }
    public void gateClose() { gate.setPosition(0.31); }

    // ==============================
    // ===== PERIODIC ===============
    // ==============================

    @Override
    public void periodic() {

        updateUpSensor();

        switch (mode) {

            case OFF:
                motorPower = 0;
                break;

            case INTAKE:
                motorPower = -0.92;
                break;

            case OUTTAKE:
                motorPower = 0.75;
                break;

            case TRANSFER:
                motorPower = -1.0;
                break;

            case TRANSFERFAR:
                motorPower = -0.4;
                break;

            case TRANSFER_SENSOR:
                runSensorTransferLogic();
                break;
        }

        motor.set(motorPower);
    }

    // ==============================
    // ===== CURRENT MONITOR =========
    // ==============================

    public double getCurrentAmps() {
        return motor.motorEx.getCurrent(CurrentUnit.AMPS);
    }

    public double getMotorVelocity() {
        return motor.getVelocity();
    }

    public double getMotorPower() {
        return motorPower;
    }

    public boolean isUpBlocked() {
        return upBlocked;
    }
}
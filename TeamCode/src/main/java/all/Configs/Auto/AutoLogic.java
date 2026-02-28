
package all.Configs.Auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import all.subsystems.Intake;
import all.subsystems.Shooter;

public class AutoLogic {

    private Intake intake;
    private Shooter shooter;


    public final ElapsedTime timer = new ElapsedTime();

    public enum ShooterState {
        IDLE,
        PRESPIN,
        BURST_FIRE,
        STOPPING
    }

    public ShooterState state = ShooterState.IDLE;

    private static final double MIN_RPM = 1900;
    private static final double MAX_SPINUP_TIME = 2.0;
    private static final double BURST_TIME = 2; // tempo suficiente pra 3 bolas
    private static final double BURST_TIME_FAR = 4; // tempo suficiente pra 3 bolas

    public void init(HardwareMap hw) {
        shooter = new Shooter(hw);
        intake = new Intake(hw);



        intake.gateOpen();

    }
    public void preSpin() {
        state = ShooterState.PRESPIN;
            shooter.shooterOnAuto();
            shooter.setTargetRPM(2220);
            shooter.HoodLow();


    }
    public void stopShooter() {
        state = ShooterState.STOPPING;

            shooter.setTargetRPM(0);


    }

    public void preSpinFar() {
        state = ShooterState.PRESPIN;
        shooter.shooterOnAuto();
        shooter.setTargetRPM(2870);
        shooter.HoodHigh();

    }

    public void burstFire() {
        if (state == ShooterState.PRESPIN) {
            openGate();
            intake.transferAuto();
            timer.reset();
            if (timer.seconds() >= BURST_TIME) {
                intake.intakeStop();
            }
        }
    }

    public void burstFireFar() {
        if (state == ShooterState.PRESPIN) {
            openGate();
            intake.transferSensorAuto();
            timer.reset();
            if (timer.seconds() >= BURST_TIME_FAR) {
                intake.intakeStop();
            }
        }
    }

    public void stopAll() {
        shooter.shooterOff();
        intake.intakeStop();

        state = ShooterState.IDLE;
    }

    public boolean readyToFire() {
        return shooter.getCurrentRPM() >= MIN_RPM;
    }

    public boolean isBusy() {
        return state != ShooterState.IDLE;
    }


    public void startIntake() {

        closeGate();
        intake.intakeOnAuto();
    }

    public void stopIntake() {
        intake.intakeStop();
    }

    public void openGate() {
        intake.gateOpen();
    }

    public void closeGate() {
        intake.gateClose();
    }

    public boolean getUpBlocke(){
        return intake.isUpBlocked();
    }


    public void update() {


        switch (state) {

            case IDLE:
                break;

            case PRESPIN:
                break;

            case BURST_FIRE:
                if (timer.seconds() >= BURST_TIME) {
                    intake.intakeStop();
                    state = ShooterState.STOPPING;
                }
                break;

            case STOPPING:
                break;
        }


    }
}

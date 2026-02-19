
package all.Configs.StateMachines;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import all.subsystems.Intake;
import all.subsystems.Shooter;

public class ShooterLogic {

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
    private static final double BURST_TIME = 4; // tempo suficiente pra 3 bolas

    public void init(HardwareMap hw) {
        shooter = new Shooter(hw);
        intake = new Intake(hw);

        intake.useSensors = false;
        shooter.shooterOff();
        intake.gateOpen();
        intake.intakeStop();
    }
    public void preSpin() {
        if (state == ShooterState.IDLE) {
            shooter.shooterOn();
            shooter.setTargetRPM(2200);
            intake.gateOpen();
            state = ShooterState.PRESPIN;
            timer.reset();
        }
    }

    public void burstFire() {
        if (state == ShooterState.PRESPIN) {
            intake.TransferAuto();
            timer.reset();
            state = ShooterState.BURST_FIRE;
        }
    }

    public void stopAll() {
        shooter.shooterOff();
        intake.intakeStop();
        intake.gateClose();
        state = ShooterState.IDLE;
    }

    public boolean readyToFire() {
        return shooter.getCurrentRPM() >= MIN_RPM;
    }

    public boolean isBusy() {
        return state != ShooterState.IDLE;
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
                stopAll();
                break;
        }

    }
}


package all.Configs.Auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import all.subsystems.Intake;
import all.subsystems.Shooter;
import all.subsystems.Turret;
import static all.Configs.Turret.FieldConstants.BLUE_GOAL;

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

    public void init(HardwareMap hw) {
        shooter = new Shooter(hw);
        intake = new Intake(hw);


        intake.useSensors = false;
        intake.gateOpen();

    }
    public void preSpin() {
        if (state == ShooterState.IDLE) {
            shooter.shooterOn();
            shooter.setTargetRPM(2050);
            shooter.HoodLow();
            state = ShooterState.PRESPIN;
            timer.reset();
        }
    }

    public void burstFire() {
        if (state == ShooterState.PRESPIN) {
            intake.useSensors = false;
            intake.TransferAuto();
            timer.reset();
            state = ShooterState.BURST_FIRE;
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


    public void startIntakeWithSensors() {
        intake.useSensors = true;
        intake.gateClose();
        intake.intakeOn();
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

    public boolean intakeFull() {
        return !intake.isTransferEnabled();
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

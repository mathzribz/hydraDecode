package all.Configs.StateMachines;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import all.subsystems.Intake;
import all.subsystems.Shooter;

public class ShooterLogic {

    private Intake intake;
    private Shooter shooter;

    private final ElapsedTime timer = new ElapsedTime();

    private enum ShooterState {
        IDLE,
        PRESPIN,
        BURST_FIRE,
        STOPPING
    }

    private ShooterState state = ShooterState.IDLE;

    private static final double MIN_RPM = 1900;
    private static final double MAX_SPINUP_TIME = 2.0;
    private static final double BURST_TIME = 1.5; // tempo suficiente pra 3 bolas

    public void init(HardwareMap hw) {
        shooter = new Shooter(hw);
        intake = new Intake(hw);

        shooter.shooterOff();
        intake.gateClose();
        intake.intakeStop();
    }


    public void preSpin() {
        if (state == ShooterState.IDLE) {
            shooter.shooterOn();
            intake.gateOpen(); // pode ficar aberto
            state = ShooterState.PRESPIN;
            timer.reset();
        }
    }


    public void burstFire() {
        if (state == ShooterState.PRESPIN) {
            intake.Transfer();
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
        return shooter.getCurrentRPM() >= MIN_RPM
                || timer.seconds() > MAX_SPINUP_TIME;
    }

    public boolean isBusy() {
        return state != ShooterState.IDLE;
    }

    /* ================= UPDATE ================= */

    public void update() {

        switch (state) {

            case IDLE:
                break;

            case PRESPIN:
                // apenas mantém flywheel ligada
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

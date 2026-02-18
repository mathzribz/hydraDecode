package all.Configs.StateMachines;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import all.subsystems.Intake;
import all.subsystems.Shooter;

public class ShooterLogic {
    Intake intake;
    Shooter shooter;

    private ElapsedTime stateTimer = new ElapsedTime();

    private enum ShooterState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        RESET_GATE
    }

    private ShooterState shooterState;

    private double GATE_OPEN_TIME = 0.4;
    private double GATE_CLOSE_TIME = 0.4;

    private int shotsRemaining = 0;

    private double shooterVel = 0;
    private double MIN_RPM = 1900;
    private double TARGET_RPM = 2300;
    private double SHOOTER_MAX_SPINUP_TIME = 2;

    public void init(HardwareMap hwmap){

        shooter = new Shooter(hwmap);
        intake = new Intake(hwmap);

        shooterState = ShooterState.IDLE;

        intake.gateOpen();
        shooter.HoodHigh();
    }


    public void update() {
        switch (shooterState){
            case IDLE:
                if (shotsRemaining > 0 ){

                    shooter.shooterOn();

                    stateTimer.reset();
                    shooterState = ShooterState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if (shooter.getCurrentRPM() > MIN_RPM || stateTimer.seconds() > SHOOTER_MAX_SPINUP_TIME){
                    intake.gateOpen();
                    intake.Transfer();
                    stateTimer.reset();

                    shooterState = ShooterState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (stateTimer.seconds() > GATE_OPEN_TIME){
                    shotsRemaining--;
                    stateTimer.reset();

                    shooterState = ShooterState.RESET_GATE;
                }
                break;
            case RESET_GATE:
                if (stateTimer.seconds() > GATE_CLOSE_TIME){
                     if (shotsRemaining > 0){
                         stateTimer.reset();

                         shooterState = ShooterState.SPIN_UP;
                     }
                     else {
                         shooter.shooterOff();
                         shooterState = ShooterState.IDLE;
                         intake.gateClose();

                     }
                }
                break;
        }

        CommandScheduler.getInstance().run();


    }


    public  void fireShots(int shots){
        if (shooterState == ShooterState.IDLE){
            shotsRemaining = shots;
        }
    }

    public void offAll(){
        shooter.shooterOff();
        intake.intakeStop();
    }

    public void spin(){
        shooter.shooterOn();
        intake.gateOpen();

    }
    public void launch(){
        intake.Transfer();
    }


    public boolean isBusy(){
        return shooterState != ShooterState.IDLE;

    }



}

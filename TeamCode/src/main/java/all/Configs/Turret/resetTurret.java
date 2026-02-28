package all.Configs.Turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import all.subsystems.Turret;
@TeleOp
public class resetTurret extends LinearOpMode {

    Turret turret ;


    @Override
    public void runOpMode() throws InterruptedException {
        initT();



        waitForStart();

        while (opModeIsActive()){

            turret.resetEncoder();

        }


    }

    public  void initT(){
        turret = new Turret(hardwareMap);
    }
}

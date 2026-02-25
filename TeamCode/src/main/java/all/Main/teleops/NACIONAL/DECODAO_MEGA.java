package all.Main.teleops.NACIONAL;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import all.Configs.Teleop.TeleopLogic;

@Config
@TeleOp
public class DECODAO_MEGA extends CommandOpMode {

    private final TeleopLogic teleopLogic = new TeleopLogic();
    private ElapsedTime elapsedtime;

    @Override
    public void initialize() {
        teleopLogic.init(hardwareMap);
        teleopLogic.startPoseTreino();
        teleopLogic.startLL(0);
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();
    }

    @Override
    public void run() {
        waitForStart();
        super.run();

//        teleopLogic.bulkRead();

        teleopLogic.update();
        teleopLogic.turretWorking("BLUE");
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
        telemetry.update();

    }
}

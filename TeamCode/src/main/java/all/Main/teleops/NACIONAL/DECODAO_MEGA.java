package all.Main.teleops.NACIONAL;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
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
    private GamepadEx gamepad1Ex;

    @Override
    public void initialize() {
        teleopLogic.init(hardwareMap);
        gamepad1Ex = new GamepadEx(gamepad1);
        teleopLogic.startPoseTreino();
        teleopLogic.startLL(0);
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();


        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void run() {
        waitForStart();
        super.run();
        teleopLogic.shooterWorking(gamepad1Ex);
        teleopLogic.intakeWorking(gamepad1Ex);
        teleopLogic.led();
//        teleopLogic.telemetryUpdate();

//        teleopLogic.bulkRead();


        teleopLogic.turretWorking("BLUE", gamepad1Ex);
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
        telemetry.update();

    }
}

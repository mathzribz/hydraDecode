
package all.Tests;

import static all.Configs.Turret.FieldConstants.BLUE_GOAL;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import all.Configs.Pedro.Constants;
import all.subsystems.Turret;

@TeleOp
public class testTurretCommand extends CommandOpMode {

    private Follower follower;
    private GamepadEx gamepads1;
    private Turret subsystem;

    @Override
    public void initialize() {
        gamepads1 = new GamepadEx(gamepad1);

        subsystem = new Turret(hardwareMap, "subsystem");
        follower = Constants.createFollower(hardwareMap);

        Pose startPos = new Pose(
                0,      // X do field
                0,      // Y do field
                Math.toRadians(90) // heading (pode ser usado em outra lógica)
        );
        follower.setStartingPose(startPos);


    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();

            subsystem.seguirPose(BLUE_GOAL);


            double headingDeg = subsystem.getHeadingDeg();
            double currentDeg = subsystem.getAngleDeg();
            double targetDeg  = subsystem.getAngleDeg(); // turrets sempre convergem pro targetTicks
            double errorDeg   = currentDeg - targetDeg;

            telemetry.addData("=== TURRET STATE ===", "");


            telemetry.addData("Robot Heading (deg)", "%.2f", headingDeg);

            telemetry.addData("Turret Angle (deg)", "%.2f", currentDeg);
            telemetry.addData("Turret Target (deg)", "%.2f", targetDeg);
            telemetry.addData("Turret Error (deg)", "%.2f", errorDeg);



            telemetry.addData("PID Fast (kp)", "%.4f", Turret.kpFast);
            telemetry.addData("PID Slow (kp)", "%.4f", Turret.kpSlow);

            telemetry.addData("D-pad Left  = Manual Left", "true");
            telemetry.addData("D-pad Right = Manual Right", "true");
            telemetry.addData("Button A    = Follow Pose", "true");

            telemetry.update();
            follower.update();
        }
        reset();
    }



}

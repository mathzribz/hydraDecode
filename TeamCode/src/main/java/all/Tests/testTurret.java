package all.Tests;

import static all.Configs.Pedro.Tuning.follower;
import static all.Configs.Turret.FieldConstants.BLUE_GOAL;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.geometry.Pose;
import all.Subsystems.Turret;

@TeleOp(name="Debug Turret - Manual + Auto", group="Test")
public class testTurret extends LinearOpMode {

    private Turret turret;



    @Override
    public void runOpMode() {

        turret = new Turret(hardwareMap);
        Pose startPos = new Pose(
                60,      // X do field
                85,      // Y do field
                Math.toRadians(135) // heading (pode ser usado em outra lógica)
        );
        follower.setStartingPose(startPos);

        waitForStart();

        while (opModeIsActive()) {

            // ----------------------
            // 1) MANUAL CONTROLE
            // ----------------------
            if (gamepad1.dpad_left) {
                turret.manual(-0.5);
            } else if (gamepad1.dpad_right) {
                turret.manual(0.5);
            } else {
                turret.automatic();
            }

            // ----------------------
            // 2) AUTO SEGUIR POSE
            // ----------------------
            if (gamepad1.a) {
                // EXEMPLO – troque por qualquer pose do Pedro

                turret.seguirPose(BLUE_GOAL);
            }

            // ----------------------
            // 3) TELEMETRIA
            // ----------------------

            // heading do robô (Pinpoint)
            double headingDeg = turret.getHeadingDeg();
            double currentDeg = turret.getAngleDeg();
            double targetDeg  = turret.getAngleDeg(); // turrets sempre convergem pro targetTicks
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
            CommandScheduler.getInstance().run();
        }
    }
}

package all.Main.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.Timer;

@Autonomous
public class SwitchCaseAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {

        DRIVE_STARTPOSE_SCOREPOSE,
        SCORE,
        DRIVE_SCOREPOSE_REPOPOSE1

    }

    PathState pathState;

    private final Pose starterPose = new Pose(20, 123, 138);
    private final Pose scorePose = new Pose(46, 98, 138);
    private final Pose repoPose1 = new Pose(18, 87, 90);

    @Override
    public void init() {
    }

    @Override
    public void loop() {
    }

}

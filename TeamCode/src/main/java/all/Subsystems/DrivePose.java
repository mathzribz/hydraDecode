package all.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;

public class DrivePose extends SubsystemBase {

    private Pose pose = new Pose(0,0,0);

    public Pose getPose() {
        return pose;
    }

    public void setPose(Pose p) {
        pose = p;
    }
}

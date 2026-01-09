package all.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.pedropathing.geometry.Pose;

import java.util.Optional;

public class LLturret extends SubsystemBase {

    private final Limelight3A ll;

    public LLturret(Limelight3A ll) {
        this.ll = ll;
        ll.start();
    }

    public Optional<Pose> getRobotPose() {
        LLResult res = ll.getLatestResult();
        if (res == null || !res.isValid()) return Optional.empty();

        // MegaTag 3D já retorna pose absoluta
        double x = res.getBotpose_MT2().getPosition().x;
        double y = res.getBotpose_MT2().getPosition().y;
        double heading = res.getBotpose_MT2().getOrientation().getYaw();

        return Optional.of(new Pose(x, y, heading));
    }
}

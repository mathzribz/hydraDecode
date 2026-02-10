
package all.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Optional;

public class LLMegatag extends SubsystemBase {

    private final Limelight3A ll;

    public LLMegatag(HardwareMap hw) {
        ll = hw.get(Limelight3A.class, "limelight");
    }

    public Pose getPedroRobotPose() {

        LLResult res = ll.getLatestResult();


        Pose3D mtPose = res.getBotpose_MT2();

        Pose2D ftcStandart =  new Pose2D(
                DistanceUnit.METER,
                -mtPose.getPosition().x,
                mtPose.getPosition().y,
                AngleUnit.RADIANS,
                mtPose.getOrientation().getYaw()
        );

        Pose ftcPose2d = PoseConverter.pose2DToPose(ftcStandart, InvertedFTCCoordinates.INSTANCE);

        Pose pedroPose =
                ftcPose2d.getAsCoordinateSystem(
                        PedroCoordinates.INSTANCE
                );

        return pedroPose;
    }

    public void start() {
        ll.start();
    }

    public void stop() {
        ll.stop();
    }

    public LLResult hasTarget() {
        LLResult res = ll.getLatestResult();

        return  res;
    }

    public void switchPipeline(int id) {
        ll.pipelineSwitch(id);
    }


    public boolean isPoseReliable() {
        LLResult res = ll.getLatestResult();
        if (res == null || !res.isValid()) return false;

        return res.getBotpose_MT2() != null;
    }




}
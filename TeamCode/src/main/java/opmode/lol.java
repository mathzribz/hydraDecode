package opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;


import config.pedro.constants.FConstants;
import config.pedro.constants.LConstants;

@Autonomous
public class lol  extends PedroOpMode {

    private final Pose start = new Pose(4, 24, Math.toRadians(180));
    private final Pose score   = new Pose(30, 60, Math.toRadians(180));
    private final Pose slide1  = new Pose(30, 28, Math.toRadians(180));
    private final Pose slide2   = new Pose(58, 28, Math.toRadians(180));



    private PathChain PathToScore, Slideone, Slidetwo;

    @Override
    public void onInit() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(start);
        buildPaths();
    }




    private void buildPaths() {
        PathToScore = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(start), new Point(score))))
                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading())
                .build();

        Slideone = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(score), new Point(slide1))))
                .setLinearHeadingInterpolation(score.getHeading(), slide1.getHeading())
                .build();

        Slidetwo = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(slide1), new Point(slide2))))
                .setLinearHeadingInterpolation(slide1.getHeading(), slide2.getHeading())
                .build();

    }
    @Override
    public void onStartButtonPressed() {
        firstRoutine().invoke();
    }

    public Command firstRoutine() {
        return new SequentialGroup(
      new FollowPath(PathToScore),
      new FollowPath(Slideone),
      new FollowPath(Slidetwo)
);

} }

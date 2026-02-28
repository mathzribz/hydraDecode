
package all.Main.Autos.AZUL;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import all.Configs.Auto.PoseStorage;
import all.Configs.Panels.Drawing;
import all.Configs.Pedro.Constants;
import all.Configs.Auto.AutoLogic;
import all.subsystems.Turret;

@Autonomous
public class blue_far_spike extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private AutoLogic autologic = new AutoLogic();
    private Turret turret;

    public ElapsedTime elapsedtime = new ElapsedTime();

    public enum PathState {

        DRIVE_STARTPOSE_SCOREPOSE,
        START_SCORE,

        DRIVE_TO_SPIKE,
        COLLECT1,
        SHOOT1,

        DRIVE_TO_LOAD,
        LOAD,
        COLLECT2,
        SHOOT2,


        DRIVE_TO_LOAD2,
        COLLECT3,
        SHOOT3,

        DRIVE_TO_LOAD3,
        COLLECT4,
        SHOOT4,

        END
    }

    PathState pathState;

    // ===== PATHS =====
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;
    public PathChain Path11;


    public void Paths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(64.000, 8.000),

                                new Pose(60.000, 21.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 21.000),
                                new Pose(63.672, 36.490),
                                new Pose(24.640, 35.993),
                                new Pose(13.699, 38)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(12.699, 38),
                                new Pose(55.861, 35.165),
                                new Pose(60.000, 21.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();


        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 21.000),
                                new Pose(36.898, 20.770),
                                new Pose(8.5, 25.774)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(205))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(8.5, 25.774),

                                new Pose(8.5, 6.300)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(205))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.698, 6.300),

                                new Pose(60.000, 21.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(205), Math.toRadians(180))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 21.000),

                                new Pose(10.624, 10.699)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.624, 10.699),

                                new Pose(60.000, 21.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 21.000),

                                new Pose(10.624, 10.699)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.624, 10.699),

                                new Pose(60.000, 21.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
    }

    public void statePathUpdate() {

        switch (pathState) {

            case DRIVE_STARTPOSE_SCOREPOSE:
                autologic.preSpinFar();

                follower.followPath(Path1,0.9, true);
                pathTimer.resetTimer();
                setPathState(PathState.START_SCORE);


                break;

            case START_SCORE:

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.7){
                    autologic.burstFireFar();
                    pathTimer.resetTimer();
                    setPathState(PathState.DRIVE_TO_SPIKE);

                }
                break;

            // ================= CICLO 1 =================

            case DRIVE_TO_SPIKE:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.7) {
                    autologic.stopShooter();
                    follower.followPath(Path2, 0.7,true);
                    autologic.startIntake();
                    pathTimer.resetTimer();
                    setPathState(PathState.COLLECT1);
                }
                break;

            case COLLECT1:
                if (!follower.isBusy() ) {
                    pathTimer.resetTimer();

                    if ( pathTimer.getElapsedTimeSeconds() > 0.8) {
                        autologic.stopIntake();
                    }
                    follower.followPath(Path4,0.9, true);
                    autologic.preSpinFar();
                    pathTimer.resetTimer();
                    setPathState(PathState.SHOOT1);
                }
                break;


            case SHOOT1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){
                    autologic.burstFireFar();
                    pathTimer.resetTimer();
                    setPathState(PathState.DRIVE_TO_LOAD);

                }

                break;

            // ================= CICLO 2 =================

            case DRIVE_TO_LOAD:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.7) {
                    autologic.stopShooter();
                    follower.followPath(Path5,0.9,true);

                    setPathState(PathState.LOAD);
                }
                break;


            case LOAD:
                if (!follower.isBusy()) {
                    autologic.startIntake();
                    follower.followPath(Path6,0.75, true);
                    pathTimer.resetTimer();
                    setPathState(PathState.COLLECT2);


                }
                break;

            case COLLECT2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    autologic.stopIntake();

                    follower.followPath(Path7,0.9, true);
                    autologic.preSpinFar();
                    pathTimer.resetTimer();
                    setPathState(PathState.SHOOT2);


                }

                break;

            case SHOOT2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){
                    autologic.burstFireFar();
                    pathTimer.resetTimer();
                    setPathState(PathState.DRIVE_TO_LOAD2);

                }

                break;

            case DRIVE_TO_LOAD2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.7) {
                    autologic.stopShooter();
                    follower.followPath(Path8,0.9,true);
                    autologic.startIntake();
                    setPathState(PathState.COLLECT3);
                }
                break;


        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        Drawing.init();

        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        autologic.init(hardwareMap);
        turret = new Turret(hardwareMap);
        elapsedtime.reset();

        Paths();
        follower.setStartingPose(new Pose(64, 8, Math.toRadians(180)));

        pathState = PathState.DRIVE_STARTPOSE_SCOREPOSE;

    }

    public void stop(){
        autologic.stopAll();

    }

    @Override
    public void loop() {


        CommandScheduler.getInstance().run();
        follower.update();
        turret.holdRobotRelative(Math.toRadians(-64.7), follower.getHeading());
        autologic.update();
        statePathUpdate();


        try {
            Drawing.drawDebug(follower);
        } catch (Exception e) {
            telemetry.addLine("drawing failed");
        }


        PoseStorage.currentPose = follower.getPose();

        telemetry.addData("State", pathState);
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        telemetry.addData("Up Blocked", autologic.getUpBlocke());
        elapsedtime.reset();
    }
}
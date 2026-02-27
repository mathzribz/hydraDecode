
package all.Main.Autos.AZUL;

import static all.Configs.Turret.FieldConstants.BLUE_GOAL_Auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import all.Configs.Auto.PoseStorage;
import all.Configs.Pedro.Constants;
import all.Configs.Auto.AutoLogic;
import all.subsystems.Turret;

@Autonomous
public class blue_far_spike extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private AutoLogic autologic = new AutoLogic();
    private Turret turret;

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

                                new Pose(70.000, 21.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(70.000, 21.000),
                                new Pose(73.747, 38.301),
                                new Pose(12.699, 35.774)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.699, 35.774),

                                new Pose(70.000, 21.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(70.000, 21.000),
                                new Pose(31.161, 6.237),
                                new Pose(10.731, 26.237)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(190), Math.toRadians(245))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.731, 26.237),

                                new Pose(10.624, 10.699)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(245))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.624, 10.699),

                                new Pose(70.000, 21.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(190))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(70.000, 21.000),

                                new Pose(10.624, 10.699)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(190))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.624, 10.699),

                                new Pose(70.000, 21.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(190))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(70.000, 21.000),

                                new Pose(10.624, 10.699)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(190))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.624, 10.699),

                                new Pose(70.000, 21.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(190))

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(70.000, 21.000),

                                new Pose(69.442, 32.203)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(190))

                .build();
    }

    public void statePathUpdate() {
        turret.followPose(BLUE_GOAL_Auto,follower.getPose(),follower.getHeading());

        switch (pathState) {

            case DRIVE_STARTPOSE_SCOREPOSE:
                autologic.preSpinFar();

                follower.followPath(Path1,0.9, true);
                setPathState(PathState.START_SCORE);

                pathTimer.resetTimer();

                break;

            case START_SCORE:

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.2){
                    autologic.burstFireFar();
                    setPathState(PathState.DRIVE_TO_SPIKE);

                }
                break;

            // ================= CICLO 1 =================

            case DRIVE_TO_SPIKE:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    autologic.stopShooter();
                    follower.followPath(Path2, 0.9,true);
                    autologic.startIntakeWithSensors();
                    setPathState(PathState.COLLECT1);
                }
                break;

            case COLLECT1:
                if (!follower.isBusy()) {
                    follower.followPath(Path3,0.9, true);
                    setPathState(PathState.SHOOT1);
                    autologic.preSpinFar();
                }
                break;


            case SHOOT1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    autologic.burstFireFar();
                    setPathState(PathState.DRIVE_TO_LOAD);

                }

                break;

            // ================= CICLO 2 =================

            case DRIVE_TO_LOAD:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    autologic.stopShooter();
                    follower.followPath(Path4,0.9,true);

                    setPathState(PathState.LOAD);
                }
                break;


            case LOAD:
                if (!follower.isBusy()) {
                    autologic.startIntakeWithSensors();
                    follower.followPath(Path5,0.9, true);
                    setPathState(PathState.COLLECT2);

                }
                break;

            case COLLECT2:
                if (!follower.isBusy()) {

                    follower.followPath(Path6,0.9, true);
                    setPathState(PathState.SHOOT2);
                    autologic.preSpinFar();
                }
                pathTimer.resetTimer();
                break;

            case SHOOT2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    autologic.burstFireFar();
                    setPathState(PathState.DRIVE_TO_LOAD2);

                }
                break;


            case DRIVE_TO_LOAD2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    autologic.stopShooter();
                    follower.followPath(Path7,0.9,true);
                    autologic.startIntakeWithSensors();
                    setPathState(PathState.COLLECT3);
                }
                break;

            case COLLECT3:
                if (!follower.isBusy()) {

                    follower.followPath(Path8,0.9, true);
                    setPathState(PathState.SHOOT3);
                    autologic.preSpinFar();
                }
                pathTimer.resetTimer();
                break;

            case SHOOT3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    autologic.burstFireFar();
                    setPathState(PathState.DRIVE_TO_LOAD3);

                }
                pathTimer.resetTimer();

                break;

            case DRIVE_TO_LOAD3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    autologic.stopShooter();
                    follower.followPath(Path9,0.9,true);
                    autologic.startIntakeWithSensors();
                    setPathState(PathState.COLLECT4);
                }
                break;

            case COLLECT4:
                if (!follower.isBusy()) {

                    follower.followPath(Path10,0.9, true);
                    setPathState(PathState.SHOOT4);
                    autologic.preSpinFar();
                }
                pathTimer.resetTimer();
                break;

            case SHOOT4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    autologic.burstFireFar();
                    setPathState(PathState.END);

                }
                pathTimer.resetTimer();

                break;

            case END:
                if ( pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(Path11,0.9, true);
                    autologic.stopAll();
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

        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        autologic.init(hardwareMap);
        turret = new Turret(hardwareMap);

        Paths();
        follower.setStartingPose(new Pose(64, 8, Math.toRadians(180)));

        pathState = PathState.DRIVE_STARTPOSE_SCOREPOSE;

    }

    @Override
    public void loop() {


        CommandScheduler.getInstance().run();
        follower.update();
        autologic.update();
        statePathUpdate();



        PoseStorage.currentPose = follower.getPose();

        telemetry.addData("State", pathState);
    }
}
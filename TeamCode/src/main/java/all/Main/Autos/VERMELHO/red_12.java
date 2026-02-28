package all.Main.Autos.VERMELHO;

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
import all.Configs.Pedro.Constants;
import all.Configs.Auto.AutoLogic;
import all.subsystems.Turret;

@Autonomous
public class red_12 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private AutoLogic autologic = new AutoLogic();
    private Turret turret;

    public ElapsedTime elapsedtime = new ElapsedTime();

    public enum PathState {

        DRIVE_STARTPOSE_SCOREPOSE,
        START_SCORE,

        DRIVE_TO_INTAKE1,
        COLLECT1,
        SHOOT1,

        DRIVE_TO_INTAKE2,
        COLLECT2,
        SHOOT2,

        DRIVE_TO_GATE,

        DRIVE_TO_INTAKE3,
        COLLECT3,
        SHOOT3,

        END,
        PARk
    }

    PathState pathState;

    public PathChain Path1, Path2, PathD, Path3, Path4, Path5, Path6, Path7, Path8;

    // ===== FUNÇÃO DE ESPELHAMENTO =====
    private double mirrorX(double x){
        return 144 - x;
    }

    public void Paths() {

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(33.118), 135.312),
                                new Pose(mirrorX(51), 93)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(mirrorX(51), 93),
                                new Pose(mirrorX(71.017), 56.623),
                                new Pose(mirrorX(56.952), 59.498),
                                new Pose(mirrorX(15.531), 58.811)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        PathD = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(mirrorX(16.531), 58.811),
                                new Pose(mirrorX(43.699), 66.714),
                                new Pose(mirrorX(17.161), 69.054)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(mirrorX(15.161), 72.054),
                                new Pose(mirrorX(56.629), 68.55925806451614),
                                new Pose(mirrorX(51), 93)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(mirrorX(51), 93),
                                new Pose(mirrorX(88.206), 26.392),
                                new Pose(mirrorX(12.242), 35.418)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(13.242), 35.418),
                                new Pose(mirrorX(51), 93)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(mirrorX(51), 93),
                                new Pose(mirrorX(57.676), 80.508),
                                new Pose(mirrorX(12.695), 84.505)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(13.695), 84.505),
                                new Pose(mirrorX(51), 93)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(51.000), 93.000),
                                new Pose(mirrorX(31.000), 93.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    public void statePathUpdate() {

        switch (pathState) {

            case DRIVE_STARTPOSE_SCOREPOSE:
                autologic.preSpin();
                follower.followPath(Path1, 0.9, true);
                setPathState(PathState.START_SCORE);
                break;

            case START_SCORE:
                if (!follower.isBusy()) {
                    autologic.burstFire();
                    setPathState(PathState.DRIVE_TO_INTAKE1);
                }
                break;

            case DRIVE_TO_INTAKE1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.8) {
                    autologic.stopShooter();
                    follower.followPath(Path2, 0.9, true);
                    autologic.startIntake();
                    setPathState(PathState.COLLECT1);
                }
                break;

            case COLLECT1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.2) {
                    autologic.stopIntake();
                    follower.followPath(PathD, 0.9, true);
                    setPathState(PathState.DRIVE_TO_GATE);
                }
                break;

            case DRIVE_TO_GATE:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    autologic.preSpin();
                    follower.followPath(Path3, 0.9, true);
                    setPathState(PathState.SHOOT1);
                }
                break;

            case SHOOT1:
                if (!follower.isBusy()) {
                    autologic.burstFire();
                    setPathState(PathState.DRIVE_TO_INTAKE2);
                }
                break;

            case DRIVE_TO_INTAKE2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.8) {
                    autologic.stopShooter();
                    follower.followPath(Path4, 0.9, true);
                    autologic.startIntake();
                    setPathState(PathState.COLLECT2);
                }
                break;

            case COLLECT2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.2) {
                    autologic.stopIntake();
                    follower.followPath(Path5, 0.9, true);
                    autologic.preSpin();
                    setPathState(PathState.SHOOT2);
                }
                break;

            case SHOOT2:
                if (!follower.isBusy()) {
                    autologic.burstFire();
                    setPathState(PathState.DRIVE_TO_INTAKE3);
                }
                break;

            case DRIVE_TO_INTAKE3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.8) {
                    autologic.stopShooter();
                    follower.followPath(Path6, 0.9, true);
                    autologic.startIntake();
                    setPathState(PathState.COLLECT3);
                }
                break;

            case COLLECT3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.2) {
                    autologic.stopIntake();
                    follower.followPath(Path7, 0.9, true);
                    autologic.preSpin();
                    setPathState(PathState.SHOOT3);
                }
                break;

            case SHOOT3:
                if (!follower.isBusy()) {
                    autologic.burstFire();
                    setPathState(PathState.END);
                }
                break;

            case END:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.8) {
                    follower.followPath(Path8, 0.9, true);
                    setPathState(PathState.PARk);
                }
                break;

            case PARk:
                autologic.stopAll();
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
        elapsedtime.reset();

        Paths();

        follower.setStartingPose(
                new Pose(mirrorX(33.118279569892465),
                        135.31182795698925,
                        Math.toRadians(180))
        );

        pathState = PathState.DRIVE_STARTPOSE_SCOREPOSE;
    }

    @Override
    public void loop() {

        CommandScheduler.getInstance().run();
        follower.update();
        turret.holdRobotRelative(Math.toRadians(55), follower.getHeading());
        turret.periodic();
        autologic.update();
        statePathUpdate();

        PoseStorage.currentPose = follower.getPose();

        telemetry.addData("State", pathState);
        telemetry.addData("tickError", turret.getTickError());
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
    }
}

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
public class blue_12 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private AutoLogic autologic = new AutoLogic();
    private Turret turret;

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

        END
    }

    PathState pathState;

    // ===== PATHS =====
    public PathChain Path1;
    public PathChain Path2;
    public PathChain PathD;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;


    public void Paths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(33.118, 135.312),

                                new Pose(51, 93)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(51, 93),
                                new Pose(71.017, 56.623),
                                new Pose(56.952, 59.498),
                                new Pose(17.531, 58.811)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();



        PathD = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(17.531, 58.811),
                                new Pose(20.400, 63.325),
                                new Pose(15.161, 72.054)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.161, 72.054),
                                new Pose(56.629, 68.55925806451614),
                                new Pose(51, 93)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(51, 93),
                                new Pose(88.206, 26.392),
                                new Pose(14.242, 35.418)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(14.242, 35.418),

                                new Pose(51, 93)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(51, 93),
                                new Pose(57.676, 80.508),
                                new Pose(12.695, 84.505)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.695, 84.505),

                                new Pose(51, 93)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(51.000, 93.000),

                                new Pose(31.000, 93.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

    }

    public void statePathUpdate() {
        turret.followPose(BLUE_GOAL_Auto,follower.getPose(),follower.getHeading());

        switch (pathState) {

            case DRIVE_STARTPOSE_SCOREPOSE:
                autologic.preSpin();


                    follower.followPath(Path1,0.9, true);
                    setPathState(PathState.START_SCORE);

                pathTimer.resetTimer();

                break;

            case START_SCORE:

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.2){
                    autologic.burstFire();
                    setPathState(PathState.DRIVE_TO_INTAKE1);

                }
                        break;

            // ================= CICLO 1 =================

            case DRIVE_TO_INTAKE1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    autologic.stopShooter();
                    follower.followPath(Path2, 0.9,true);
                    autologic.startIntakeWithSensors();
                    setPathState(PathState.COLLECT1);
                }
                break;

            case COLLECT1:
                if (!follower.isBusy()) {
                    follower.followPath(PathD,0.9, true);
                    setPathState(PathState.DRIVE_TO_GATE);
                    autologic.preSpin();
                }
                break;

            case DRIVE_TO_GATE:
                if (!follower.isBusy()) {

                    follower.followPath(Path3,0.9, true);
                    setPathState(PathState.SHOOT1);
                }
                pathTimer.resetTimer();
                break;

            case SHOOT1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    autologic.burstFire();
                    setPathState(PathState.DRIVE_TO_INTAKE2);

                }

                break;

            // ================= CICLO 2 =================

            case DRIVE_TO_INTAKE2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    autologic.stopShooter();
                    follower.followPath(Path4,0.9,true);
                    autologic.startIntakeWithSensors();
                    setPathState(PathState.COLLECT2);
                }
                break;

            case COLLECT2:
                if (!follower.isBusy()) {

                    follower.followPath(Path5,0.9, true);
                    setPathState(PathState.SHOOT2);
                    autologic.preSpin();
                }
                pathTimer.resetTimer();
                break;

            case SHOOT2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    autologic.burstFire();
                    setPathState(PathState.DRIVE_TO_INTAKE3);

                }
                break;


            case DRIVE_TO_INTAKE3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    autologic.stopShooter();
                    follower.followPath(Path6,0.9,true);
                    autologic.startIntakeWithSensors();
                    setPathState(PathState.COLLECT3);
                }
                break;

            case COLLECT3:
                if (!follower.isBusy()) {

                    follower.followPath(Path7,0.9, true);
                    setPathState(PathState.SHOOT3);
                    autologic.preSpin();
                }
                pathTimer.resetTimer();
                break;

            case SHOOT3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    autologic.burstFire();
                    setPathState(PathState.END);

                }
                pathTimer.resetTimer();

                break;

            case END:
                if ( pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(Path8,0.9, true);
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
        follower.setStartingPose(new Pose(33.118279569892465, 135.31182795698925, Math.toRadians(180)));

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
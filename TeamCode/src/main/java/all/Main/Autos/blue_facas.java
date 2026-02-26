
package all.Main.Autos;

import static all.Configs.Turret.FieldConstants.BLUE_GOAL;
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
public class blue_facas extends OpMode {

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

    // ===== POSES =====
    private final Pose starterPose = new Pose(17.537,119.723,Math.toRadians(142));
    private final Pose scorePose   = new Pose(38.17,102.26,Math.toRadians(142));

    private final Pose repo1 = new Pose(35.37,84.10,Math.toRadians(180));
    private final Pose intake1 = new Pose(16.37,84.10,Math.toRadians(180));
    private final Pose intake2 = new Pose(16.66,59.66,Math.toRadians(180));
    private final Pose intake3 = new Pose(15.07,36.47,Math.toRadians(180));
    private final Pose gate    = new Pose(16.21,69.37,Math.toRadians(-90));

    // ===== PATHS =====
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Pathd;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;

    public void Paths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(33.118, 135.312),

                                new Pose(51.613, 93.419)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(51.613, 93.419),
                                new Pose(71.017, 56.623),
                                new Pose(56.952, 59.498),
                                new Pose(17.531, 58.811)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(17.531, 58.811),
                                new Pose(56.629, 85.177),
                                new Pose(51.742, 93.398)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(51.742, 93.398),
                                new Pose(88.206, 26.392),
                                new Pose(14.242, 35.418)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(14.242, 35.418),

                                new Pose(52.086, 93.624)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.086, 93.624),
                                new Pose(57.676, 80.508),
                                new Pose(12.695, 84.505)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.695, 84.505),

                                new Pose(51.613, 93.419)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();


        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(51.989, 93.742),
                                new Pose(57.161, 83.828),
                                new Pose(16.548, 83.978)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.548, 83.978),

                                new Pose(51.61290322580646, 93.4193548387097)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public void statePathUpdate() {
        turret.followPose(BLUE_GOAL_Auto,follower.getPose(),follower.getHeading());
        autologic.preSpin();

        switch (pathState) {

            case DRIVE_STARTPOSE_SCOREPOSE:


                if (opModeTimer.getElapsedTimeSeconds() > 0.25) {
                    follower.followPath(Path1,1, true);
                    setPathState(PathState.START_SCORE);
                }
                pathTimer.resetTimer();

                break;

            case START_SCORE:

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    autologic.burstFire();
                    setPathState(PathState.DRIVE_TO_INTAKE1);

                }
                        break;

            // ================= CICLO 1 =================

            case DRIVE_TO_INTAKE1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {

                    follower.followPath(Path2, 0.9,true);
                    autologic.startIntakeWithSensors();
                    setPathState(PathState.COLLECT1);
                }
                break;

            case COLLECT1:
                if (!follower.isBusy()) {
                    follower.followPath(Path3,1, true);
                    setPathState(PathState.SHOOT1);
                }
                break;

            case SHOOT1:
                if (!follower.isBusy()) {
                    autologic.burstFire();
                    setPathState(PathState.DRIVE_TO_INTAKE2);
                    pathTimer.resetTimer();
                }

                break;

            // ================= CICLO 2 =================

            case DRIVE_TO_INTAKE2:
                if (!follower.isBusy()  && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(Path4, 0.9,true);
                    autologic.startIntakeWithSensors();

                    setPathState(PathState.COLLECT2);
                    pathTimer.resetTimer();
                }
                break;

            case COLLECT2:
                if (!follower.isBusy()) {

                    follower.followPath(Path5, true);
                    setPathState(PathState.SHOOT2);
                }
                break;

            case SHOOT2:
                if (!follower.isBusy() ) {
                    autologic.burstFire();
                    setPathState(PathState.DRIVE_TO_INTAKE3);
                    pathTimer.resetTimer();
                }

                break;


            case DRIVE_TO_INTAKE3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(Path6, 0.9,true);
                    autologic.startIntakeWithSensors();
                    setPathState(PathState.COLLECT3);
                    pathTimer.resetTimer();
                }
                break;

            case COLLECT3:
                if (!follower.isBusy()) {

                    follower.followPath(Path7, true);
                    setPathState(PathState.SHOOT3);
                }
                break;

            case SHOOT3:
                if (!follower.isBusy()) {
                    autologic.burstFire();
                    pathTimer.resetTimer();

                }

                break;

            case END:
                if ( pathTimer.getElapsedTimeSeconds() > 2) {
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
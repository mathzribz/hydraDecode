package all.Main.Autos;

import static all.Configs.Turret.FieldConstants.BLUE_GOAL;

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

    private AutoLogic shooterLogic = new AutoLogic();
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
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(51.613, 93.419),
                                new Pose(70.091, 57.317),
                                new Pose(55.156, 58.661),
                                new Pose(18.226, 60.505)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(18.226, 60.505),
                                new Pose(56.629, 85.177),
                                new Pose(51.742, 93.398)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(51.742, 93.398),
                                new Pose(57.366, 49.419),
                                new Pose(10.538, 59.032)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(140))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(10.538, 59.032),
                                new Pose(37.070, 68.269),
                                new Pose(51.989, 93.742)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(140))
                .setReversed()
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(51.989, 93.742),
                                new Pose(57.366, 49.419),
                                new Pose(10.538, 59.032)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(140))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(10.538, 59.032),
                                new Pose(37.070, 68.269),
                                new Pose(51.989, 93.742)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(140))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(51.989, 93.742),
                                new Pose(57.161, 83.828),
                                new Pose(16.548, 83.978)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.548, 83.978),

                                new Pose(52.086, 93.624)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public void statePathUpdate() {

        switch (pathState) {

            case DRIVE_STARTPOSE_SCOREPOSE:

                shooterLogic.preSpin();
                if (opModeTimer.getElapsedTimeSeconds() > 0.25) {
                    follower.followPath(Path1, true);
                    setPathState(PathState.START_SCORE);
                }
                pathTimer.resetTimer();

                break;

            case START_SCORE:

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    shooterLogic.burstFire();
                    setPathState(PathState.START_SCORE);
                }



                break;

            // ================= CICLO 1 =================

            case DRIVE_TO_INTAKE1:
                if (!follower.isBusy()) {

                    follower.followPath(Path2, true);
                    shooterLogic.startIntakeWithSensors();
                    setPathState(PathState.COLLECT1);
                }
                break;

            case COLLECT1:
                if (shooterLogic.intakeFull()) {
                    shooterLogic.stopIntake();
                    follower.followPath(Path3, true);
                    setPathState(PathState.SHOOT1);
                }
                break;

            case SHOOT1:
                if (!follower.isBusy() && shooterLogic.readyToFire()) {
                    shooterLogic.openGate();
                    shooterLogic.burstFire();
                    setPathState(PathState.DRIVE_TO_INTAKE2);
                }



//                break;
//
//            // ================= CICLO 2 =================
//
//            case DRIVE_TO_INTAKE2:
//                if (!follower.isBusy()) {
//                    shooterLogic.startIntakeWithSensors();
//                    setPathState(PathState.COLLECT2);
//                }
//                break;
//
//            case COLLECT2:
//                if (shooterLogic.intakeFull()) {
//                    shooterLogic.stopIntake();
//                    follower.followPath(back2, true);
//                    setPathState(PathState.SHOOT2);
//                }
//                break;
//
//            case SHOOT2:
//                if (!follower.isBusy() && shooterLogic.readyToFire()) {
//                    shooterLogic.openGate();
//                    shooterLogic.burstFire();
//                }
//                if (!shooterLogic.isBusy()) {
//                    follower.followPath(to_gate, true);
//                    setPathState(PathState.DRIVE_TO_GATE);
//                }
//                break;
//
//            // ================= GATE =================
//
//            case DRIVE_TO_GATE:
//                if (!follower.isBusy()) {
//                    follower.followPath(to_intake3, true);
//                    setPathState(PathState.DRIVE_TO_INTAKE3);
//                }
//                break;
//
//            // ================= CICLO 3 =================
//
//            case DRIVE_TO_INTAKE3:
//                if (!follower.isBusy()) {
//                    shooterLogic.startIntakeWithSensors();
//                    setPathState(PathState.COLLECT3);
//                }
//                break;
//
//            case COLLECT3:
//                if (shooterLogic.intakeFull()) {
//                    shooterLogic.stopIntake();
//                    follower.followPath(back3, true);
//                    setPathState(PathState.SHOOT3);
//                }
//                break;
//
//            case SHOOT3:
//                if (!follower.isBusy() && shooterLogic.readyToFire()) {
//                    shooterLogic.openGate();
//                    shooterLogic.burstFire();
//                }
                if (!shooterLogic.isBusy()) {
                    setPathState(PathState.END);
                }
                break;

            case END:
                shooterLogic.stopAll();
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
        shooterLogic.init(hardwareMap);
        turret = new Turret(hardwareMap);

        Paths();
        follower.setStartingPose(new Pose(33.118279569892465, 135.31182795698925, Math.toRadians(180)));

        pathState = PathState.DRIVE_STARTPOSE_SCOREPOSE;

    }

    @Override
    public void loop() {
        turret.followPose(BLUE_GOAL,follower.getPose(),follower.getHeading());

        CommandScheduler.getInstance().run();
        follower.update();
        shooterLogic.update();
        statePathUpdate();

        PoseStorage.currentPose = follower.getPose();

        telemetry.addData("State", pathState);
    }
}
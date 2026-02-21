package all.Main.Autos;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
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
    PathChain start_score;
    PathChain to_intake1,repo1d, back1;
    PathChain to_intake2, back2;
    PathChain to_gate;
    PathChain to_intake3, back3;

    public void buildPaths() {

        start_score = follower.pathBuilder()
                .addPath(new BezierLine(starterPose, scorePose))
                .build();

        repo1d = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, repo1))
                .build();
  to_intake1 = follower.pathBuilder()
                .addPath(new BezierLine(repo1, intake1))
                .build();

        back1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, scorePose))
                .build();

        to_intake2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intake2))
                .build();

        back2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2, scorePose))
                .build();

        to_gate = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, gate))
                .build();

        to_intake3 = follower.pathBuilder()
                .addPath(new BezierLine(gate, intake3))
                .build();

        back3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3, scorePose))
                .build();
    }

    public void statePathUpdate() {

        switch (pathState) {

            case DRIVE_STARTPOSE_SCOREPOSE:

                shooterLogic.preSpin();
                if (opModeTimer.getElapsedTimeSeconds() > 0.25) {
                    follower.followPath(start_score, true);
                    setPathState(PathState.START_SCORE);
                }
                pathTimer.resetTimer();

                break;

            case START_SCORE:

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    shooterLogic.burstFire();
                }

                if (!shooterLogic.isBusy()) {
                    setPathState(PathState.START_SCORE);
                    follower.followPath(repo1d, true);
                }

                break;

            // ================= CICLO 1 =================

            case DRIVE_TO_INTAKE1:
                if (!follower.isBusy()) {

                    follower.followPath(to_intake1, true);
                    shooterLogic.startIntakeWithSensors();
                    setPathState(PathState.COLLECT1);
                }
                break;

            case COLLECT1:
                if (shooterLogic.intakeFull()) {
                    shooterLogic.stopIntake();
                    follower.followPath(back1, true);
                    setPathState(PathState.SHOOT1);
                }
                break;

            case SHOOT1:
                if (!follower.isBusy() && shooterLogic.readyToFire()) {
                    shooterLogic.openGate();
                    shooterLogic.burstFire();
                }
                if (!shooterLogic.isBusy()) {
                    follower.followPath(to_intake2, true);
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

        buildPaths();
        follower.setPose(starterPose);

        pathState = PathState.DRIVE_STARTPOSE_SCOREPOSE;

    }

    @Override
    public void loop() {

        CommandScheduler.getInstance().run();
        follower.update();
        shooterLogic.update();
        statePathUpdate();

        PoseStorage.currentPose = follower.getPose();

        telemetry.addData("State", pathState);
    }
}
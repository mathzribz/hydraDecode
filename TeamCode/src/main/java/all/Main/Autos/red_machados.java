//
//package all.Main.Autos;
//
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import all.Configs.Auto.AutoLogic;
//import all.Configs.Auto.PoseStorage;
//import all.Configs.Pedro.Constants;
//
//@Autonomous
//public class red_machados extends OpMode {
//
//    private Follower follower;
//    private Timer pathTimer, opModeTimer;
//
//    private AutoLogic shooterLogic = new AutoLogic();
//
//    public enum PathState {
//
//        DRIVE_STARTPOSE_SCOREPOSE,
//        START_SCORE,
//
//        DRIVE_TO_INTAKE1,
//        COLLECT1,
//        SHOOT1,
//
//        DRIVE_TO_INTAKE2,
//        COLLECT2,
//        SHOOT2,
//
//        DRIVE_TO_GATE,
//
//        DRIVE_TO_INTAKE3,
//        COLLECT3,
//        SHOOT3,
//
//        END
//    }
//
//    PathState pathState;
//
//    // ===== POSES =====
//    private final Pose starterPose = new Pose(
//            126.463,
//            119.723,
//            Math.toRadians(38)
//    );
//
//    private final Pose scorePose = new Pose(
//            105.83,
//            102.26366559485528,
//            Math.toRadians(38)
//    );
//
//    private final Pose intake1 = new Pose(
//            127.624,
//            84.10289389067523,
//            Math.toRadians(0)
//    );
//
//    private final Pose intake2 = new Pose(
//            127.33333333333334,
//            59.66666666666667,
//            Math.toRadians(0)
//    );
//
//    private final Pose gate = new Pose(
//            127.78494623655915,
//            69.3763440860215,
//            Math.toRadians(270)
//    );
//
//    private final Pose intake3 = new Pose(
//            128.9247311827957,
//            36.47311827956988,
//            Math.toRadians(0)
//    );
//
//    // ===== PATHS =====
//    PathChain start_score;
//    PathChain to_intake1, back1;
//    PathChain to_intake2, back2;
//    PathChain to_gate;
//    PathChain to_intake3, back3;
//
//    public void buildPaths() {
//
//        start_score = follower.pathBuilder()
//                .addPath(new BezierLine(starterPose.mirror(), scorePose.mirror()))
//                .build();
//
//        to_intake1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose.mirror(), intake1.mirror()))
//                .build();
//
//        back1 = follower.pathBuilder()
//                .addPath(new BezierLine(intake1.mirror(), scorePose.mirror()))
//                .build();
//
//        to_intake2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose.mirror(), intake2.mirror()))
//                .build();
//
//        back2 = follower.pathBuilder()
//                .addPath(new BezierLine(intake2.mirror(), scorePose.mirror()))
//                .build();
//
//        to_gate = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose.mirror(), gate.mirror()))
//                .build();
//
//        to_intake3 = follower.pathBuilder()
//                .addPath(new BezierLine(gate.mirror(), intake3.mirror()))
//                .build();
//
//        back3 = follower.pathBuilder()
//                .addPath(new BezierLine(intake3.mirror(), scorePose.mirror()))
//                .build();
//    }
//
//    public void statePathUpdate() {
//
//        switch (pathState) {
//
//            case DRIVE_STARTPOSE_SCOREPOSE:
//
//                shooterLogic.preSpin();
//
//                if (opModeTimer.getElapsedTimeSeconds() > 0.2) {
//                    follower.followPath(start_score, true);
//                    setPathState(PathState.START_SCORE);
//                }
//                break;
//
//            case START_SCORE:
//
//                if (!follower.isBusy() && shooterLogic.readyToFire()) {
//                    shooterLogic.openGate();
//                    shooterLogic.burstFire();
//                }
//
//                if (!shooterLogic.isBusy()) {
//                    follower.followPath(to_intake1, true);
//                    setPathState(PathState.DRIVE_TO_INTAKE1);
//                }
//                break;
//
//            // ================= CICLO 1 =================
//
//            case DRIVE_TO_INTAKE1:
//                if (!follower.isBusy()) {
//                    shooterLogic.startIntakeWithSensors();
//                    setPathState(PathState.COLLECT1);
//                }
//                break;
//
//            case COLLECT1:
//                if (shooterLogic.intakeFull()) {
//                    shooterLogic.stopIntake();
//                    follower.followPath(back1, true);
//                    setPathState(PathState.SHOOT1);
//                }
//                break;
//
//            case SHOOT1:
//                if (!follower.isBusy() && shooterLogic.readyToFire()) {
//                    shooterLogic.openGate();
//                    shooterLogic.burstFire();
//                }
//                if (!shooterLogic.isBusy()) {
//                    follower.followPath(to_intake2, true);
//                    setPathState(PathState.DRIVE_TO_INTAKE2);
//                }
////                break;
////
////            // ================= CICLO 2 =================
////
////            case DRIVE_TO_INTAKE2:
////                if (!follower.isBusy()) {
////                    shooterLogic.startIntakeWithSensors();
////                    setPathState(PathState.COLLECT2);
////                }
////                break;
////
////            case COLLECT2:
////                if (shooterLogic.intakeFull()) {
////                    shooterLogic.stopIntake();
////                    follower.followPath(back2, true);
////                    setPathState(PathState.SHOOT2);
////                }
////                break;
////
////            case SHOOT2:
////                if (!follower.isBusy() && shooterLogic.readyToFire()) {
////                    shooterLogic.openGate();
////                    shooterLogic.burstFire();
////                }
////                if (!shooterLogic.isBusy()) {
////                    follower.followPath(to_gate, true);
////                    setPathState(PathState.DRIVE_TO_GATE);
////                }
////                break;
////
////            // ================= GATE =================
////
////            case DRIVE_TO_GATE:
////                if (!follower.isBusy()) {
////                    follower.followPath(to_intake3, true);
////                    setPathState(PathState.DRIVE_TO_INTAKE3);
////                }
////                break;
////
////            // ================= CICLO 3 =================
////
////            case DRIVE_TO_INTAKE3:
////                if (!follower.isBusy()) {
////                    shooterLogic.startIntakeWithSensors();
////                    setPathState(PathState.COLLECT3);
////                }
////                break;
////
////            case COLLECT3:
////                if (shooterLogic.intakeFull()) {
////                    shooterLogic.stopIntake();
////                    follower.followPath(back3, true);
////                    setPathState(PathState.SHOOT3);
////                }
////                break;
////
////            case SHOOT3:
////                if (!follower.isBusy() && shooterLogic.readyToFire()) {
////                    shooterLogic.openGate();
////                    shooterLogic.burstFire();
////                }
//                if (!shooterLogic.isBusy()) {
//                    setPathState(PathState.END);
//                }
//                break;
//
//            case END:
//                shooterLogic.stopAll();
//                break;
//        }
//    }
//
//    public void setPathState(PathState newState) {
//        pathState = newState;
//        pathTimer.resetTimer();
//    }
//
//    @Override
//    public void init() {
//
//        pathTimer = new Timer();
//        opModeTimer = new Timer();
//
//        follower = Constants.createFollower(hardwareMap);
//        shooterLogic.init(hardwareMap);
//
//        buildPaths();
//        follower.setPose(starterPose.mirror());
//
//        pathState = PathState.DRIVE_STARTPOSE_SCOREPOSE;
//
//    }
//
//    @Override
//    public void loop() {
//
//        CommandScheduler.getInstance().run();
//        follower.update();
//        shooterLogic.update();
//        statePathUpdate();
//
//        PoseStorage.currentPose = follower.getPose();
//
//        telemetry.addData("State", pathState);
//    }
//}

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
import all.Configs.Pedro.Constants;
import all.Configs.Auto.AutoLogic;
import all.subsystems.Turret;

@Autonomous
public class blue_12_3spikes extends OpMode {

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

        DRIVE_TO_gate,

        DRIVE_TO_INTAKE3,
        COLLECT3,
        SHOOT3,

        END,
        PARk
    }

    PathState pathState;

    // ===== PATHS =====
    public PathChain shoot1;
    public PathChain intake1;
    public PathChain gate;
    public PathChain shoot2;
    public PathChain intake2;
    public PathChain shoot3;
    public PathChain intake3;
    public PathChain shoot4;
    public PathChain leave;


    public void Paths() {
        shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(33.118, 135.312),

                                new Pose(51, 93)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(51, 93),
                                new Pose(71.017, 56.623),
                                new Pose(56.952, 59.498),
                                new Pose(16.531, 58.811)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();



        gate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(16.531, 58.811),
                                new Pose(43.699, 66.714),
                                new Pose(17.161, 69.054)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        shoot2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(17.161, 69.054),
                                new Pose(56.629, 68.55925806451614),
                                new Pose(51, 93)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(51, 93),
                                new Pose(88.206, 26.392),
                                new Pose(13.242, 35.418)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        shoot3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.242, 35.418),

                                new Pose(51, 93)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intake3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(51, 93),
                                new Pose(57.676, 80.508),
                                new Pose(15.695, 84.505)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        shoot4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.695, 84.505),

                                new Pose(51, 93)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        leave = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(51.000, 93.000),

                                new Pose(31.000, 93.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

    }

    public void statePathUpdate() {

        switch (pathState) {

            case DRIVE_STARTPOSE_SCOREPOSE:

                    autologic.preSpin();


                    follower.followPath(shoot1, 0.9, true);
                    setPathState(PathState.START_SCORE);


                break;

            case START_SCORE:

                if (!follower.isBusy() ){
                    autologic.burstFire();
                    pathTimer.resetTimer();
                    setPathState(PathState.DRIVE_TO_INTAKE1);

                }
                        break;

            // ================= CICLO 1 =================

            case DRIVE_TO_INTAKE1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.8) {
                    autologic.stopShooter();
                    follower.followPath(intake1, 0.9,true);
                    autologic.startIntake();
                    pathTimer.resetTimer();
                    setPathState(PathState.COLLECT1);
                }
                break;

            case COLLECT1:
                if (!follower.isBusy() &&  pathTimer.getElapsedTimeSeconds() > 0.2) {
                    autologic.stopIntake();
                    follower.followPath(gate,0.9, true);
                    pathTimer.resetTimer();
                    setPathState(PathState.DRIVE_TO_gate);

                }
                break;

            case DRIVE_TO_gate:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    autologic.preSpin();

                    follower.followPath(shoot2,0.9, true);
                    setPathState(PathState.SHOOT1);
                }

                break;

            case SHOOT1:
                if (!follower.isBusy() ){
                    autologic.burstFire();
                    pathTimer.resetTimer();
                    setPathState(PathState.DRIVE_TO_INTAKE2);

                }

                break;

            // ================= CICLO 2 =================

            case DRIVE_TO_INTAKE2:
                if (!follower.isBusy() &&  pathTimer.getElapsedTimeSeconds() > 1.8 ) {
                    autologic.stopShooter();
                    follower.followPath(intake2,0.9,true);
                    autologic.startIntake();
                    pathTimer.resetTimer();

                    setPathState(PathState.COLLECT2);
                }
                break;

            case COLLECT2:
                if (!follower.isBusy() &&  pathTimer.getElapsedTimeSeconds() > 0.2) {
                    autologic.stopIntake();


                    follower.followPath(shoot3,0.9, true);
                    setPathState(PathState.SHOOT2);

                        autologic.preSpin();

                }
                break;

            case SHOOT2:
                if (!follower.isBusy() ){
                    autologic.burstFire();
                    pathTimer.resetTimer();
                    setPathState(PathState.DRIVE_TO_INTAKE3);

                }
                break;


            case DRIVE_TO_INTAKE3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.8) {
                    autologic.stopShooter();
                    follower.followPath(intake3,0.9,true);
                    autologic.startIntake();
                    pathTimer.resetTimer();
                    setPathState(PathState.COLLECT3);
                }
                break;

            case COLLECT3:
                if (!follower.isBusy() &&  pathTimer.getElapsedTimeSeconds() > 0.2) {
                    autologic.stopIntake();

                    follower.followPath(shoot4,0.9, true);
                    autologic.preSpin();
                    pathTimer.resetTimer();
                    setPathState(PathState.SHOOT3);
                }
                break;

            case SHOOT3:
                if (!follower.isBusy() ){
                    autologic.burstFire();
                    setPathState(PathState.END);
                    pathTimer.resetTimer();

                }

                break;

            case END:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.8) {
                    follower.followPath(leave,0.9, true);
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
        follower.setStartingPose(new Pose(33.118279569892465, 135.31182795698925, Math.toRadians(180)));

        pathState = PathState.DRIVE_STARTPOSE_SCOREPOSE;


    }

    @Override
    public void loop() {


        CommandScheduler.getInstance().run();
        follower.update();
        turret.holdRobotRelative(Math.toRadians(-40), follower.getHeading());
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
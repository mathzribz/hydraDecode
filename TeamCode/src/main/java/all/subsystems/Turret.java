package all.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.pedropathing.geometry.Pose;

import all.Configs.Pedro.Constants;

public class Turret extends SubsystemBase {

    private final DcMotorEx motor;
    private final Follower follower;

    public static double TICKS_PER_REV = 537.7;
    public static double kp = 0.00065;
    public static double kd = 0.0001;

    private final PIDController pid;
    private double targetAngle = 0.0;

    public static double MAX_ANGLE = Math.toRadians(180);

    public static double gear_ratio = 4.0;

    public Turret(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "turret");
        follower = Constants.createFollower(hw);

        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setPower(0);

        pid = new PIDController(kp, 0.0, kd);
    }

    @Override
    public void periodic() {
        follower.update();

        pid.setPID(kp, 0, kd);

        double current = getCurrentAngle();
        double power = pid.calculate(current, targetAngle);

        motor.setPower(power);
    }

    public void seguirPose(Pose fieldTarget, Pose robot) {
        double robotX = robot.getPose().getX();
        double robotY = robot.getPose().getY();
        double robotHeading = robot.getPose().getHeading();

        double dx = fieldTarget.getX() - robotX;
        double dy = fieldTarget.getY() - robotY;

        double absoluteAngle = Math.atan2(dy, dx);

        double relativeAngle = absoluteAngle - robotHeading;
        relativeAngle = wrap(relativeAngle);

        setTarget(relativeAngle);
    }

    public void setTarget(double angle) {
        angle = wrap(angle);
        angle = clamp(angle);

        targetAngle = angle;
    }

    public double getCurrentAngle() {
        double ticks = motor.getCurrentPosition();
        return ticksToRads(ticks);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void resetEncoder() {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        pid.reset();
    }

    private double ticksToRads(double ticks) {
        return ticks / TICKS_PER_REV * (2 * Math.PI) * gear_ratio;
    }

    private double clamp(double angle) {
        return Math.max(-MAX_ANGLE, Math.min(MAX_ANGLE, angle));
    }

    private double wrap(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}

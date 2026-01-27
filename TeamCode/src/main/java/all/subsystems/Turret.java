
package all.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.geometry.Pose;

import all.Configs.Pedro.Constants;

public class Turret extends SubsystemBase {

    private final DcMotorEx motor;
    private final Follower follower;

    public static double TICKS_PER_REV = 537.7;
//    public static double kp = 0.05;
//    public static double kd = 0.0001;

    private final PIDController pid;
    private double targetAngle = 0.0;

    public static double MAX_ANGLE = Math.toRadians(180);

    public static double gear_ratio = 4.0;
    private PIDFController p, s; // pidf controller for turret
    private double t = 0;
    public static double pidfSwitch = 30; // target for turret
    public static double kp = 0.003, kf = 0.0, kd = 0.000, sp = 0.005, sf = 0, sd = 0.00001;

    private double error = 0;

    public Turret(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "turret");
        follower = Constants.createFollower(hw);

        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);

        pid = new PIDController(kp, 0.0, kd);

        p = new PIDFController(new PIDFCoefficients(kp, 0, kd, kf));
        s = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));

    }
    private void setTurretTarget(double ticks) {
        t = ticks;
    }

    public double getTurretTarget() {
        return t;
    }

    public double getTurret() {
        return motor.getCurrentPosition();
    }

    @Override
    public void periodic() {

        // converte target angular (rad) → ticks
        double targetTicks = radsToTicks(targetAngle);
        setTurretTarget(targetTicks);

        double currentTicks = getTurret();
        error = targetTicks - currentTicks;

        // atualiza coeficientes
        p.setCoefficients(new PIDFCoefficients(kp, 0, kd, kf));
        s.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));

        // zona morta para evitar jitter
        if (Math.abs(error) < 5) {
            motor.setPower(0);
            p.reset();
            s.reset();
            return;
        }

        // PID rápido ou lento
        double power = 0;
        if (Math.abs(error) > pidfSwitch) {
            p.updateError(error);
            p.updateFeedForwardInput(Math.signum(error));
            power = p.run();
        } else {
            s.updateError(error);
            power = s.run();
        }

        power = Math.max(-1.0, Math.min(1.0, power));
        motor.setPower(power);
    }
    public void seguirPose(Pose fieldTarget, Pose robot) {
        double robotX = robot.getX();
        double robotY = robot.getY();
        double robotHeading = robot.getHeading();

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


    private double radsToTicks(double rad) {
        return (rad / (2 * Math.PI)) * TICKS_PER_REV * gear_ratio;
    }
    public void resetEncoder() {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        pid.reset();
    }

    private double ticksToRads(double ticks) {
        return (ticks / TICKS_PER_REV / gear_ratio) * (2 * Math.PI) ;
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

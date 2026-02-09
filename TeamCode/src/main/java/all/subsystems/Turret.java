
package all.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret extends SubsystemBase {

    private final DcMotorEx motor;


    public static double TICKS_PER_REV = 537.7;
    public static double GEAR_RATIO = 3.906976744186047;
    public static double MAX_ANGLE = Math.toRadians(150);


    public static double kp = 0.0025, kd = 0.0, kf = 0.0;
    public static double sp = 0.005,  sd = 0.00001, sf = 0.0;

    public static double pidfSwitch = 30;

    private final PIDFController pFast;
    private final PIDFController pSlow;


    private double targetAngleRad = 0.0;

    private double encoderOffsetRad = 0.0;


    public Turret(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "turret");

        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);

        pFast = new PIDFController(new PIDFCoefficients(kp, 0, kd, kf));
        pSlow = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
    }


    @Override
    public void periodic() {

        double currentAngle = getCurrentAngleRad();
        double errorRad = targetAngleRad - currentAngle;
        double errorTicks = radsToTicks(errorRad);

        if (Math.abs(errorTicks) < 5) {
            motor.setPower(0);
            pFast.reset();
            pSlow.reset();
            return;
        }

        double power;

        if (Math.abs(errorTicks) > pidfSwitch) {
            pFast.updateError(errorTicks);
            pFast.updateFeedForwardInput(Math.signum(errorTicks));
            power = pFast.run();
        } else {
            pSlow.updateError(errorTicks);
            power = pSlow.run();
        }

        motor.setPower(clamp(power, -1.0, 1.0));
    }


    public void followPose(Pose fieldTarget, Pose robotPose) {

        double dx = fieldTarget.getX() - robotPose.getX();
        double dy = fieldTarget.getY() - robotPose.getY();

        // ângulo global do alvo
        double absoluteAngle = Math.atan2(dy, dx);

        // converte para referencial do robô
        double relativeAngle = wrap(absoluteAngle - robotPose.getHeading());

        setTargetAngle(relativeAngle);
    }


    public void setTargetAngle(double angleRad) {

        angleRad = clampAngle(angleRad);
        targetAngleRad = angleRad;
    }

    public double getTargetAngleRad() {
        return targetAngleRad;
    }


    public double getCurrentAngleRad() {
        return ticksToRads(motor.getCurrentPosition()) + encoderOffsetRad;
    }

    public void calibrateZero() {
        encoderOffsetRad = -ticksToRads(motor.getCurrentPosition());
        pFast.reset();
        pSlow.reset();
    }

    public void resetEncoder() {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        encoderOffsetRad = 0.0;
        pFast.reset();
        pSlow.reset();
    }


    private double radsToTicks(double rad) {
        return (rad / (2 * Math.PI)) * TICKS_PER_REV * GEAR_RATIO;
    }

    private double ticksToRads(double ticks) {
        return (ticks / TICKS_PER_REV / GEAR_RATIO) * (2 * Math.PI);
    }


    private double wrap(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double clampAngle(double angle) {
        return Math.max(-MAX_ANGLE, Math.min(MAX_ANGLE, angle));
    }
}

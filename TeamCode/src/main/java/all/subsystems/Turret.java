
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
    public static double GEAR_RATIO = 4.0;
    public static double MAX_ANGLE = Math.toRadians(180);

    public static double kp = 0.003, kd = 0.000, kf = 0.0;

    private final PIDFController pid;

    // alvo ABSOLUTO no campo
    private double targetFieldAngle = 0.0;

    public Turret(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "turret");

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pid = new PIDFController(new PIDFCoefficients(kp, 0, kd, kf));
    }

    @Override
    public void periodic() {
        pid.setCoefficients(new PIDFCoefficients(kp, 0, kd, kf));
    }

    public void followPose(Pose fieldTarget, Pose robotPose, Double head) {

        double dx = fieldTarget.getX() - robotPose.getX();
        double dy = fieldTarget.getY() - robotPose.getY();

        // ângulo ABSOLUTO do alvo no campo
        targetFieldAngle = Math.atan2(dy, dx) + Math.PI;
        targetFieldAngle = wrap(targetFieldAngle);

        updateControl(head);
    }

    private void updateControl(double robotHeading) {

        // ângulo atual da turret NO CAMPO
        double turretRelative = ticksToRads(motor.getCurrentPosition());
        double turretFieldAngle = wrap(robotHeading + turretRelative);

        double error = wrap(targetFieldAngle - turretFieldAngle);

        // respeita limite mecânico
        double clampedRelative =
                clamp(wrap(targetFieldAngle - robotHeading));

        double targetTicks = radsToTicks(clampedRelative);
        double currentTicks = motor.getCurrentPosition();

        double tickError = targetTicks - currentTicks;

        if (Math.abs(tickError) < 5) {
            motor.setPower(0);
            pid.reset();
            return;
        }

        pid.updateError(tickError);
        pid.updateFeedForwardInput(Math.signum(tickError));

        double power = pid.run();
        motor.setPower(Math.max(-1, Math.min(1, power)));
    }

    public void setTargetFieldAngle(double angle) {
        targetFieldAngle = wrap(angle);
    }

    private double radsToTicks(double rad) {
        return (rad / (2 * Math.PI)) * TICKS_PER_REV * GEAR_RATIO;
    }

    private double ticksToRads(double ticks) {
        return (ticks / (TICKS_PER_REV * GEAR_RATIO)) * (2 * Math.PI);
    }

    private double clamp(double angle) {
        return Math.max(-MAX_ANGLE, Math.min(MAX_ANGLE, angle));
    }

    private double wrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}

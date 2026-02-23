
package all.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret extends SubsystemBase {

    private final DcMotorEx motor;

    public static double TICKS_PER_REV = 537.7;
    public static double GEAR_RATIO = 3.906976744186047;
    public static double MAX_ANGLE = Math.toRadians(90);

    public static double kp = 0.0025, kd = 0.000, kf = 0.0;

    private final PIDFController pid;

    private double targetFieldAngle = 0.0;

    private double relocalizationAngleOffset = 0.0;

    private boolean holdMode = false;
    private double holdAngle = 0.0;
    public Turret(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "turret");

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pid = new PIDFController(new PIDFCoefficients(kp, 0, kd, kf));
    }

    @Override
    public void periodic() {
        pid.setCoefficients(new PIDFCoefficients(kp, 0, kd, kf));


    }

    public void setRelocalizationOffset(double offset) {
        relocalizationAngleOffset = offset;
    }

    public void followPose(Pose fieldTarget, Pose robotPose, Double head) {

        double dx = fieldTarget.getX() - robotPose.getX();
        double dy = fieldTarget.getY() - robotPose.getY();

        // ângulo ABSOLUTO do alvo no campo
        targetFieldAngle = Math.atan2(dy, dx) ;
        targetFieldAngle = wrap(targetFieldAngle + relocalizationAngleOffset);

        updateControl(head);
    }

    private void updateControl(double robotHeading) {

        double turretRelative = ticksToRads(motor.getCurrentPosition());
        double turretFieldAngle = wrap(robotHeading + turretRelative + relocalizationAngleOffset);

        double error = wrap(targetFieldAngle - turretFieldAngle);

        double clampedRelative = clamp(wrap(targetFieldAngle - robotHeading));

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
        motor.setPower(Math.max(-0.75, Math.min(0.75, power)));
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

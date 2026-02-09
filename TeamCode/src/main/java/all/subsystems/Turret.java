
package all.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret extends SubsystemBase {


    private final DcMotorEx motor;
    private final Limelight3A limelight;


    private enum TurretMode {
        ODOMETRY,
        LIMELIGHT
    }

    private TurretMode mode = TurretMode.ODOMETRY;

    public static double TICKS_PER_REV = 537.7;
    public static double gear_ratio = 3.906976744186047;
    public static double MAX_ANGLE = Math.toRadians(150);

    public static double kp = 0.0025, kd = 0.000, kf = 0.0;
    public static double sp = 0.005, sd = 0.00001, sf = 0.0;
    public static double pidfSwitch = 30;

    private final PIDFController pFast;
    private final PIDFController pSlow;

    public static double LL_kP = 0.075;
    public static double LL_kD = 0.004;

    private double targetTx = 0;

    private double targetAngle = 0;
    private double error = 0;

    private double encoderOffsetTicks = 0;

    public Turret(HardwareMap hw) {

        motor = hw.get(DcMotorEx.class, "turret");
        limelight = hw.get(Limelight3A.class, "limelight");

        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        limelight.pipelineSwitch(0);
        limelight.start();

        pFast = new PIDFController(new PIDFCoefficients(kp, 0, kd, kf));
        pSlow = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
    }

    @Override
    public void periodic() {

        if (mode == TurretMode.LIMELIGHT) {
            updateLimelight();
        } else {
            updateOdom();
        }
    }


    private void updateOdom() {

        double targetTicks = radsToTicks(targetAngle);
        double currentTicks = motor.getCurrentPosition();

        error = targetTicks - currentTicks;

        if (Math.abs(error) < 5) {
            motor.setPower(0);
            pFast.reset();
            pSlow.reset();
            return;
        }

        double power;
        if (Math.abs(error) > pidfSwitch) {
            pFast.updateError(error);
            pFast.updateFeedForwardInput(Math.signum(error));
            power = pFast.run();
        } else {
            pSlow.updateError(error);
            power = pSlow.run();
        }

        motor.setPower(clamp(power, -1, 1));
    }

    private void updateLimelight() {

        LLResult res = limelight.getLatestResult();

        if (res == null || !res.isValid()) {
            motor.setPower(0);
            return;
        }

        double tx = res.getTx();
        double error = tx - targetTx;

        double power = (LL_kP * error) + (LL_kD * (error - this.error));
        this.error = error;

        motor.setPower(clamp(power, -0.5, 0.5));
    }


    public void followPose(Pose fieldTarget, Pose robot, Vector velocity) {

        if (mode != TurretMode.ODOMETRY) return;

        double dx = fieldTarget.getX() - robot.getX();
        double dy = fieldTarget.getY() - robot.getY();

        double absoluteAngle = Math.atan2(dy, dx);
        double relativeAngle = wrap(absoluteAngle - robot.getHeading());

        double distance = Math.hypot(dx, dy);

        double compensation = 0;

        if (distance > 1e-3) {

            double vLateral = velocity.getYComponent();

            compensation = vLateral / distance;
        }

        double compensatedAngle = relativeAngle + compensation;

        setTarget(compensatedAngle);
    }

    public void setTarget(double angle) {
        angle = wrap(angle);
        angle = clampAngle(angle);
        targetAngle = angle;
    }


    public void enableLimelight(double targetTx) {
        this.targetTx = targetTx;
        error = 0;
        pFast.reset();
        pSlow.reset();
        mode = TurretMode.LIMELIGHT;
    }

    public void disableLimelight() {

        targetAngle = getCurrentAngle();

        pFast.reset();
        pSlow.reset();
        mode = TurretMode.ODOMETRY;
        error = 0;

    }

    public double getCurrentAngle() {
        return ticksToRads(motor.getCurrentPosition());
    }


    private double radsToTicks(double rad) {
        return (rad / (2 * Math.PI)) * TICKS_PER_REV * gear_ratio;
    }

    private double ticksToRads(double ticks) {
        return (ticks / TICKS_PER_REV / gear_ratio) * (2 * Math.PI);
    }

    private double wrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // clamp genérico
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // clamp físico da turret
    private double clampAngle(double angle) {
        return Math.max(-MAX_ANGLE, Math.min(MAX_ANGLE, angle));
    }

    public void resetEncoder() {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        pFast.reset();
        pSlow.reset();
    }

    public void setInitialAngle(double angleRad) {
        encoderOffsetTicks = radsToTicks(angleRad);
    }


}

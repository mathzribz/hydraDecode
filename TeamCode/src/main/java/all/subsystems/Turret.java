package all.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret extends SubsystemBase {

    private final DcMotorEx motor;

    public static double TICKS_PER_REV = 537.7;
    public static double GEAR_RATIO = 3.906976744186047;
    public static double MAX_ANGLE = Math.toRadians(170);

    public static double kp = 0.003, kd = 0.0001, kf = 0.0;

    private final PIDFController pid;

    private double relocalizationAngleOffset = 0.0;
    public static double tickError = 0;

    private double targetRad = 0.0;

    public Turret(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "turret");

        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);

        pid = new PIDFController(new PIDFCoefficients(kp, 0, kd, kf));
    }

    @Override
    public void periodic() {
        pid.setCoefficients(new PIDFCoefficients(kp, 0, kd, kf));
    }

    public void followPose(Pose fieldTarget, Pose robotPose, double robotHeading) {
        double dx = fieldTarget.getX() - robotPose.getX();
        double dy = fieldTarget.getY() - robotPose.getY();
        targetRad = Math.atan2(dy, dx);
        updateTurret(robotHeading);
    }

    private void updateTurret(double robotHeading) {

        double desiredRelative = wrap(targetRad - robotHeading + relocalizationAngleOffset);
        desiredRelative = clamp(desiredRelative);

        double currentTicks = motor.getCurrentPosition();
        double targetTicks = radsToTicks(desiredRelative);

        tickError = targetTicks - currentTicks;

        if (Math.abs(tickError) < 4) {
            motor.setPower(0);
            pid.reset();
            return;
        }

        pid.updateError(tickError);
        pid.updateFeedForwardInput(0);

        double power = pid.run();
        motor.setPower(Math.max(-1, Math.min(1, power)));
    }

    // =====================================================
    // 🔥 MÉTODO NOVO — SINCRONIZA TARGET COM ÂNGULO ATUAL
    // =====================================================

    public void syncTargetToCurrent(double robotHeading) {

        double currentRad = ticksToRads(motor.getCurrentPosition());

        // converte posição atual da turret para ângulo absoluto de campo
        targetRad = wrap(currentRad + robotHeading - relocalizationAngleOffset);

        pid.reset(); // evita impulso acumulado
    }

    // =====================================================

    private double radsToTicks(double rad) {
        return (rad / (2 * Math.PI)) * TICKS_PER_REV * GEAR_RATIO;
    }

    public double ticksToRads(double ticks) {
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

    public void applyVisionCorrection(double txDegrees, double offset) {

        if (Math.abs(txDegrees) < 0.5){
            relocalizationAngleOffset = 0;
            return;
        }

        double correctedTx = txDegrees - offset;
        double correctionRad = Math.toRadians(correctedTx);

        relocalizationAngleOffset = -correctionRad;
        relocalizationAngleOffset = wrap(relocalizationAngleOffset);
    }

    public double getTickError(){
        return tickError;
    }

    public void resetEncoder(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void holdRobotRelative(double relativeAngleRad, double robotHeading) {

        targetRad = wrap(relativeAngleRad + robotHeading);

        updateTurret(robotHeading);
    }
}
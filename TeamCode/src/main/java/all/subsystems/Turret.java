
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
    public static double MAX_ANGLE = Math.toRadians(160);

    public static double kp = 0.003, kd = 0.0001, kf = 0.0;

    private final PIDFController pid;

    private double relocalizationAngleOffset = 0.0;


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

    public void setRelocalizationOffset(double offset) {
        relocalizationAngleOffset = offset;
    }

    public void followPose(Pose fieldTarget, Pose robotPose, double robotHeading) {
        double dx = fieldTarget.getX() - robotPose.getX();
        double dy = fieldTarget.getY() - robotPose.getY();
        targetRad = Math.atan2(dy, dx);
        updateTurret(robotHeading);
    }

    private void updateTurret(double robotHeading) {
        // Turret atual em radianos
        double currentRad = ticksToRads(motor.getCurrentPosition());

        // Ângulo em campo da turret
        double turretField = wrap(robotHeading + currentRad);

        // Erro alvo
        double error = wrap(targetRad - turretField);

        // Lógica de Ângulo desejado relativo ao robô
        double offsetRad = 0.0;
        double desiredRelative = wrap(error + offsetRad);
        desiredRelative = clamp(desiredRelative);

        // Converte para radianos alvo
        double targetTicks = radsToTicks(desiredRelative);
        double currentTicks = motor.getCurrentPosition();

        double tickError = targetTicks - currentTicks;

        if (Math.abs(tickError) < 4) {
            motor.setPower(0);
            pid.reset();
            return;
        }

        pid.updateError(tickError);
        pid.updateFeedForwardInput(0); // sem feedforward por agora

        double power = pid.run();
        motor.setPower(Math.max(-0.7, Math.min(0.7, power)));
    }

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

        relocalizationAngleOffset =- correctionRad;

        relocalizationAngleOffset = wrap(relocalizationAngleOffset) ;
    }

}


package all.subsystems;

import android.os.ParcelUuid;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Shooter extends SubsystemBase {

    private final DcMotorEx shooterL, shooterR;
    private final Servo hood;

    public static double kP = 0.015    ;
    public static double kD = 0.0000;
    public static double kF = 0.000215;

    private final PIDFController pidf = new PIDFController(kP, 0, kD, 0);


    public static double TICKS_PER_REV = 28;
    public static double targetRPM = 2200;

    private boolean enabled = false;
    private double power = 0;

    public Shooter(HardwareMap hwMap) {

        shooterL = hwMap.get(DcMotorEx.class, "shooterL");
        shooterR = hwMap.get(DcMotorEx.class, "shooterR");
        hood = hwMap.get(Servo.class, "capuz");

        shooterL.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void HoodHigh() {
        hood.setPosition(0.85);
    }

    public void HoodLow() {
        hood.setPosition(0.68);
    }
    public double HoodPos(double pos) {
        hood.setPosition(pos);
        return pos;
    }

    public void shooterOn() {
        enabled = true;
    }

    public void shooterOnAuto() {
        enabled = true;
    }


    public void shooterOff() {
        enabled = false;
        power = 0;
        pidf.reset();
        shooterL.setPower(0);
        shooterR.setPower(0);
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }
    public double getTargetRPM() {
        return targetRPM ;
    }

    public double getCurrentRPM() {
        return (shooterL.getVelocity() / TICKS_PER_REV) * 60.0;
    }

    public static double flywheelSpeed (double goalDistance) {
        return MathFunctions.clamp(0.0000146828 * Math.pow(goalDistance, 4) -
                0.00561643 * Math.pow(goalDistance, 3) +
                0.73465 * Math.pow(goalDistance, 2) -
                27.49451 * goalDistance +
                2149.83295, 800, 3200 );
    }

    public static double hoodAngle (double goalDistance) {
        return MathFunctions.clamp((5.93422 * Math.pow(10, -7)) * Math.pow(goalDistance, 3) -
                0.000221425 * Math.pow(goalDistance, 2) +
                0.0282625 * goalDistance -
                0.428675, 0.2, 0.85);
    }


    public void putHood ( double dd){
        hood.setPosition(hoodAngle(dd));
    }
    public void putRpm ( double dd){
        setTargetRPM(flywheelSpeed(dd));
    }

    @Override
    public void periodic() {


        pidf.setPIDF(kP, 0, kD, kF);

        double currentTPS = shooterL.getVelocity();
        double targetTPS = (targetRPM / 60.0) * TICKS_PER_REV;

        if (enabled) {
            double ff = targetTPS * kF;
            double pid = pidf.calculate(currentTPS, targetTPS);

            power = ff + pid;
            power = Math.max(0.0, Math.min(1.0, power));

            shooterL.setPower(power);
            shooterR.setPower(power);
        } else {
            shooterL.setPower(0);
            shooterR.setPower(0);
            pidf.reset();
        }







    }




}

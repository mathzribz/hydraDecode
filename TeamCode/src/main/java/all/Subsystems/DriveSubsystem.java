package all.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem extends SubsystemBase {

    private final DcMotorEx LMF, LMB, RMF, RMB;
    private final GoBildaPinpointDriver pinpoint;
    private double headingOffset = 0.0;
    private double driveSpeed = 0.85;

    public DriveSubsystem(HardwareMap hwMap) {

        LMF = hwMap.get(DcMotorEx.class, "LMF");
        LMB = hwMap.get(DcMotorEx.class, "LMB");
        RMF = hwMap.get(DcMotorEx.class, "RMF");
        RMB = hwMap.get(DcMotorEx.class, "RMB");

        LMF.setDirection(DcMotorEx.Direction.REVERSE);
        LMB.setDirection(DcMotorEx.Direction.REVERSE);
        RMF.setDirection(DcMotorEx.Direction.FORWARD);
        RMB.setDirection(DcMotorEx.Direction.FORWARD);

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }


    public void resetFieldOrientation() {
        headingOffset = getRawHeading();
    }

    public void adjustFieldOrientation(double deltaRadians) {
        headingOffset += deltaRadians;
    }

    private double getRawHeading() {
        pinpoint.update();
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    // =================== DRIVE ===================

    /**
     * @param x  strafe (-1 a 1)
     * @param y  frente/trás (-1 a 1)
     * @param rx rotação (-1 a 1)
     */

    public void drive(double x, double y, double rx) {

        double botHeading = getRawHeading() - headingOffset;

        // Field Centric transform
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(
                Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx),
                1.0
        );

        double powerLMF = (rotY + rotX + rx) / denominator;
        double powerLMB = (rotY - rotX + rx) / denominator;
        double powerRMF = (rotY - rotX - rx) / denominator;
        double powerRMB = (rotY + rotX - rx) / denominator;

        LMF.setPower(powerLMF * driveSpeed);
        LMB.setPower(powerLMB * driveSpeed);
        RMF.setPower(powerRMF * driveSpeed);
        RMB.setPower(powerRMB * driveSpeed);
    }


    public void stop() {
        LMF.setPower(0);
        LMB.setPower(0);
        RMF.setPower(0);
        RMB.setPower(0);
    }

    public void setDriveSpeed(double speed) {
        driveSpeed = Math.max(0.1, Math.min(1.0, speed));
    }

    public double getDriveSpeed() {
        return driveSpeed;
    }

    @Override
    public void periodic() {
        telemetry.addData("pionpoint YAW ",pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Drive Speed", driveSpeed);
    }
}

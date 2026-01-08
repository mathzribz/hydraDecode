package all.subsystems;

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
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");

    }


    public void resetFieldOrientation() {
        headingOffset = getRawHeading();
    }

    public void adjustFieldOrientation(double deltaRadians) {
        headingOffset += deltaRadians;
    }

    private double getRawHeading() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    public void drive(double x, double y, double rx) {

        double botHeading = getRawHeading() - headingOffset;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(
                Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx),
                1
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

    public void setDriveSpeed(double speed) {
        driveSpeed = speed;
    }

    public double getDriveSpeed() {
        return driveSpeed;
    }




}

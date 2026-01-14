package all.subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive extends SubsystemBase {

    private final DcMotorEx LMF, LMB, RMF, RMB;
    private final GoBildaPinpointDriver pinpoint;

    private double headingOffset = 0.0;
    private double driveSpeed = 0.85;

    private double currentHeadingRad = 0.0;
    private double currentHeadingDeg = 0.0;

    public Drive(HardwareMap hwMap) {
        LMF = hwMap.get(DcMotorEx.class, "LMF");
        LMB = hwMap.get(DcMotorEx.class, "LMB");
        RMF = hwMap.get(DcMotorEx.class, "RMF");
        RMB = hwMap.get(DcMotorEx.class, "RMB");

        LMF.setDirection(DcMotorEx.Direction.REVERSE);
        LMB.setDirection(DcMotorEx.Direction.REVERSE);

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    /** CHAMAR UMA VEZ POR LOOP */
    public void updatePinpoint() {
        pinpoint.update();
        currentHeadingRad = pinpoint.getHeading(AngleUnit.RADIANS);
        currentHeadingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public void resetFieldOrientation() {
        headingOffset = currentHeadingRad;
    }

    public void drive(double x, double y, double rx) {

        double botHeading = currentHeadingRad - headingOffset;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(
                Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0
        );

        LMF.setPower((rotY + rotX + rx) / denominator * driveSpeed);
        LMB.setPower((rotY - rotX + rx) / denominator * driveSpeed);
        RMF.setPower((rotY - rotX - rx) / denominator * driveSpeed);
        RMB.setPower((rotY + rotX - rx) / denominator * driveSpeed);
    }

    public double getHeadingDeg() {
        return currentHeadingDeg;
    }

    public double getHeadingRad() {
        return currentHeadingRad;
    }

    public double getDriveSpeed() {
        return driveSpeed;
    }

    public void setDriveSpeed(double speed) {
        driveSpeed = Math.max(0.1, Math.min(1.0, speed));
    }
}

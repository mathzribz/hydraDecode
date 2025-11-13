package all.configPedro;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()

            .mass(12.55)
            .forwardZeroPowerAcceleration(-47)
            .lateralZeroPowerAcceleration(-75)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.04, 0, 0.001, 0.01))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0.02))

            .headingPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2,0,0.08,0.03))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015,0,0.001,0.6,0.01))

            .centripetalScaling(0.005)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()

            .maxPower(0.8)
            .rightFrontMotorName("RMF")
            .rightRearMotorName("RMB")
            .leftRearMotorName("LMB")
            .leftFrontMotorName("LMF")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(46.97)
            .yVelocity(37.5)

            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()

            .forwardPodY(5.98)
            .strafePodX(4.055)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.85, 0.8);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)

                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
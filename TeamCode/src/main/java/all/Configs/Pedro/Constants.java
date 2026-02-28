package all.Configs.Pedro;

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

            .mass(13.55)
            .forwardZeroPowerAcceleration(72.08701498677412)
            .lateralZeroPowerAcceleration(43.21313716858391)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.055, 0, 0.0001, 0.05))

            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0.0001, 0.02))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.008,0,0.0001,0.6,0.005))

            .centripetalScaling(0.0007)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()

            .maxPower(0.95)
            .rightFrontMotorName("RMF")
            .rightRearMotorName("RMB")
            .leftRearMotorName("LMB")
            .leftFrontMotorName("LMF")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(70.99888502706693)
            .yVelocity(52.08411870402925)

            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()

            .forwardPodY(-72.5/25.4)
            .strafePodX(-87/25.4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static PathConstraints pathConstraints = new PathConstraints(
            0.975,
            100,
            0.8,
            0.75);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)

                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
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

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-39.25)
            .lateralZeroPowerAcceleration(-79.51)
            .mass(9.07)
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.056, 0.0001, 0.005, 0))
//            .headingPIDFCoefficients(new PIDFCoefficients(.8,0.01,0.001,0.01))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.004,0.000004,0.00008,0.6,0.0))

            .translationalPIDFCoefficients(new PIDFCoefficients(0.056, 0.000, 0.005, 0.0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.92,0.0,0.001,0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.018,0.00000,0.00008,0.6,0.0)) // p = 0.03
            .centripetalScaling(0.0005);


    public static MecanumConstants driveConstants = new MecanumConstants()
            // Check issue with either wheels or wiring of motors because drive code is not working atm
            .maxPower(1)
            .xVelocity(64.4630)
            .yVelocity(52.4704)

            .rightFrontMotorName("rf")
            .rightRearMotorName("rb")
            .leftRearMotorName("lb")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.125) // 3.125   5.500
            .strafePodX(-0.375) // 0.375  -3.700
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            //If running into issue with pods later keep in mind the x is y and y is x, no clue why but otherwise values wouldnt work
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
}

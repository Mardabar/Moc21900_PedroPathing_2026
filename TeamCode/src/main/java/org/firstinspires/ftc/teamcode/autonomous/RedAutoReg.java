package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotPoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "RedAutoReg", group = "autonomous")
public class RedAutoReg extends OpMode{

    // PEDROPATHING VARS

    private Follower fol;

    private Timer pathTimer, actionTimer, opmodeTimer; // Game timer
    private int pathState; // Current path #
    private int chainNum;

    // CAMERA VARS

    private Limelight3A cam;
    private VisionPortal visPort;
    private AprilTagProcessor apTag;
    private AprilTagDetection foundTag;
    private boolean tagFound;

    private final int GPP_ID = 21;
    private final int PGP_ID = 22;
    private final int PPG_ID = 23;

    // POSITIONS

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // STARTING POSITION
    private final Pose parkPose = new Pose(20, 20, Math.toRadians(45)); // PARKING POSITION

        // Obelisk #21 --------------------------------------------------
    private final Pose Ob21Grab1GP1 = new Pose(31, 35.5, Math.toRadians(0)); // POSITION
    private final Pose Ob21Grab1GP1CP = new Pose(56, 35.5, Math.toRadians(0)); // CONTROL POINT
    private final Pose Ob21Grab2P1 = new Pose(36, 59.8, Math.toRadians(0)); // POSITION
    private final Pose Ob21Grab2P1CP = new Pose(61, 59.8, Math.toRadians(0)); // CONTROL POINT
    private final Pose Ob21Score1 = new Pose(61, 76.5, Math.toRadians(42)); // POSITION
    private final Pose Ob21Grab1G2 = new Pose(31, 59.8, Math.toRadians(0)); // POSITION
    private final Pose Ob21Grab1G2CP = new Pose(59, 59.8, Math.toRadians(0)); // CONTROL POINT
    private final Pose Ob21Grab2PP2 = new Pose(31, 84, Math.toRadians(0)); // POSITION
    private final Pose Ob21Grab2PP2CP = new Pose(64, 84, Math.toRadians(0)); // CONTROL POINT
    // Ob21Score2 is the same as Ob21Score1
    private final Pose Ob21Grab3 = new Pose(19, 98, Math.toRadians(90)); // POSITION
    private final Pose Ob21Grab3CP = new Pose(60, 98, Math.toRadians(0)); // CONTROL POINT
    private final Pose Ob21GrabGPP3 = new Pose(19, 42, Math.toRadians(90)); // POSITION
    private final Pose Ob21Score3 = new Pose(61, 18, Math.toRadians(56)); // POSITION

        // Obelisk #22 --------------------------------------------------


        // Obelisk #23 --------------------------------------------------


    // PID CONSTANTS & OTHER MOTORS

    private DcMotor ls;
    private DcMotor rs;
    private DcMotor intakeBelt;
    private DcMotor elbow;
    private PIDFController lsPID;
    private PIDFController rsPID;
    private PIDFController ibPID;
    private PIDFController elPID;
    private static double p = 0, i = 0, d = 0, f = 0;
    private static PIDFCoefficients coef;

    // Encoder resolution for elbow motor is 751.8 PPR

    private static double ANGLE_CONST = 2.08833333;
    private static double lsTarget;
    private static double rsTarget;
    private static double ibTarget;
    private static double elTarget;
    private double lsIntSum;
    private double rsIntSum;
    private double ibIntSum;
    private double elIntSum;
    private double lsLastErr;
    private double rsLastErr;
    private double ibLastErr;
    private double elLastErr;

    // PATH CHAINS

        // Obelisk #21
    private PathChain pathOb21Grab1GP1, pathOb21Grab2P1, pathOb21Score1, pathOb21Grab1G2, pathOb21Grab2PP2,
                pathOb21Score2, pathOb21Grab3, pathOb21GrabGPP3, pathOb21Score3, pathOb21Park;
        // Obelisk #22
    private PathChain pathOb22Grab1G1;
        // Obelisk #23
    private PathChain pathOb23Grab1PP1;

    // OTHER VARS

    private ElapsedTime timer;
    private double dur;
    private int timerCount = -1;

    @Override
    public void init(){
        ls = hardwareMap.get(DcMotor.class, "ls");
        rs = hardwareMap.get(DcMotor.class, "rs");
        intakeBelt = hardwareMap.get(DcMotor.class, "intakeBelt");
        elbow = hardwareMap.get(DcMotor.class, "elbow");

        // PATH INIT

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        setPathState(0);

        // CAMERA INIT

        apTag = new AprilTagProcessor.Builder()
                .setCameraPose(new Position(DistanceUnit.INCH, -7, -7, 14, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, 14, 0, 0))
                .build();
        apTag.setDecimation(2);

        visPort = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Cam"))
                .addProcessor(apTag)
                .build();

        // PID INIT

        coef = new PIDFCoefficients(p, i, d, f);
        lsPID = new PIDFController(coef);
        rsPID = new PIDFController(coef);
        ibPID = new PIDFController(coef);
        elPID = new PIDFController(coef);
    }

    public void loop(){
        fol.update();
        autonomousPathUpdate();

        lsPID.setCoefficients(coef);
        rsPID.setCoefficients(coef);
        ibPID.setCoefficients(coef);
        elPID.setCoefficients(coef);

        double lsErr = lsTarget - ls.getCurrentPosition();
        double rsErr = rsTarget - rs.getCurrentPosition();
        double ibErr = ibTarget - intakeBelt.getCurrentPosition();
        double elErr = elTarget - elbow.getCurrentPosition();

        double lsD = (lsErr - lsLastErr) / opmodeTimer.getElapsedTimeSeconds();
        double rsD = (rsErr - rsLastErr) / opmodeTimer.getElapsedTimeSeconds();
        double ibD = (ibErr - ibLastErr) / opmodeTimer.getElapsedTimeSeconds();
        double elD = (elErr - elLastErr) / opmodeTimer.getElapsedTimeSeconds();

        lsIntSum = lsIntSum + (lsErr * opmodeTimer.getElapsedTimeSeconds());
        rsIntSum = rsIntSum + (rsErr * opmodeTimer.getElapsedTimeSeconds());
        ibIntSum = ibIntSum + (ibErr * opmodeTimer.getElapsedTimeSeconds());
        elIntSum = elIntSum + (elErr * opmodeTimer.getElapsedTimeSeconds());

        ls.setPower((p * lsErr) + (i * lsIntSum) + (d * lsD));
        rs.setPower((p * rsErr) + (i * rsIntSum) + (d * rsD));
        intakeBelt.setPower((p * ibErr) + (i * ibIntSum) + (d * ibD));
        elbow.setPower((p * elErr) + (i * elIntSum) + (d * elD));

        lsLastErr = lsErr;
        rsLastErr = rsErr;
        ibLastErr = ibErr;
        elLastErr = elErr;

        // This stores the ending position of the bot at the end of auto
        Pose finalPose = fol.getPose();
        RobotPoseStorage.currentPose = finalPose;

        // Not sure if this is in the right spot :skull:
        // Its either inside the loop or outside but outside prolly wouldnt make sense
        updatePos();
    }

    public void buildPaths(int obNum){
        setChainNum(obNum);
        if (obNum == GPP_ID){
            pathOb21Grab1GP1 = fol.pathBuilder()
                    .addPath(new BezierCurve(startPose, Ob21Grab1GP1CP, Ob21Grab1GP1))
                    .setLinearHeadingInterpolation(startPose.getHeading(), Ob21Grab1GP1.getHeading())
                    .setBrakingStrength(4)
                    .build();

            pathOb21Grab2P1 = fol.pathBuilder()
                    .addPath(new BezierCurve(Ob21Grab1GP1, Ob21Grab2P1CP, Ob21Grab2P1))
                    .setLinearHeadingInterpolation(Ob21Grab1GP1.getHeading(), Ob21Grab2P1.getHeading())
                    .build();

            pathOb21Score1 = fol.pathBuilder()
                    .addPath(new BezierLine(Ob21Grab2P1, Ob21Score1))
                    .setLinearHeadingInterpolation(Ob21Grab2P1.getHeading(), Ob21Score1.getHeading())
                    .build();
        }
        else if (obNum == PGP_ID){

        }
        else if (obNum == PPG_ID){

        }
    }

    public void autonomousPathUpdate(){
        if (!tagFound){
            List<AprilTagDetection> detections = apTag.getDetections();

            for (AprilTagDetection tag : detections){
                if (tag.metadata != null && (tag.id == GPP_ID || tag.id == PGP_ID || tag.id == PPG_ID)){
                    buildPaths(tag.id);
                    foundTag = tag;
                    tagFound = true;
                    break;
                }
            }
        }
        else if (chainNum == 21 && tagFound) {
            switch (pathState) {
                case 0:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb21Grab1GP1);
                        timer.reset();
                        timerCount = 0;
                    }
                    break;

                case 1:
                    break;

                case 2:
                    if (!fol.isBusy()) {
                        setPathState(-1);
                    }
                    break;
            }
        }
        else if (chainNum == 22){

        }
        else if (chainNum == 23){

        }
    }

    private void setChainNum(int num){
        chainNum = num;
    }

    private void setPathState(int num){
        pathState = num;
        pathTimer.resetTimer();
    }

    private void setLsTarget(double num){
        lsTarget = num;
    }

    private void setRsTarget(double num){
        rsTarget = num;
    }

    private void setIbTarget(double num){
        ibTarget = num;
    }

    private void setElTarget(double num){
        elTarget = num;
    }


    // Updates the pos to the station
    public void updatePos(){
        fol.update();

        // Get the position of the robot
        Pose currentPose = fol.getPose();

        double currentX = currentPose.getX();
        double currentY = currentPose.getY();

        telemetry.addData("X Position", "%.2f", currentX);
        telemetry.addData("Y Position", "%.2f", currentY);
    }

}

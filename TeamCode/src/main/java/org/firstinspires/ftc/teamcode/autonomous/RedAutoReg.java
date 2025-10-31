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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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
    private final Pose preScorePose = new Pose(56, 16, Math.toRadians(56)); // PRE-LOAD SCORING POSITION
    private final Pose parkPose = new Pose(39, 33, Math.toRadians(90)); // PARKING POSITION

        // Obelisk #21 --------------------------------------------------
    private final Pose Ob21Grab1GP1 = new Pose(31, 35.5, Math.toRadians(0)); // POSITION
    private final Pose Ob21Grab1GP1CP = new Pose(70, 45, Math.toRadians(0)); // CONTROL POINT
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
    private final Pose Ob22Grab1P1 = new Pose(36, 84, Math.toRadians(0)); // POSITION
    private final Pose Ob22Grab1P1CP = new Pose(70, 96, Math.toRadians(0)); // CONTROL POINT
    private final Pose Ob22Grab2GP1 = new Pose(31, 35.5, Math.toRadians(0)); // POSITION
    private final Pose Ob22Grab2GP1CP = new Pose(56, 35.5, Math.toRadians(0)); // CONTROL POINT
    private final Pose Ob22Score1 = new Pose(61, 76.5, Math.toRadians(42)); // POSITION
    private final Pose Ob22Score1CP = new Pose(60, 50, Math.toRadians(42)); // CONTROL POINT
    private final Pose Ob22Grab1PG2 = new Pose(26, 84, Math.toRadians(0)); // POSITION
    private final Pose Ob22Grab1PG2CP = new Pose(60, 88, Math.toRadians(0)); // CONTROL POINT
    private final Pose Ob22Grab2P2 = new Pose(26, 35.5, Math.toRadians(0)); // POSITION
    private final Pose Ob22Grab2P2CP = new Pose(84, 40, Math.toRadians(0)); // CONTROL POINT
    // Score2 is same as Score1
    private final Pose Ob22Grab3 = new Pose(45, 59.8, Math.toRadians(0)); // POSITION
    private final Pose Ob22Grab3CP = new Pose(62, 62, Math.toRadians(0)); // CONTROL POINT
    private final Pose Ob22GrabPGP3 = new Pose(26, 59.8, Math.toRadians(0)); // POSITION
    private final Pose Ob22Score3 = new Pose(61, 18, Math.toRadians(56)); // POSITION

        // Obelisk #23 --------------------------------------------------
    private final Pose Ob23Grab1PP1 = new Pose(31, 84, Math.toRadians(0)); // POSITION
    private final Pose Ob23Grab1PP1CP = new Pose(68, 96, Math.toRadians(0)); // CONTROL POINT
    private final Pose Ob23Grab2G1 = new Pose(36, 35.5, Math.toRadians(0)); // POSITION
    private final Pose Ob23Grab2G1CP = new Pose(70, 45, Math.toRadians(0)); // CONTROL POINT
    private final Pose Ob23Score1 = new Pose(61, 18, Math.toRadians(56)); // POSITION
    private final Pose Ob23Grab1P2 = new Pose(31, 35.5, Math.toRadians(0)); // POSITION
    private final Pose Ob23Grab1P2CP = new Pose(60, 44, Math.toRadians(0)); // CONTROL POINT
    private final Pose Ob23Grab2PG2 = new Pose(31, 59.8, Math.toRadians(0)); // POSITION
    private final Pose Ob23Grab2PG2CP = new Pose(60, 59.8, Math.toRadians(0)); // CONTROL POINT
    // Score2 same as Score1
    private final Pose Ob23Grab3 = new Pose(19, 22, Math.toRadians(-90)); // POSITION
    private final Pose Ob23GrabPPG3 = new Pose(19, 77, Math.toRadians(-90)); // POSITION
    // Score3 same as Score1

    // SHOOTING VARS

    private DcMotor ls;
    private DcMotor rs;
    private DcMotor belt;
    private DcMotor elbow;

    private final double OVERSHOOT_VEL_MULT = 1.5;
    private final double OVERSHOOT_ANG_MULT = 1.2;
    private final double ANGLE_CONST = 2.08833333;
    private final double MAX_HEIGHT = 1.4;

    private double shootVel;
    private double shootAngle;
    private double elbowSpeed = 0.5;

    private CRServo bl;
    private CRServo br;
    private double beltSpeed = 1;

    private Servo blocker;
    private double openPos = 0.1;
    private double blockPos = 0.3;
    private ElapsedTime feedTimer;
    private double feedDur = 400;
    private double retDur = 800;
    private int feeding = 1;

    // PATH CHAINS

        // Obelisk #21
    private PathChain pathOb21PreScore, pathOb21Grab1GP1, pathOb21Grab2P1, pathOb21Score1, pathOb21Grab1G2, pathOb21Grab2PP2,
                pathOb21Score2, pathOb21Grab3, pathOb21GrabGPP3, pathOb21Score3, pathOb21Park;
        // Obelisk #22
    private PathChain pathOb22PreScore, pathOb22Grab1P1, pathOb22Grab2GP1, pathOb22Score1, pathOb22Grab1PG2, pathOb22Grab2P2,
                pathOb22Score2, pathOb22Grab3, pathOb22GrabPGP3, pathOb22Score3, pathOb22Park;
        // Obelisk #23
    private PathChain pathOb23PreScore, pathOb23Grab1PP1, pathOb23Grab2G1, pathOb23Score1, pathOb23Grab1P2, pathOb23Grab2PG2,
                pathOb23Score2, pathOb23Grab3, pathOb23GrabPPG3, pathOb23Score3, pathOb23Park;

    // OTHER VARS

    private ElapsedTime timer;
    private double dur;
    private int timerCount = -1;

    private ElapsedTime shootTimer;
    private int shootTimerCount = -1;

    @Override
    public void init(){
        // HARDWARE INIT

        ls = hardwareMap.get(DcMotor.class, "ls");
        rs = hardwareMap.get(DcMotor.class, "rs");
        belt = hardwareMap.get(DcMotor.class, "belt");
        elbow = hardwareMap.get(DcMotor.class, "elbow");

        bl = hardwareMap.get(CRServo.class, "bl");
        br = hardwareMap.get(CRServo.class, "br");
        blocker = hardwareMap.get(Servo.class, "blocker");

        ls.setDirection(DcMotorSimple.Direction.FORWARD);
        rs.setDirection(DcMotorSimple.Direction.REVERSE);
        belt.setDirection(DcMotorSimple.Direction.FORWARD);
        elbow.setDirection(DcMotor.Direction.REVERSE);

        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setPower(elbowSpeed);

        blocker.scaleRange(openPos, blockPos);
        blocker.setPosition(1);

        // TIMER INIT

        timer = new ElapsedTime();
        shootTimer = new ElapsedTime();
        feedTimer = new ElapsedTime();

        // PATH INIT

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        setPathState(-1);

        // CAMERA INIT

        tagFound = false;

        apTag = new AprilTagProcessor.Builder()
                .setCameraPose(new Position(DistanceUnit.INCH, -7, -7, 14, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, 14, 0, 0))
                .build();
        apTag.setDecimation(2);

        visPort = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Cam"))
                .addProcessor(apTag)
                .build();
    }

    public void loop(){
        fol.update();
        autonomousPathUpdate();

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
            pathOb21PreScore = fol.pathBuilder()
                    .addPath(new BezierLine(startPose, preScorePose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), preScorePose.getHeading())
                    .setBrakingStrength(4)
                    .build();

            pathOb21Grab1GP1 = fol.pathBuilder()
                    .addPath(new BezierCurve(preScorePose, Ob21Grab1GP1CP, Ob21Grab1GP1))
                    .setLinearHeadingInterpolation(preScorePose.getHeading(), Ob21Grab1GP1.getHeading())
                    .build();

            pathOb21Grab2P1 = fol.pathBuilder()
                    .addPath(new BezierCurve(Ob21Grab1GP1, Ob21Grab2P1CP, Ob21Grab2P1))
                    .setLinearHeadingInterpolation(Ob21Grab1GP1.getHeading(), Ob21Grab2P1.getHeading())
                    .build();

            pathOb21Score1 = fol.pathBuilder()
                    .addPath(new BezierLine(Ob21Grab2P1, Ob21Score1))
                    .setLinearHeadingInterpolation(Ob21Grab2P1.getHeading(), Ob21Score1.getHeading())
                    .build();

            pathOb21Grab1G2 = fol.pathBuilder()
                    .addPath(new BezierCurve(Ob21Score1, Ob21Grab1G2CP, Ob21Grab1G2))
                    .setLinearHeadingInterpolation(Ob21Score1.getHeading(), Ob21Grab1G2.getHeading())
                    .build();

            pathOb21Grab2PP2 = fol.pathBuilder()
                    .addPath(new BezierCurve(Ob21Grab1G2, Ob21Grab2PP2CP, Ob21Grab2PP2))
                    .setLinearHeadingInterpolation(Ob21Grab1G2.getHeading(), Ob21Grab2PP2.getHeading())
                    .build();

            pathOb21Score2 = fol.pathBuilder()
                    .addPath(new BezierLine(Ob21Grab2PP2, Ob21Score1))
                    .setLinearHeadingInterpolation(Ob21Grab2PP2.getHeading(), Ob21Score1.getHeading())
                    .build();

            pathOb21Grab3 = fol.pathBuilder()
                    .addPath(new BezierCurve(Ob21Score1, Ob21Grab3CP, Ob21Grab3))
                    .setLinearHeadingInterpolation(Ob21Score1.getHeading(), Ob21Grab3.getHeading())
                    .build();

            pathOb21GrabGPP3 = fol.pathBuilder()
                    .addPath(new BezierLine(Ob21Grab3, Ob21GrabGPP3))
                    .setLinearHeadingInterpolation(Ob21Grab3.getHeading(), Ob21GrabGPP3.getHeading())
                    .build();

            pathOb21Score3 = fol.pathBuilder()
                    .addPath(new BezierCurve(Ob21GrabGPP3, Ob21Score3))
                    .setLinearHeadingInterpolation(Ob21GrabGPP3.getHeading(), Ob21Score3.getHeading())
                    .build();

            pathOb21Park = fol.pathBuilder()
                    .addPath(new BezierLine(Ob21Score3, parkPose))
                    .setLinearHeadingInterpolation(Ob21Score3.getHeading(), parkPose.getHeading())
                    .build();
        }
        else if (obNum == PGP_ID){
            pathOb22PreScore = fol.pathBuilder()
                    .addPath(new BezierLine(startPose, preScorePose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), preScorePose.getHeading())
                    .setBrakingStrength(4)
                    .build();

            pathOb22Grab1P1 = fol.pathBuilder()
                    .addPath(new BezierCurve(preScorePose, Ob22Grab1P1CP, Ob22Grab1P1))
                    .setLinearHeadingInterpolation(preScorePose.getHeading(), Ob22Grab1P1.getHeading())
                    .build();

            pathOb22Grab2GP1 = fol.pathBuilder()
                    .addPath(new BezierCurve(Ob22Grab1P1, Ob22Grab2GP1CP, Ob22Grab2GP1))
                    .setLinearHeadingInterpolation(Ob22Grab1P1.getHeading(), Ob22Grab2GP1.getHeading())
                    .build();

            pathOb22Score1 = fol.pathBuilder()
                    .addPath(new BezierCurve(Ob22Grab1P1, Ob22Score1CP, Ob22Score1))
                    .setLinearHeadingInterpolation(Ob22Grab2GP1.getHeading(), Ob22Score1.getHeading())
                    .build();

            pathOb22Grab1PG2 = fol.pathBuilder()
                    .addPath(new BezierCurve(Ob22Score1, Ob22Grab1PG2CP, Ob22Grab1PG2))
                    .setLinearHeadingInterpolation(Ob22Score1.getHeading(), Ob22Grab1PG2.getHeading())
                    .build();

            pathOb22Grab2P2 = fol.pathBuilder()
                    .addPath(new BezierCurve(Ob22Grab1PG2, Ob22Grab2P2CP, Ob22Grab2P2))
                    .setLinearHeadingInterpolation(Ob22Grab1PG2.getHeading(), Ob22Grab2P2.getHeading())
                    .build();

            pathOb22Score2 = fol.pathBuilder()
                    .addPath(new BezierCurve(Ob22Grab2P2, Ob22Score1CP, Ob22Score1))
                    .setLinearHeadingInterpolation(Ob22Grab2P2.getHeading(), Ob22Score1.getHeading())
                    .build();

            pathOb22Grab3 = fol.pathBuilder()
                    .addPath(new BezierCurve(Ob22Score1, Ob22Grab3CP, Ob22Grab3))
                    .setLinearHeadingInterpolation(Ob22Score1.getHeading(), Ob22Grab3.getHeading())
                    .build();

            pathOb22GrabPGP3 = fol.pathBuilder()
                    .addPath(new BezierLine(Ob22Grab3, Ob22GrabPGP3))
                    .setLinearHeadingInterpolation(Ob22Grab3.getHeading(), Ob22GrabPGP3.getHeading())
                    .build();

            pathOb22Score3 = fol.pathBuilder()
                    .addPath(new BezierLine(Ob22GrabPGP3, Ob22Score3))
                    .setLinearHeadingInterpolation(Ob22GrabPGP3.getHeading(), Ob22Score3.getHeading())
                    .build();

            pathOb22Park = fol.pathBuilder()
                    .addPath(new BezierLine(Ob22Score3, parkPose))
                    .setLinearHeadingInterpolation(Ob22Score3.getHeading(), parkPose.getHeading())
                    .build();
        }
        else if (obNum == PPG_ID){
            pathOb23PreScore = fol.pathBuilder()
                    .addPath(new BezierLine(startPose, preScorePose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), preScorePose.getHeading())
                    .setBrakingStrength(4)
                    .build();

            pathOb23Grab1PP1 = fol.pathBuilder()
                    .addPath(new BezierCurve(preScorePose, Ob23Grab1PP1CP, Ob23Grab1PP1))
                    .setLinearHeadingInterpolation(preScorePose.getHeading(), Ob23Grab1PP1.getHeading())
                    .build();

            pathOb23Grab2G1 = fol.pathBuilder()
                    .addPath(new BezierCurve(Ob23Grab1PP1, Ob23Grab2G1CP, Ob23Grab2G1))
                    .setLinearHeadingInterpolation(Ob23Grab1PP1.getHeading(), Ob23Grab2G1.getHeading())
                    .build();

            pathOb23Score1 = fol.pathBuilder()
                    .addPath(new BezierLine(Ob23Grab2G1, Ob23Score1))
                    .setLinearHeadingInterpolation(Ob23Grab2G1.getHeading(), Ob23Score1.getHeading())
                    .build();

            pathOb23Grab1P2 = fol.pathBuilder()
                    .addPath(new BezierCurve(Ob23Score1, Ob23Grab1P2CP, Ob23Grab1P2))
                    .setLinearHeadingInterpolation(Ob23Score1.getHeading(), Ob23Grab1P2.getHeading())
                    .build();

            pathOb23Grab2PG2 = fol.pathBuilder()
                    .addPath(new BezierCurve(Ob23Grab1P2, Ob23Grab2PG2CP, Ob23Grab2PG2))
                    .setLinearHeadingInterpolation(Ob23Grab1P2.getHeading(), Ob23Grab2PG2.getHeading())
                    .build();

            pathOb23Score2 = fol.pathBuilder()
                    .addPath(new BezierLine(Ob23Grab2PG2, Ob23Score1))
                    .setLinearHeadingInterpolation(Ob23Grab2PG2.getHeading(), Ob23Score1.getHeading())
                    .build();

            pathOb23Grab3 = fol.pathBuilder()
                    .addPath(new BezierLine(Ob23Score1, Ob23Grab3))
                    .setLinearHeadingInterpolation(Ob23Score1.getHeading(), Ob23Grab3.getHeading())
                    .build();

            pathOb23GrabPPG3 = fol.pathBuilder()
                    .addPath(new BezierLine(Ob23Grab3, Ob23GrabPPG3))
                    .setLinearHeadingInterpolation(Ob23Grab3.getHeading(), Ob23GrabPPG3.getHeading())
                    .build();

            pathOb23Score3 = fol.pathBuilder()
                    .addPath(new BezierLine(Ob23GrabPPG3, Ob23Score1))
                    .setLinearHeadingInterpolation(Ob23GrabPPG3.getHeading(), Ob23Score1.getHeading())
                    .build();

            pathOb23Park = fol.pathBuilder()
                    .addPath(new BezierLine(Ob23Score1, parkPose))
                    .setLinearHeadingInterpolation(Ob23Score1.getHeading(), parkPose.getHeading())
                    .build();
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
                case -1:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb21PreScore);
                        setShootPos(Ob21Score1.getX(), Ob21Score1.getY(), 135, 135);
                        shoot();
                        timerCount++;
                    }

                    if (shootTimerCount == 2){
                        shootTimerCount = -1;
                        timerCount = -1;
                        setPathState(0);
                    }
                    break;

                case 0:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb21Grab1GP1);
                        setShootPos(Ob21Score1.getX(), Ob21Score1.getY(), 135, 135);
                        setPathState(1);
                    }
                    break;

                case 1:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb21Grab2P1);
                        setPathState(2);
                    }
                    break;

                case 2:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb21Score1);
                        shoot();
                        timerCount++;
                    }

                    if (shootTimerCount == 2){
                        shootTimerCount = -1;
                        timerCount = -1;
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb21Grab1G2);
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb21Grab2PP2);
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb21Score2);
                        shoot();
                        timerCount++;
                    }

                    if (shootTimerCount == 2){
                        shootTimerCount = -1;
                        timerCount = -1;
                        setPathState(6);
                    }
                    break;
                case 6:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb21Grab3);
                        setShootPos(Ob21Score3.getX(), Ob21Score3.getY(), 135, 135);
                        setPathState(7);
                    }
                    break;
                case 7:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb21GrabGPP3);
                        setPathState(8);
                    }
                    break;
                case 8:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb21Score3);
                        shoot();
                        timerCount++;
                    }

                    if (shootTimerCount == 2){
                        shootTimerCount = -1;
                        timerCount = -1;
                        setPathState(9);
                    }
                    break;
                case 9:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb21Park);
                        setPathState(10);
                    }
                    break;

                case 10:
                    if (!fol.isBusy()) {
                        setPathState(-2);
                    }
                    break;
            }
        }
        else if (chainNum == 22 && tagFound){
            switch (pathState) {
                case -1:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb22PreScore);
                        setShootPos(Ob21Score1.getX(), Ob21Score1.getY(), 135, 135);
                        shoot();
                        timerCount++;
                    }

                    if (shootTimerCount == 2){
                        shootTimerCount = -1;
                        timerCount = -1;
                        setPathState(0);
                    }
                    break;

                case 0:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb22Grab1P1);
                        setShootPos(Ob22Score1.getX(), Ob22Score1.getY(), 135, 135);
                        setPathState(1);
                    }
                    break;

                case 1:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb22Grab2GP1);
                        setPathState(2);
                    }
                    break;

                case 2:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb22Score1);
                        shoot();
                        timerCount++;
                    }

                    if (shootTimerCount == 2){
                        shootTimerCount = -1;
                        timerCount = -1;
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb22Grab1PG2);
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb22Grab2P2);
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb22Score2);
                        shoot();
                        timerCount++;
                    }

                    if (shootTimerCount == 2){
                        shootTimerCount = -1;
                        timerCount = -1;
                        setPathState(6);
                    }
                    break;
                case 6:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb22Grab3);
                        setShootPos(Ob22Score3.getX(), Ob22Score3.getY(), 135, 135);
                        setPathState(7);
                    }
                    break;
                case 7:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb22GrabPGP3);
                        setPathState(8);
                    }
                    break;
                case 8:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb22Score3);
                        shoot();
                        timerCount++;
                    }

                    if (shootTimerCount == 2){
                        shootTimerCount = -1;
                        timerCount = -1;
                        setPathState(9);
                    }
                    break;
                case 9:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb22Park);
                        setPathState(10);
                    }
                    break;

                case 10:
                    if (!fol.isBusy()) {
                        setPathState(-2);
                    }
                    break;
            }
        }
        else if (chainNum == 23 && tagFound){
            switch (pathState) {
                case -1:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb23PreScore);
                        setShootPos(Ob21Score1.getX(), Ob21Score1.getY(), 135, 135);
                        shoot();
                        timerCount++;
                    }

                    if (shootTimerCount == 2){
                        shootTimerCount = -1;
                        timerCount = -1;
                        setPathState(0);
                    }
                    break;

                case 0:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb23Grab1PP1);
                        setShootPos(Ob23Score1.getX(), Ob23Score1.getY(), 135, 135);
                        setPathState(1);
                    }
                    break;

                case 1:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb23Grab2G1);
                        setPathState(2);
                    }
                    break;

                case 2:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb23Score1);
                        shoot();
                        timerCount++;
                    }

                    if (shootTimerCount == 2){
                        shootTimerCount = -1;
                        timerCount = -1;
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb23Grab1P2);
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb23Grab2PG2);
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb23Score2);
                        shoot();
                        timerCount++;
                    }

                    if (shootTimerCount == 2){
                        shootTimerCount = -1;
                        timerCount = -1;
                        setPathState(6);
                    }
                    break;
                case 6:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb23Grab3);
                        setPathState(7);
                    }
                    break;
                case 7:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb23GrabPPG3);
                        setPathState(8);
                    }
                    break;
                case 8:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb23Score3);
                        shoot();
                        timerCount++;
                    }

                    if (shootTimerCount == 2){
                        shootTimerCount = -1;
                        timerCount = -1;
                        setPathState(9);
                    }
                    break;
                case 9:
                    if (!fol.isBusy() && timerCount == -1){
                        fol.followPath(pathOb23Park);
                        setPathState(10);
                    }
                    break;

                case 10:
                    if (!fol.isBusy()) {
                        setPathState(-2);
                    }
                    break;
            }
        }
    }

    private void setChainNum(int num){
        chainNum = num;
    }

    private void setPathState(int num){
        pathState = num;
        pathTimer.resetTimer();
    }

    // This method sets the speed of the shooter motors and the angle of the shooting posit
    private void setShootPos(double ix, double iy, double fx, double fy){
    /* dist is the total distance the ball will travel until it hits the ground
       It's divided by 40 to turn the field units into meters
       Then, it's multiplied by 1.3 because the ball will hit the goal first, so using
       equation, it'll be about 1 meter high (the height of the goal) when it hit our r
     */
        double dist = (Math.sqrt(Math.pow(fx - ix, 2) + Math.pow(fy - iy, 2)) / 40) * 1.3;

        // The angle and velocity are both calculated using the distance we found
        shootAngle = (distToAngle(dist) - 45) * OVERSHOOT_ANG_MULT;
        shootVel = angleToVel(distToAngle(dist)) * OVERSHOOT_VEL_MULT;

        telemetry.addData("Distance", dist);
        telemetry.addData("Angle", shootAngle);
        telemetry.addData("Real Angle", distToAngle(dist));
        telemetry.addData("Velocity", shootVel);
        telemetry.addData("Real Velocity", angleToVel(distToAngle(dist)));
        telemetry.update();

        setElbowTarget(angleToEncoder(shootAngle));
    }

    private double distToAngle(double dist){
        return Math.toDegrees(Math.atan(54.88 / (9.8 * dist)));
    }

    // This function translates angle to velocity using the already set maximum height
    private double angleToVel(double angle){
        return Math.sqrt((MAX_HEIGHT * 19.6) / Math.pow(Math.sin(Math.toRadians(angle)), 2));
    }

    // This function translates velocity to motor power specifically for 6000 RPM motors co
    private double velToPow(double vel){
        return vel / (7.2 * Math.PI);
    }

    // This function translates an angle in degrees to an encoder value on 223 RPM motors
    private double angleToEncoder(double angle){
        return angle * ANGLE_CONST;
    }

    private void setElbowTarget(double angle){
        elbow.setTargetPosition((int) angleToEncoder(angle));
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void shoot(){
        shootTimer.reset();
        if (shootTimerCount == -1)
            shootTimerCount = 0;

        while (shootTimer.milliseconds() < 2000 && shootTimerCount == 0){
            ls.setPower(velToPow(shootVel));
            rs.setPower(velToPow(shootVel));
        }
        shootTimer.reset();
        shootTimerCount = 1;
        while (shootTimer.milliseconds() < 3000 && shootTimerCount == 1){
            feedLauncher();
        }
        shootTimerCount = 2;

        ls.setPower(0);
        rs.setPower(0);
        blocker.setPosition(0);
    }

    private void runBelt(double speed){
        belt.setPower(speed);
        br.setPower(speed);
        bl.setPower(-speed);
    }

    private void feedLauncher(){
        if (feedTimer.milliseconds() < feedDur && feeding == 1){
            blocker.setPosition(1);
            runBelt(0);
        }
        else if (feedTimer.milliseconds() < retDur && feeding == -1) {
            blocker.setPosition(0);
            runBelt(-beltSpeed);
        }
        else {
            feeding *= -1;
            feedTimer.reset();
        }
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

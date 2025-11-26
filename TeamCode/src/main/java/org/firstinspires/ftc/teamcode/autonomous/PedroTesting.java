package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "PedroTesting", group = "autonomous")
public class PedroTesting extends OpMode{

    // PEDROPATHING VARS

    private Follower follower;

    private Timer pathTimer, actionTimer, opmodeTimer; // Game timer
    private int pathState; // Current path #
    private int chainNum;
    private int ballNum = 2;
    private int shootPos = 1;


    // CAMERA VARS
    private Limelight3A cam;
    private VisionPortal visPort;
    private AprilTagProcessor apTag;
    private LLResultTypes.FiducialResult foundTag;
    private boolean tagFound;
    private final int GPP_ID = 21;
    private final int PGP_ID = 22;
    private final int PPG_ID = 23;


    // POSITIONS

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // STARTING POSITION, where your bot starts relative to field coords

    private final Pose preScorePose = new Pose(64, 16, Math.toRadians(123)); // PRE-LOAD SCORING POSITION

    private final Pose parkPose = new Pose(50, 72, Math.toRadians(132)); // PARKING POSITION, this is the parking position



    // MOTORS
    private DcMotorEx ls;
    private DcMotorEx rs;
    private DcMotor belt;
    private DcMotor elbow;

    // SERVOS
    private CRServo bl;
    private CRServo br;
    private CRServo ascension;
    private Servo blocker;


    // SHOOTING VARS
    private final double OVERSHOOT_VEL_MULT = 1.68;
    private final double OVERSHOOT_ANG_MULT = 1;
    private final double ANGLE_CONST = 2.08833333;
    private final int ELBOW_GEAR_RATIO = 4;
    private final double MAX_HEIGHT = 1.4;
    private double shootVel;
    private double shootAngle;


    // INTAKE VARS
    private double elbowSpeed = 0.5;
    private double beltSpeed = -1;


    // SERVO VARS
    private double openPos = 0.53;
    private double feedPos = 0.02;


    // TIMER VARS
    private ElapsedTime feedTimer;
    private double ascendDur = 500;
    private double feedDur = 400; // was 50
    private double retDur = 1000;
    private int feeding = 0;


    // PATH CHAINS
    private PathChain pathPreScore, pathParkPose;


/***    // OTHER VARS
    private ElapsedTime timer;
    private double dur;

    private ElapsedTime shootTimer;
    private int shootTimerCount = -1; */

    @Override
    public void init(){
        // HARDWARE INIT
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        ls = hardwareMap.get(DcMotorEx.class, "ls");
        rs = hardwareMap.get(DcMotorEx.class, "rs");
        belt = hardwareMap.get(DcMotor.class, "belt");
        elbow = hardwareMap.get(DcMotor.class, "elbow");

        bl = hardwareMap.get(CRServo.class, "bl");
        br = hardwareMap.get(CRServo.class, "br");
        ascension = hardwareMap.get(CRServo.class, "ascension");
        blocker = hardwareMap.get(Servo.class, "blocker");

        ls.setDirection(DcMotorSimple.Direction.FORWARD);
        rs.setDirection(DcMotorSimple.Direction.REVERSE);
        belt.setDirection(DcMotorSimple.Direction.FORWARD);
        elbow.setDirection(DcMotor.Direction.REVERSE);

        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setPower(elbowSpeed);

        blocker.scaleRange(feedPos, openPos);
        //blocker.setPosition(1);


        // TIMER INIT

        /*timer = new ElapsedTime();
        shootTimer = new ElapsedTime();
        feedTimer = new ElapsedTime();

        // PATH INIT

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer(); */



        // The magic begins
        buildPaths();
        setPathState(-1);

    }

    public void loop(){
        follower.update();
        autonomousPathUpdate();
        updatePos();
    }

    // HERE WE TRANSLATE THE PATHS INTO A FORMAT OF CODE THE ROBOT CAN USE
    public void buildPaths(){
        // Here we do the format as follows
        pathPreScore = follower.pathBuilder()
                .addPath(new BezierLine(startPose, preScorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), preScorePose.getHeading())
                .build();

        pathParkPose = follower.pathBuilder()
                .addPath(new BezierLine(preScorePose, parkPose))
                .setLinearHeadingInterpolation(preScorePose.getHeading(), parkPose.getHeading())
                .build();


//        pathPreScore = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, preScorePose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), preScorePose.getHeading())
//                .build();

    }



    public void autonomousPathUpdate() {

        switch (pathState) {
            // Edited so it runs to the first pose and scores preloads
            case -1:
                if (!follower.isBusy()) {
                    //follower.followerlowPath(pathPreScore);
                    //follower.followPath(preScoreP);
                    setPathState(0);
                }
                break;
        }
    }

    private void setChainNum(int num){
        chainNum = num;
    }

    private void setPathState(int num){
        pathState = num;
        pathTimer.resetTimer();
    }



    // Updates the pos to the station
    public void updatePos(){
        follower.update();

        // Get the position of the robot
        Pose currentPose = follower.getPose();

        double currentX = currentPose.getX();
        double currentY = currentPose.getY();

        telemetry.addData("Current Path State", pathState);
        telemetry.addData("X Position", "%.2f", currentX);
        telemetry.addData("Y Position", "%.2f", currentY);
        telemetry.addData("Right Launch Power", rs.getPower());
        telemetry.addData("Left Launch Power", ls.getPower());
        telemetry.update();
    }
}
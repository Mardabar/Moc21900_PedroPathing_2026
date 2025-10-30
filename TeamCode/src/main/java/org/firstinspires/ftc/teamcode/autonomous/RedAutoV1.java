package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotPoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "RedAutoV1", group = "autonomous")
public class RedAutoV1 extends OpMode {

    // Variables needed for pedro, a bunch is copy paste from last year
    private Follower follower;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState; // Current path
    private int timerCount = -1;

    // Variables for april tag stuff but idk how to use yet soooo good job Leif and Vlad

    private Limelight3A cam;
    private VisionPortal visPort;
    private AprilTagProcessor apTag;
    private AprilTagDetection foundTag;
    private boolean tagFound;

    private final int GPP_ID = 21;
    private final int PGP_ID = 22;
    private final int PPG_ID = 23;

    // Positions
    private final Pose startPose = new Pose(56,8.5,90); // Starting position
    private final Pose parkPose = new Pose(56,36,90); // Ending position





    // Motors/Servo variables for shooting

    private DcMotor ls;
    private DcMotor rs;
    private DcMotor belt;
    private DcMotor elbow;


    private double elbowSpeed = 0.5;

    private CRServo bl;
    private CRServo br;
    private double beltSpeed = 1;

    private Servo blocker;
    private double openPos = 0.1;
    private double blockPos = 0.3;
    private ElapsedTime timer;


    // Path chains for the path...
    // Very big move here guys

    private PathChain pathStartPose, pathParkPose;


    @Override
    public void init(){
        // Typical init stuff here

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

        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setPower(elbowSpeed);

        blocker.scaleRange(openPos, blockPos);

        // Also have to set timer here
        // Will add more stuff later but base timer is good
        timer = new ElapsedTime();

        // Path init stuff
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        setPathState(0);
    }

    public void loop(){
        follower.update();
        autonomousPathUpdate();

        updatePos();

        // Saves robot position and puts into our file to hold, very easily could be fixed but erm idk how to rn
        // Ask Vlad
        Pose finalPose = follower.getPose();
        RobotPoseStorage.currentPose = finalPose;

    }

    // Here is where the pathing actually happens taking and using the poses values
    public void buildPaths(){
        // Moving from start pose to park
        pathParkPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, parkPose))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
                if (!follower.isBusy() && timerCount == -1) {
                    follower.followPath(pathParkPose);
                    setPathState(1);
                }
                break;

            case 1:
        }
    }


    private void setPathState(int num){
        pathState = num;
        pathTimer.resetTimer();
    }

    public void updatePos(){
        follower.update();

        // Get the position of the robot and print to station

        telemetry.addData("X Position", follower.getPose().getX());
        telemetry.addData("Y Position", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path state", pathState);
        telemetry.update();
    }
}

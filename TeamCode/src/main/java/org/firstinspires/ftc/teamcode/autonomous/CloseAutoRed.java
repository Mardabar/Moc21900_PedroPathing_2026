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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "CloseAutoRed", group = "autonomous")
public class CloseAutoRed extends OpMode {

    // Variables needed for pedro, a bunch is copy paste from last year
    private Follower follower;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState; // Current path

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
    private final Pose startPose = new Pose(123,122.5, Math.toRadians(36)); // Starting position
    private final Pose shootPose = new Pose(86,91,Math.toRadians(36)); // Shooting position
    private final Pose parkPose = new Pose(97,70,Math.toRadians(36));





    // Motors/Servo variables for shooting

    private DcMotor ls;
    private DcMotor rs;
    private DcMotor belt;
    private DcMotor elbow;



    private CRServo bl;
    private CRServo br;

    private Servo blocker;
    private double openPos = 0.1;
    private double blockPos = 0.3;

    private double dur;
    private int timerCount = -1;

    private ElapsedTime timer;
    private ElapsedTime shootTimer;
    private ElapsedTime feedTimer;
    private int shootTimerCount = -1;
    private double feedDur = 700;
    private double retDur = 1000;
    private double gapDur = 500;
    private double beltDur = 500;
    private double blockerDur = 300;
    private int feeding = 1;
    double beltSpeed = 1;



    // Path chains for the path...
    // Very big move here guys

    private PathChain pathShootPose, pathParkPose;


    @Override
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        // Typical init stuff here

        ls = hardwareMap.get(DcMotor.class, "ls");
        rs = hardwareMap.get(DcMotor.class, "rs");
        belt = hardwareMap.get(DcMotor.class, "belt");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bl = hardwareMap.get(CRServo.class, "bl");
        br = hardwareMap.get(CRServo.class, "br");
        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.scaleRange(0.0, 1.0);
        blocker.setPosition(0.54);

        ls.setDirection(DcMotorSimple.Direction.FORWARD);
        rs.setDirection(DcMotorSimple.Direction.REVERSE);
        belt.setDirection(DcMotorSimple.Direction.FORWARD);
        elbow.setDirection(DcMotor.Direction.REVERSE);
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Also have to set timer here
        timer = new ElapsedTime();
        shootTimer = new ElapsedTime();
        feedTimer = new ElapsedTime();
        // Path init stuff
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        setPathState(0);
        buildPaths();
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
        pathShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setConstantHeadingInterpolation(Math.toRadians(36))
                .build();

        pathParkPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setConstantHeadingInterpolation(Math.toRadians(36))
                .build();
    }

    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
                if (!follower.isBusy() && timerCount == -1) {
                    follower.followPath(pathShootPose);
                    shootTimerCount = 3;
                }

                // Very important, wait till the motor is done moving then moves on in code
//                if (timerCount == 3 && !follower.isBusy()){
//                    timerCount = 4;
//                }

                if (!follower.isBusy() && shootTimerCount == 3){
                    shootTimerCount = -1;
                    timerCount = -1;
                    shoot();
                    setPathState(1);
                }
                break;

            case 1:
                if (!follower.isBusy() && timerCount == -1){
                    follower.followPath(pathParkPose);
                }
                break;
        }
    }

    /*** Prints stuff to drive station */
    public void updatePos(){
        //follower.update();

        telemetry.addData("X Position", follower.getPose().getX());
        telemetry.addData("Y Position", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path state", pathState);
        telemetry.addData("currentPath", follower.getCurrentPath()); // VERY IMPORTANT
        telemetry.update();
    }


    // Displays PathState
    private void setPathState(int num){
        pathState = num;
        pathTimer.resetTimer();
    }

    // Motor moving stuff
    private void shoot(){
        shootTimer.reset();
        if (shootTimerCount == -1)
            shootTimerCount = 0;

        while (shootTimer.milliseconds() < 2000 && shootTimerCount == 0){
            ls.setPower(.47);
            rs.setPower(.47);
        }
        shootTimer.reset();
        shootTimerCount = 1;
        // 7800 should be about 3 cycles
        while (shootTimer.milliseconds() < 7800 && shootTimerCount == 1){
            feedLauncher();
        }
        shootTimerCount = 2;

        ls.setPower(0);
        rs.setPower(0);
        blocker.setPosition(0);
        runBelt(0);
    }

    // Intake system
    private void feedLauncher(){
        // 1st state launch 1st ball
        if (feeding == 1) {
            if (feedTimer.milliseconds() < feedDur) {
                blocker.setPosition(1);
                runBelt(0);
            } else {
                feeding = 2;
                feedTimer.reset();
            }
        }

        // 2nd state uhhh load ball w belt
        else if (feeding == 2) {
            if (feedTimer.milliseconds() < retDur) {
                blocker.setPosition(.54);
                runBelt(-beltSpeed);
            } else {
                feeding = 3;
                feedTimer.reset();
            }
        }

        // 3rd state advance ig, moves intake stuff while servo is down
        // Honestly might be able to combine these two at some point
        else if (feeding == 3) {
            if (feedTimer.milliseconds() < beltDur) {
                blocker.setPosition(.54);
                runBelt(-beltSpeed);
            } else {
                feeding = 4;
                feedTimer.reset();
            }
        }
        else if (feeding == 4) {
            // Now servo moves up once ball is in the position,
            // Very slow rn but once we get wtv is wrong with the servos solved itll be good
            if (feedTimer.milliseconds() < blockerDur){
                blocker.setPosition(.01);
                runBelt(0);
            } else {
                feeding = 1;
                feedTimer.reset();
            }
        }
    }

    private void runBelt(double speed){
        belt.setPower(speed);
        br.setPower(speed);
        bl.setPower(-speed);
    }



    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

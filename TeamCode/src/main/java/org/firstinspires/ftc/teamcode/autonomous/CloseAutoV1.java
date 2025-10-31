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

@Autonomous(name = "CloseAutoV1", group = "autonomous")
public class CloseAutoV1 extends OpMode {

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
    private final Pose startPose = new Pose(21,122.5, Math.toRadians(144)); // Starting position
    private final Pose shootPose = new Pose(58,91,Math.toRadians(144)); // Shooting position
    private final Pose parkPose = new Pose(105.25,33,90);





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
    private ElapsedTime timer;
    private double dur;
    private int timerCount = -1;

    private ElapsedTime shootTimer;
    private ElapsedTime feedTimer;
    private int shootTimerCount = -1;
    private double feedDur = 400;
    private double retDur = 800;
    private int feeding = 1;

    private ElapsedTime elbowTimer;
    private int elbowTargetPosition;

    double elbowPos = 0;
    double beltSpeed = 1;

    public void setElbowTargetPosition(int targetPosition, double power) {
        // Stop and reset the encoder if necessary, then set to RUN_TO_POSITION
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setTargetPosition(targetPosition);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setPower(power);
    }

    public boolean isElbowMoveFinished() {
        return !elbow.isBusy();
    }

    public void startElbowTimeMove(double power, long duration) {
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        dur = duration;
        elbowTimer.reset();

        // Start the motor
        elbow.setPower(power);
    }

    /**
     * Checks if the time for the time-based move has elapsed.
     * @return True if the duration is over, false otherwise.
     */
    public boolean isElbowTimeMoveFinished() {
        return elbowTimer.milliseconds() >= dur;
    }

    /**
     * Stops the elbow motor after a time-based move.
     */
    public void stopElbowTimeMove() {
        elbow.setPower(0);
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

    public int getElbowPos() {
        elbowPos = elbow.getCurrentPosition();
        return elbow.getCurrentPosition();
    }


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

        bl = hardwareMap.get(CRServo.class, "bl");
        br = hardwareMap.get(CRServo.class, "br");
        blocker = hardwareMap.get(Servo.class, "blocker");

        ls.setDirection(DcMotorSimple.Direction.FORWARD);
        rs.setDirection(DcMotorSimple.Direction.REVERSE);
        belt.setDirection(DcMotorSimple.Direction.FORWARD);
        elbow.setDirection(DcMotor.Direction.REVERSE);

        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        blocker.scaleRange(openPos, blockPos);

        // Also have to set timer here
        // Will add more stuff later but base timer is good
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
                .setConstantHeadingInterpolation(Math.toRadians(144))
                .build();

        pathParkPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(90))
                .build();
    }

    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
                if (!follower.isBusy() && timerCount == -1) {
                    follower.followPath(pathShootPose);
                    setElbowTargetPosition(1180, 0.6);
//
//                    dur = 500;
//                    timer.reset();
//
//                  setElbowTarget(712);
                      setPathState(1);
                }

//                if (timer.milliseconds() >= dur && timerCount == -1){
//                    timerCount = 0;
//                }

//                if (!follower.isBusy() && timerCount == 0){
//                    dur = 600;
//                    timerCount = -1;
//                    setPathState(1);
//                }

                break;

            case 1:
//                if (!follower.isBusy() && timerCount == -1){
//                    follower.followPath(pathParkPose);
//                }
        }
    }


    private void setPathState(int num){
        pathState = num;
        pathTimer.resetTimer();
    }

    private void intakeSystem(double power){
        belt.setPower(power);
        br.setPower(power);
        bl.setPower(-power);
    }

    public void updatePos(){
        //follower.update();

        // Get the position of the robot and print to station

        telemetry.addData("X Position", follower.getPose().getX());
        telemetry.addData("Y Position", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path state", pathState);
        telemetry.addData("currentPath", follower.getCurrentPath()); // VERY IMPORTANT
        telemetry.update();
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

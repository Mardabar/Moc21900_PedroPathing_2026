package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;




@Configurable
@TeleOp(name = "StraferOpV2")
public class StraferOpV2 extends LinearOpMode {

    // Motors and Servos
    private DcMotor lb;
    private DcMotor rb;
    private DcMotor lf;
    private DcMotor rf;
    private DcMotorEx ls;
    private DcMotorEx rs;
    private DcMotor belt;
    private DcMotorEx elbow;

    private CRServo ascension;
    private Servo blocker;
    private CRServo br;
    private CRServo bl;

    // Other variables we'll need
    private String shootMode = "Middle";
    private double launchPower;
    private double speed = 0.5;

    // Holds the power of the motor so it doesn't get messed up with the if statement here
    private double currentPower = middlePower;


    // Will be used to update and change in real time until I find proper values for each individual shootMode
    public static double middlePower = 0.50;  // Default power for Middle D-Pad Up
    public static double backPower = 0.75;    // Default power for Back D-Pad Down
    public static double topPower = 0.55;     // Default power for Top D-Pad Right
    public static double boxPower = 0.40;     // Default power for Box D-Pad Left


    // Taking and storing robot pos from auto

    private double elbowSpeed = 0.4;

    private double shootRot;
    private ElapsedTime feedTimer, blockTimer;

    private double feedDur = 200;
    private double ascendDur = 900;
    private double retDur = 600;
    private int feeding;

    private Follower follower;

    @Override
    public void runOpMode() {


        // Motors are set to each of its variables
        rf = hardwareMap.get(DcMotor.class, "rf");
        lf = hardwareMap.get(DcMotor.class, "lf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rs = hardwareMap.get(DcMotorEx.class, "rs");
        ls = hardwareMap.get(DcMotorEx.class, "ls");
        belt = hardwareMap.get(DcMotorEx.class, "belt");
        elbow = hardwareMap.get(DcMotorEx.class, "elbow");


        // Zero power behaviors are set for the motors
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Motor direction is set for straight forward values prolly will have to change later
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        ls.setDirection(DcMotorEx.Direction.FORWARD);
        rs.setDirection(DcMotorEx.Direction.REVERSE);
        belt.setDirection(DcMotor.Direction.FORWARD);
        elbow.setDirection(DcMotorEx.Direction.REVERSE);

        // Stopping and resetting encoders in erm the uh other motors
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setPower(elbowSpeed);


        ls.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rs.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        // Servo naming and scale ranges
        blocker = hardwareMap.get(Servo.class, "blocker");
        ascension = hardwareMap.get(CRServo.class, "ascension");
        br = hardwareMap.get(CRServo.class, "br");
        bl = hardwareMap.get(CRServo.class, "bl");

        blocker.scaleRange(0.02, 0.53);
        blocker.setPosition(1);

        // Timers
        blockTimer = new ElapsedTime();
        feedTimer = new ElapsedTime();


        // Robot waits for the opmode to be activated
        waitForStart();


        while (opModeIsActive()) {

            // Controls drive motors
            drive();

            // Apply's power to motors
            updateLaunchers();

            // Prints stuff to station
            updatePos();

            // Function that launches za boals
            ballToLaunch();

            // Power mode swap
            powerSwap();

            // feeding system
            feedLauncher();



//            // New servo Gabe added that is in the launcher.
//            if (gamepad1.y) {
//                ascension.setPower(1);
//            } else {
//                ascension.setPower(0);
//            }
//            // Function for new servo Gabe added
//            if (gamepad2.y)
//                blocker.setPosition(0.1);
//            else
//                blocker.setPosition(.54);




            // Stuff for controlling the elbow
            /** ADD SERVO CODE HERE WHEN GABE GETS HIS DESIGN FINISHED */
            if (gamepad2.left_trigger > 0.2) {
                elbow.setPower(0.4);
            } else if (gamepad2.right_trigger > 0.2) {
                elbow.setPower(-0.4);
            } else {
                elbow.setPower(0);
            }


            // Function that turns the motors on and off
            if (gamepad2.a) {
                // If Y is pressed sets the desired speed
                launchPower = currentPower;
            } else {
                launchPower = 0;
            }

            if (gamepad2.left_bumper) {
                // intake system sucks in
                belt.setPower(.5);
                br.setPower(.5);
                bl.setPower(-.5);
            } else if (gamepad2.right_bumper) {
                // intake sucks out
                belt.setPower(-.5);
                br.setPower(-.5);
                bl.setPower(.5);
            } else {
                belt.setPower(0);
                br.setPower(0);
                bl.setPower(0);
            }



        }


    }


    // Prints a bunch of values to driverstation
    public void updatePos() {
        follower.update();

        telemetry.addData("Motor goal power: ", launchPower);
        telemetry.addData("Motor actual power: ", ls.getPower());
        telemetry.addData("Current pose: ", shootMode);
        telemetry.addData("X Position", follower.getPose().getX());
        telemetry.addData("Y Position", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();


    }

    // Updates the motor shooting power and makes code easier to read up top
    public void updateLaunchers() {
        rs.setPower(-launchPower);
        ls.setPower(launchPower);
    }

    // SET ELBOW TARGET POSITION
    private void setElbowTarget(double angle) {
        elbow.setTargetPosition((int) angle);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // FEED BALL INTO LAUNCHER SYSTEM
    // Not sure if I like the whole thing being automatic
    private void feedLauncher(){
        if (feedTimer.milliseconds() < feedDur && feeding == 0){
            blocker.setPosition(0);
            runBelt(0);
        }
        else if (feedTimer.milliseconds() < ascendDur && feeding == 1){
            ascension.setPower(1);
        }
        else if (feedTimer.milliseconds() < retDur && feeding == 2) {
            blocker.setPosition(1);
            ascension.setPower(0);
            runBelt(-1);
        }
    }
    
    // SECONDARY BALL TO LAUNCH SYSTEM IF I DONT LIKE LEFIS
    // Basically will just check for a previous condition to be met
    public void ballToLaunch(){
        // Starts off by setting the ball to be up in the launcher but not to the motors yet so the motors can reach thier desired power
        if (gamepad1.aWasPressed()){
            blocker.setPosition(0);
            rs.setPower(currentPower);
            ls.setPower(currentPower);
        }
        // Checks for the motor to be at the desired velocity, if it is then itl shoot the ball once
        if (rs.getPower() == currentPower){
            ascension.setPower(1);
            feedTimer.reset();
        }
        // Sets everything to the state right before it, so it can then launch another ball right into the system
        if (rs.getPower() == currentPower && ascension.getPower() == 1 && feedTimer.milliseconds() > 300){
            ascension.setPower(0);
            blocker.setPosition(1);
        }
    }



    // RUN BELT FUNCTION{
    private void runBelt(double speed) {
        belt.setPower(speed);
        br.setPower(speed);
        bl.setPower(-speed);
    }

    
    // SECONDARY BELT FUNCTION IF I DONT LIKE LEIFS
    public void intakeSystem(){
        if (gamepad2.right_bumper)
            runBelt(1);
        else if (gamepad2.left_bumper)
            runBelt(-1);
        else
            runBelt(0);
    }
    
    
    // DRIVE CODE
    public void drive(){
        lb.setPower(gamepad1.right_stick_x * -speed + speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);
        rb.setPower(gamepad1.right_stick_x * speed + -speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);

        lf.setPower(gamepad1.right_stick_x * -speed + -speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);
        rf.setPower(gamepad1.right_stick_x * speed + speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);

        if (gamepad1.leftBumperWasPressed())
            speed = .45;
        else if (gamepad1.rightBumperWasPressed())
            speed = 1.15;
        else
            speed = .6;
    }

    // Swap launch power function
    public void powerSwap(){
        /*** Here is the mode swap chunk thing that will need alot of testing on the field */

        // When robot is near middle of shooting area
        if (gamepad2.dpad_up) {
            shootMode = "Middle";
            currentPower = middlePower; // .5
        }
        // When robot is at the back of the field in the triangle box
        // Power will need to be high here
        else if (gamepad2.dpad_down) {
            shootMode = "Back";
            currentPower = backPower; // .75
        }
        // When robot is at the tip of the shooting triangle box thingy
        else if (gamepad2.dpad_right) {
            shootMode = "Top";
            currentPower = topPower; // .55
        }
        // When robot is at the bottom of the lebron box thingy
        // Relatively low power here
        else if (gamepad2.dpad_left) {
            shootMode = "Box";
            currentPower = boxPower; // .3
        }
    }

}

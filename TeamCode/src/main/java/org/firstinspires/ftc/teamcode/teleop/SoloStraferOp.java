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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.teamcode.RobotPoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Configurable
@TeleOp(name = "SoloStraferOp")
public class SoloStraferOp extends LinearOpMode {

    // Motors and Servos
    private DcMotor lb;
    private DcMotor rb;
    private DcMotor lf;
    private DcMotor rf;
    private DcMotor ls;
    private DcMotor rs;
    private DcMotor belt;
    private DcMotor elbow;

    private CRServo br;
    private CRServo bl;
    private Servo blocker;

    // Other variables we'll need
    private String shootMode = "Middle";
    private double launchPower;

    private double speed;


    // Holds the power of the motor so it doesn't get messed up with the if statement here
    private double powerSetpoint = middlePower;


    // Will be used to update and change in real time until I find proper values for each individual shootMode
    public static double middlePower = 0.50;  // Default power for Middle D-Pad Up
    public static double backPower = 0.75;    // Default power for Back D-Pad Down
    public static double topPower = 0.55;     // Default power for Top D-Pad Right
    public static double boxPower = 0.40;     // Default power for Box D-Pad Left


    private boolean highSpeedMode = false;
    private boolean halfSpeedMode = false;

    // Taking and storing robot pos from auto

    private Follower follower;

    @Override
    public void runOpMode() {


        // Motors are set to each of its variables
        rf = hardwareMap.get(DcMotor.class,"rf");
        lf = hardwareMap.get(DcMotor.class,"lf");
        rb = hardwareMap.get(DcMotor.class,"rb");
        lb = hardwareMap.get(DcMotor.class,"lb");
        rs = hardwareMap.get(DcMotor.class,"rs");
        ls = hardwareMap.get(DcMotor.class,"ls");
        belt = hardwareMap.get(DcMotor.class, "belt");
        elbow = hardwareMap.get(DcMotor.class, "elbow");


        // Zero power behaviors are set for the motors
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Motor direction is set for straight forward values prolly will have to change later
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        belt.setDirection(DcMotor.Direction.FORWARD);
        elbow.setDirection(DcMotor.Direction.REVERSE);

        // Stopping and resetting encoders in erm the uh other motors
        rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Servo naming
        blocker = hardwareMap.get(Servo.class, "blocker");
        br = hardwareMap.get(CRServo.class, "br");
        bl = hardwareMap.get(CRServo.class, "bl");

        blocker.scaleRange(0.0, 1.0);
        blocker.setPosition(0.54);


        //Variables needed for pedro and updating position on the field
        follower = Constants.createFollower(hardwareMap);

        // Starting pos after teleop
        follower.setStartingPose(new Pose(0, 0, 0));




        // Robot waits for the opmode to be activated
        waitForStart();


        while (opModeIsActive()){

            /*follower = Constants.createFollower(hardwareMap);

            // If the autonomous ran it will use the end position other wise itll be set to (0, 0, 0).
            follower.setStartingPose(RobotPoseStorage.currentPose); */



            lb.setPower(gamepad1.right_stick_x * -speed + speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);
            rb.setPower(gamepad1.right_stick_x * speed + -speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);

            lf.setPower(gamepad1.right_stick_x * -speed + -speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);
            rf.setPower(gamepad1.right_stick_x * speed + speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);



            if (gamepad1.leftStickButtonWasPressed()) {
                halfSpeedMode = !halfSpeedMode; // Flips the state && turns off other mode so errors don't occur
                highSpeedMode = false;
            }

            if (gamepad1.rightStickButtonWasPressed()) {
                highSpeedMode = !highSpeedMode; // Flips the state && turns off other mode so errors don't occur
                halfSpeedMode = false;
            }

            if (halfSpeedMode) {
                speed = 0.45;
                telemetry.addData("Speed of: ", speed);
            } else if (highSpeedMode) {
                speed = 1.15;
                telemetry.addData("Speed of: ", speed);
            } else {
                speed = .6;
            }


            // Funtion for controlling the belt and servo intake stuff
            if (gamepad1.b) {
                belt.setPower(.5);
                br.setPower(1);
                bl.setPower(-1);
            } else if (gamepad1.x) {
                belt.setPower(-.5);
                br.setPower(-1);
                bl.setPower(1);
            } else {
                belt.setPower(0);
                br.setPower(0);
                bl.setPower(0);
            }

            // Function for new servo Gabe added
            if (gamepad1.y)
                blocker.setPosition(0.1);
            else
                blocker.setPosition(.54);

            // Here is the mode swap chunk thing that will need alot of testing on the field

            // When robot is near middle of shooting area
            if (gamepad1.dpad_up){
                shootMode = "Middle";
                powerSetpoint = middlePower /*.5*/;
            }
            // When robot is at the back of the field in the triangle box
            // Power will need to be high here
            else if (gamepad1.dpad_down){
                shootMode = "Back";
                powerSetpoint = backPower /*.75*/;
            }
            // When robot is at the tip of the shooting triangle box thingy
            else if (gamepad1.dpad_right){
                shootMode = "Top";
                powerSetpoint = topPower /*.55*/;
            }
            // When robot is at the bottom of the lebron box thingy
            // Relatively low power here
            else if (gamepad1.dpad_left){
                shootMode = "Box";
                powerSetpoint = boxPower /*.3*/;
            }

            // Stuff for controlling the elbow
            /** ADD SERVO CODE HERE WHEN GABE GETS HIS DESIGN FINISHED */
            if (gamepad1.left_trigger > 0.2){
                elbow.setPower(0.5);
            } else if (gamepad1.right_trigger > 0.2){
                elbow.setPower(-0.5);
            } else {
                elbow.setPower(0);
            }



            // Function that turns the motors on and off
            if (gamepad1.a) {
                // If Y is pressed sets the desired speed
                launchPower = powerSetpoint;
            } else {
                // If Y is not pressed turn the motors off
                launchPower = 0;
            }

            if (gamepad1.left_bumper) {
                // intake system sucks in
                belt.setPower(.5);
                br.setPower(1);
                bl.setPower(-1);
            } else if (gamepad1.right_bumper) {
                belt.setPower(-.5);
                br.setPower(-1);
                bl.setPower(1);
            } else {
                belt.setPower(0);
                br.setPower(0);
                bl.setPower(0);
            }

            // Apply's power to motors
            updateLaunchers();

            // Prints stuff to station
            updatePos();




        }


    }


    // Prints a bunch of values to driverstation
    public void updatePos(){
        follower.update();

        telemetry.addData("Motor goal power: ", launchPower);
        //telemetry.addData("Motor actual power: ", ls.getCurrentPosition());
        telemetry.addData("Current pose: ", shootMode);
        telemetry.addData("X Position", follower.getPose().getX());
        telemetry.addData("Y Position", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();


    }

    // Updates the motor shooting power and makes code easier to read up top
    public void updateLaunchers(){
        rs.setPower(-launchPower);
        ls.setPower(launchPower);
    }





    // Heres a bunch of useful telemetry values that will be printed to the driver station

    /** If this doesn't work prolly have to slap this inside the opmode is active loop */
    /* public void updateValues(){
        // Erm not 100% sure if the telemetry system will like what I do here so if not will be an easy change
        // Gets the battery voltage and prints to driver station
        double currentVoltage = getBatteryVoltage();
        telemetry.addData("Current Voltage", "%.2f V", currentVoltage);
        if (currentVoltage < 12.0) {
            telemetry.addData("WARNING", "Battery voltage is low!");
        }

        telemetry.update();
    }


    // I think im cooking here but it probably wont work :(
    // If this doesnt work prolly have to slap this inside the opmode is active loop
    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        // hardwareMap.voltageSensor is a collection of all voltage sensors like control hub, expansion hub, ect
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    } */


}

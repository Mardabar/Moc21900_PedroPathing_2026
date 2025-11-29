package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class SoloShoot {

    // NAMING MOTORS AND SERVOS
    private DcMotorEx ls;
    private DcMotorEx rs;
    private DcMotorEx elbow;
    private CRServo ascension;
    private Servo blocker;


    // Other Vars
    private String shootMode = "Middle";
    private double launchPower;
    private double speed = 0.65;

    // Holds the power of the motor so it doesn't get messed up with the if statement here
    private double currentPower = middlePower;


    // Will be used to update and change in real time until I find proper values for each individual shootMode
    public static double middlePower = 0.50;  // Default power for Middle D-Pad Up
    public static double backPower = 0.75;    // Default power for Back D-Pad Down
    public static double topPower = 0.55;     // Default power for Top D-Pad Right
    public static double boxPower = 0.40;     // Default power for Box D-Pad Left
    public static double blockerPos = 0;

    public SoloShoot(HardwareMap hardwareMap){

        // Motor init
        rs = hardwareMap.get(DcMotorEx.class, "rs");
        rs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rs.setDirection(DcMotorEx.Direction.REVERSE);
        rs.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        ls = hardwareMap.get(DcMotorEx.class, "ls");
        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ls.setDirection(DcMotorEx.Direction.FORWARD);
        ls.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setDirection(DcMotorEx.Direction.REVERSE);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Servo init
        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.scaleRange(0.02, 0.53);
        blocker.setPosition(1);

        ascension = hardwareMap.get(CRServo.class, "ascension");


    }


    // This updates in teleop
    public void update(Gamepad gamepad){
        // Servos
        if (gamepad.left_bumper){
            ascension.setPower(1);
        }
        else if (gamepad.right_bumper){
            ascension.setPower(-1);
        }
        else {
            ascension.setPower(0);
        }

        if (gamepad.y) {
            blocker.setPosition(0);
        }
        else {
            blocker.setPosition(1);
        }


        // Power swap
        if (gamepad.dpad_up) {
            shootMode = "Middle";
            currentPower = middlePower; // .5
        }
        // When robot is at the back of the field in the triangle box
        // Power will need to be high here
        else if (gamepad.dpad_down) {
            shootMode = "Back";
            currentPower = backPower; // .75
        }
        // When robot is at the tip of the shooting triangle box thingy
        else if (gamepad.dpad_right) {
            shootMode = "Top";
            currentPower = topPower; // .55
        }
        // When robot is at the bottom of the lebron box thingy
        // Relatively low power here
        else if (gamepad.dpad_left) {
            shootMode = "Box";
            currentPower = boxPower; // .3
        }


        if (gamepad.a) {
            // If Y is pressed sets the desired speed
            launchPower = currentPower;
        } else {
            launchPower = 0;
        }


    }


    public void updateLaunchers() {
        rs.setPower(launchPower);
        ls.setPower(launchPower);
    }




}
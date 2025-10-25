package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Configurable
@TeleOp(name = "StraferOpV2")
public class StraferOpV2 extends LinearOpMode {

    // Motors and Servos
    private DcMotor rf;
    private DcMotor lf;
    private DcMotor rb;
    private DcMotor lb;
    private DcMotor rs;
    private DcMotor ls;
    private DcMotor belt;
    private DcMotor elbow;


    // Other variables we'll need
    private String shootMode = null;
    private double launchPower = 0;
    private double speedChange = 1;
    private double speed;

    // This is a temporary variable
    // Will be used to update and change in real time until I find proper values for each individual shootMode
    public static double newLaunchPower;

    private double xPos = 0;
    private double yPos = 0;

    // I think im cooking here but it probably wont work :(
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
    }

    // Get the current battery voltage
    double currentVoltage = getBatteryVoltage();


    @Override
    public void runOpMode(){


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

        // Stopping and resetting encoders in erm the uh other motors
        rs.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ls.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ls.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rs.setPower(launchPower);
        ls.setPower(launchPower);


        // Robot waits for the opmode to be activated
        waitForStart();

        while (opModeIsActive()){

            lb.setPower(gamepad1.right_stick_x * -speed + speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);
            rb.setPower(gamepad1.right_stick_x * speed + -speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);

            lf.setPower(gamepad1.right_stick_x * -speed + -speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);
            rf.setPower(gamepad1.right_stick_x * speed + speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);

            if (gamepad1.left_bumper)
                speed = .5;
            else if (gamepad1.right_bumper)
                speed = 1.5;
            else
                speed = speedChange;


            // Here is the mode swap chunk thing that will need alot of testing on the field

            // When robot is near middle of shooting area
            if (gamepad2.dpad_up){
                shootMode = "Middle";
                launchPower = newLaunchPower /*.5*/;
                updateLaunchers();
            }
            // When robot is at the back of the field in the triangle box
            // Power will need to be high here
            else if (gamepad2.dpad_down){
                shootMode = "Back";
                launchPower = newLaunchPower /*.75*/;
                updateLaunchers();
            }
            // When robot is at the tip of the shooting triangle box thingy
            else if (gamepad2.dpad_right){
                shootMode = "Top";
                launchPower = newLaunchPower /*.55*/;
                updateLaunchers();
            }
            // When robot is at the bottom of the lebron box thingy
            // Relatively low power here
            else if (gamepad2.dpad_left){
                shootMode = "Box";
                launchPower = newLaunchPower /*.3*/;
                updateLaunchers();
            }

        }


    }

    // Updates the motor shooting power and makes code easier to read up top
    public void updateLaunchers(){
        rs.setPower(launchPower);
        ls.setPower(launchPower);

        // Also updates what mode & what the power is prints to the driver station
        telemetry.addData("Motor power: ", launchPower);
        telemetry.addData("Current pose: ", shootMode);
        telemetry.update();
    }

    // Heres a bunch of useful telemetry values that will be printed to the driver station
    public void updateValues(){
        // Erm not 100% sure if the telemetry system will like what I do here so if not will be an easy change
        telemetry.addData("X Pos: ", xPos + "\nY Pos: " + yPos);

        // Gets the battery voltage and prints to driver station
        double currentVoltage = getBatteryVoltage();
        telemetry.addData("Current Voltage", "%.2f V", currentVoltage);
        if (currentVoltage < 12.0) {
            telemetry.addData("WARNING", "Battery voltage is low!");
        }

        telemetry.update();
    }
}

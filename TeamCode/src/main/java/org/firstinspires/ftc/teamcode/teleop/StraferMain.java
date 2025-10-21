package org.firstinspires.ftc.teamcode.teleop;

/*import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;*/
import android.graphics.Camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "StraferMain")
public class StraferMain extends LinearOpMode{
    // MOTORS AND SERVOS

    private DcMotor lb;
    private DcMotor rb;
    private DcMotor lf;
    private DcMotor rf;
    private DcMotor ls;
    private DcMotor rs;
    private DcMotor belt;
    private DcMotor elbow;

    //private Servo blocker;

    // CAMERA

    //private Limelight3A cam;
    //private LLResult camPic;

    private OpenCvCamera cam;
    private Mat camFrame;

    // SPEED AND POSITIONS

    private double mainSpeed = 0.5;
    private double speed;
    private double turnMult = 1.6;
    private double slowMult = 0.4;
    private double fastMult = 1.8;

    private double beltSpeed = 0.3;
    private double elbowSpeed = 0.6;

    private double blockPos = 0;
    private double openPos = 1;

    // SHOOTING CONSTANTS

    private final double OVERSHOOT_MULT = 1.2;
    private final double ANGLE_CONST = 2.08833333;
    private final double MAX_HEIGHT = 1.4;

    private double shootVel;
    private double shootAngle;
    private double elbowTarget;
    private double shootPow;
    private boolean shootPrep;
    private boolean shootReady;


    // OTHER VARS

    private int robotMode = 0;
    private boolean modeSelected = false;

    @Override
    public void runOpMode(){
        // Motors are set to each of its variables
        lb = hardwareMap.get(DcMotor.class,"lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        ls = hardwareMap.get(DcMotor.class, "ls");
        rs = hardwareMap.get(DcMotor.class, "rs");
        belt = hardwareMap.get(DcMotor.class, "belt");
        elbow = hardwareMap.get(DcMotor.class, "elbow");

        // The camera also needs a hardware map
        //cam = hardwareMap.get(Limelight3A.class, "cam");

        // Zero power behaviors are set for the motors
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Motor direction is set for straight forward values
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        ls.setDirection(DcMotor.Direction.FORWARD);
        rs.setDirection(DcMotor.Direction.REVERSE);
        belt.setDirection(DcMotor.Direction.FORWARD);
        elbow.setDirection(DcMotor.Direction.REVERSE);

        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setPower(elbowSpeed);

        speed = mainSpeed;
        shootReady = false;
        shootPrep = false;
        //blocker.scaleRange(blockPos, openPos);

        // The robot waits for the opmode to become active
        waitForStart();
        while (opModeIsActive()){
            if (!modeSelected){
                // This is where the mode is selected and only runs when there is no mode selected
                if (gamepad1.dpad_down){
                    robotMode = 0;
                    elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    setElbowTarget(0);
                    elbow.setPower(elbowSpeed);
                    modeSelected = true;
                }
                else if (gamepad1.dpad_right){
                    robotMode = 1;
                    elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    setElbowTarget(0);
                    elbow.setPower(elbowSpeed);
                    modeSelected = true;
                }
                else if (gamepad1.dpad_up){
                    robotMode = 2;
                    elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    setElbowTarget(0);
                    elbow.setPower(elbowSpeed);
                    modeSelected = true;
                }
                else if (gamepad1.right_stick_button){
                    robotMode = 3;
                    elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    elbow.setPower(0);
                    modeSelected = true;
                }
            } else if (modeSelected) {
                // This only runs when a mode is selected
                switch (robotMode) {
                    case 0: // Regular mode, the robot has basic presets and main controls given to the drivers

                        // MAIN DRIVER CONTROLS

                        lb.setPower(turnMult * gamepad1.right_stick_x * -speed + speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);
                        rb.setPower(turnMult * gamepad1.right_stick_x * speed + -speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);

                        lf.setPower(turnMult * gamepad1.right_stick_x * -speed + -speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);
                        rf.setPower(turnMult * gamepad1.right_stick_x * speed + speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);

                        if (gamepad1.left_bumper)
                            speed = mainSpeed * slowMult;
                        else if (gamepad1.right_bumper)
                            speed = mainSpeed * fastMult;
                        else
                            speed = mainSpeed;

                        // ACCESSORY DRIVER CONTROLS

                        if (gamepad2.right_bumper){
                            belt.setPower(beltSpeed);
                            //blocker.setPosition(1);
                        }
                        else if (gamepad2.left_bumper){
                            belt.setPower(-beltSpeed);
                            //blocker.setPosition(1);
                        }
                        else if (gamepad2.a && !shootPrep && !shootReady){
                            setShootPos(30, 30, 135, 135);
                            //blocker.setPosition(0);
                            shootPrep = true;
                        }
                        else{
                            belt.setPower(0);
                            //blocker.setPosition(0);
                        }

                        if (gamepad2.a && shootReady){
                            shootPow = velToPow(shootVel);
                            //setElbowTarget(shootAngle);
                            telemetry.addData("Launch Motor Power", velToPow(shootVel));
                            telemetry.addData("Elbow Angle", shootAngle);
                            telemetry.update();

                            ls.setPower(shootPow);
                            rs.setPower(shootPow);
                        }
                        else if (shootReady){
                            ls.setPower(0);
                            rs.setPower(0);
                            shootReady = false;
                        }


                        break;

                    case 1: // Auto mode, the robot has very complex presets with minimal control to the drivers


                        break;

                    case 2: // Single player mode, only one controller is required

                        // MAIN DRIVER CONTROLS

                        lb.setPower(turnMult * gamepad1.right_stick_x * speed + speed * -gamepad1.left_stick_x + speed * gamepad1.left_stick_y);
                        rb.setPower(turnMult * gamepad1.right_stick_x * speed + speed * -gamepad1.left_stick_x + speed * gamepad1.left_stick_y);

                        lf.setPower(turnMult * gamepad1.right_stick_x * speed + speed * -gamepad1.left_stick_x + speed * gamepad1.left_stick_y);
                        rf.setPower(turnMult * gamepad1.right_stick_x * speed + speed * -gamepad1.left_stick_x + speed * gamepad1.left_stick_y);

                        if (gamepad1.left_bumper)
                            speed = mainSpeed * slowMult;
                        else if (gamepad1.right_bumper)
                            speed = mainSpeed * fastMult;
                        else
                            speed = mainSpeed;

                        if (gamepad1.left_trigger > 0.2){
                            belt.setPower(beltSpeed);
                            //blocker.setPosition(1);
                        }
                        else if (gamepad1.right_trigger > 0.2){
                            belt.setPower(-beltSpeed);
                            //blocker.setPosition(1);
                        }
                        else if (gamepad1.a && !shootPrep && !shootReady){
                            setShootPos(0, 0, 0, 0);
                            //blocker.setPosition(0);
                            shootPrep = true;
                        }
                        else{
                            belt.setPower(0);
                            //blocker.setPosition(0);
                        }

                        if (gamepad1.a && shootReady){
                            shootPow = velToPow(shootVel);
                            elbowTarget = angleToEncoder(shootAngle);

                            ls.setPower(shootPow);
                            rs.setPower(shootPow);
                        }
                        else
                            shootReady = false;

                        break;

                    case 3: // Free mode, the robot has zero presets and the drivers have full control
                        lb.setPower(turnMult * gamepad1.right_stick_x * -speed + speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);
                        rb.setPower(turnMult * gamepad1.right_stick_x * speed + -speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);

                        lf.setPower(turnMult * gamepad1.right_stick_x * -speed + -speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);
                        rf.setPower(turnMult * gamepad1.right_stick_x * speed + speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);

                        if (gamepad1.left_bumper)
                            speed = mainSpeed * slowMult;
                        else if (gamepad1.right_bumper)
                            speed = mainSpeed * fastMult;
                        else
                            speed = mainSpeed;

                        if (gamepad1.b){
                            belt.setPower(beltSpeed);
                            //blocker.setPosition(1);
                        }
                        else if (gamepad1.x){
                            belt.setPower(-beltSpeed);
                            //blocker.setPosition(1);
                        }
                        else
                            belt.setPower(0);

                        if (gamepad1.a){
                            ls.setPower(0.6);
                            rs.setPower(0.6);
                        }
                        else{
                            ls.setPower(0);
                            rs.setPower(0);
                        }

                        if (gamepad1.left_trigger > 0.2)
                            elbow.setPower(0.5);
                        else if (gamepad1.right_trigger > 0.2)
                            elbow.setPower(-0.5);
                        else
                            elbow.setPower(0);
                }

                if (gamepad1.dpad_left) {
                    modeSelected = false;
                    break;
                }
            }
        }
    }

    // ACCESSORY METHODS

    // This method sets the speed of the shooter motors and the angle of the shooting position
    private void setShootPos(double ix, double iy, double fx, double fy){
        // Temporary vars are created to set the actual vars at the end of the method
        /* dist is the total distance the ball will travel until it hits the ground
           It's divided by 40 to turn the field units into meters
           Then, it's multiplied by 1.3 because the ball will hit the goal first, so using the
           equation, it'll be about 1 meter high (the height of the goal) when it hit our requested distance
         */
        double dist = (Math.sqrt(Math.pow(fx - ix, 2) + Math.pow(fy - iy, 2)) / 40) * 1.3;
        double angle = 0;
        double speed = 0;

        // We loop the calculations until the equation equals zero
        while (!(shootPosCalc(dist, speed, angle) > -0.2 && shootPosCalc(dist, speed, angle) < 0.2)){
            angle += 0.1;
            speed = angleToVel(angle);
        }

        // The global shooter velocity is now set to the speed calulated in this method, as well as the global angle set to the calulated angle
        shootVel = speed * OVERSHOOT_MULT;
        shootAngle = angle * OVERSHOOT_MULT;
        shootPrep = false;
        shootReady = true;
    }

    // This is the function used to determine the velocity and angle of the launcher
    private double shootPosCalc(double tempDist, double speed, double angle){
        double calc = tempDist * Math.tan(angle) - (9.8 / (2 * Math.pow(speed, 2) * Math.pow(Math.cos(angle), 2))) * Math.pow(tempDist, 2);
        return calc;
    }

    // This function translates angle to velocity using the already set maximum height
    private double angleToVel(double angle){
        double newVel = Math.sqrt((MAX_HEIGHT * 19.6) / Math.pow(Math.sin(angle), 2));
        return newVel;
    }

    // This function translates velocity to motor power specifically for 6000 RPM motors combined with 72 mm Gecko Wheels
    private double velToPow(double vel){
        double newVel = vel / (7.2 * Math.PI);
        return newVel;
    }

    // This function translates an angle in degrees to an encoder value on 223 RPM motors
    private double angleToEncoder(double angle){
        double newAngle = angle * ANGLE_CONST;
        return newAngle;
    }

    private void setElbowTarget(double angle){
        elbow.setTargetPosition((int) angleToEncoder(angle));
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
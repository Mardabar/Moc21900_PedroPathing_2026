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

    // SPEED AND POSITIONS

    private double mainSpeed = 0.5;
    private double speed;
    private double turnMult = 1.6;
    private double slowMult = 0.4;
    private double fastMult = 1.8;

    private double beltSpeed = 0.3;

    private double blockPos = 0;
    private double openPos = 1;

    // SHOOTING CONSTANTS

    private final double VEL_CONST = 0.025;
    private final double ANGLE_CONST = 0.5;
    private final double MAX_HEIGHT = 110;

    private double shootVel;
    private double shootAngle;
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
        ls.setDirection(DcMotor.Direction.REVERSE);
        rs.setDirection(DcMotor.Direction.FORWARD);
        belt.setDirection(DcMotor.Direction.FORWARD);
        elbow.setDirection(DcMotor.Direction.FORWARD);

        belt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        belt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        belt.setPower(beltSpeed);

        speed = mainSpeed;
        shootReady = false;
        shootPrep = false;
        //blocker.scaleRange(blockPos, openPos);
        //cam.pipelineSwitch(0);

        // The robot waits for the opmode to become active
        waitForStart();
        while (opModeIsActive()){
            if (!modeSelected){
                // This is where the mode is selected and only runs when there is no mode selected
                if (gamepad1.dpad_down){
                    robotMode = 0;
                    modeSelected = true;
                }
                else if (gamepad1.dpad_right){
                    robotMode = 1;
                    modeSelected = true;
                }
                else if (gamepad1.dpad_up){
                    robotMode = 2;
                    modeSelected = true;
                }
            } else if (modeSelected) {
                // This only runs when a mode is selected
                switch (robotMode) {
                    case 0: // Regular mode, the robot has basic presets and main controls given to the drivers

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

                        // ACCESSORY DRIVER CONTROLS

                        if (gamepad2.right_bumper){
                            moveBelt(1);
                            //blocker.setPosition(1);
                        }
                        else if (gamepad2.left_bumper){
                            moveBelt(-1);
                            //blocker.setPosition(1);
                        }
                        else if (gamepad2.a && !shootPrep && !shootReady){
                            setShootPos(0, 0, 0, 0);
                            //blocker.setPosition(0);
                            shootPrep = true;
                        }
                        else
                            //blocker.setPosition(0);

                        if (gamepad2.a && shootReady){
                            shootPow = velToPow(shootVel);
                            ls.setPower(shootPow);
                            rs.setPower(shootPow);
                        }
                        else
                            shootReady = false;

                        break;

                    case 1: // Auto mode, the robot has very complex presets with minimal control to the drivers


                        break;

                    case 2: // Free mode, the robot has zero presets and the drivers have full control

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
                            moveBelt(1);
                            //blocker.setPosition(1);
                        }
                        else if (gamepad1.a){
                            moveBelt(-1);
                            //blocker.setPosition(1);
                        }
                        else if (gamepad1.right_trigger > 0.2 && !shootPrep && !shootReady){
                            setShootPos(0, 0, 0, 0);
                            //blocker.setPosition(0);
                            shootPrep = true;
                        }
                        else{
                            ls.setPower(0);
                            rs.setPower(0);
                            //blocker.setPosition(0);
                        }

                        if (gamepad1.right_trigger > 0.2 && shootReady){
                            shootPow = velToPow(shootVel);
                            ls.setPower(shootPow);
                            rs.setPower(shootPow);
                        }
                        else
                            shootReady = false;

                        break;
                }

                if (gamepad1.dpad_left) {
                    modeSelected = false;
                    break;
                }
            }
        }
    }

    // ACCESSORY METHODS

    private void moveBelt(int direction){
        belt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (belt.getPower() < beltSpeed)
            belt.setTargetPosition(belt.getCurrentPosition() + (1 * direction));
        belt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // This method sets the speed of the shooter motors and the angle of the shooting position
    private void setShootPos(double ix, double iy, double fx, double fy){
        // Temporary vars are created to set the actual vars at the end of the method
        // dist is the total distance the ball will travel and halfDist is half the distance, therefore the max height the ball will reach
        double dist = 0.5 * (Math.abs(fx - ix) * Math.abs(fy - iy));
        double angle = elbow.getCurrentPosition() / ANGLE_CONST;
        double speed = 1;
        double halfDist = dist / 2;

        // Now the while loop starts, which will constantly adjust the speed and the angle until the right path is calculated
        while (shootPosCalc(halfDist, speed, angle) < MAX_HEIGHT - 0.3 && shootPosCalc(halfDist, speed, angle) > MAX_HEIGHT + 0.3 &&
                shootPosCalc(dist, speed, angle) < -0.2 && shootPosCalc(dist, speed, angle) > 0.2){
            if (shootPosCalc(dist, speed, angle) < -0.2){
                speed += 0.2;
            }
            else if (shootPosCalc(dist, speed, angle) > 0.2){
                speed -= 0.2;
            }

            if (shootPosCalc(halfDist, speed, angle) < MAX_HEIGHT - 0.3){
                angle += 0.4;
            }
            else if (shootPosCalc(halfDist, speed, angle) > MAX_HEIGHT + 0.3) {
                angle -= 0.4;
            }
        }

        // The global shooter velocity is now set to the speed calulated in this method, as well as the global angle set to the calulated angle
        shootVel = speed;
        shootAngle = angle;
        shootPrep = false;
        shootReady = true;
    }

    // This is the function used to determine the heights for the adjustments done in the setShootPos method
    private double shootPosCalc(double tempDist, double speed, double angle){
        double calc = tempDist * Math.tan(angle) - 9.8 / (2 * Math.pow(speed, 2) * Math.pow(Math.cos(angle), 2)) * Math.pow(tempDist, 2);
        return calc;
    }

    // This function translates velocity to motor power specifically for 6000 RPM motors combined with 72 mm Gecko Wheels
    private double velToPow(double vel){
        double newVel = vel / (7.2 * Math.PI);
        return newVel;
    }
}
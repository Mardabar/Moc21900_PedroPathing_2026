package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import subsystems.ShootSystem;

@TeleOp(name = "StraferMain")
public class StraferMain extends LinearOpMode{
    // MOTORS AND SERVOS

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

    // CAMERA

    private Limelight3A cam;
    private LLResult camPic;

    // SPEED AND POSITIONS

    private double mainSpeed = 0.5;
    private double speed;
    private double turnMult = 1.6;
    private double slowMult = 0.4;
    private double fastMult = 1.8;

    private double lStickPosX;
    private double lStickPosY;
    private double snapPos = 0.1;

    private double beltSpeed = 1;
    private double elbowSpeed = 0.4;

    private double openPos = 0.53;
    private double feedPos = 0.02;
    private ElapsedTime feedTimer;
    private double feedDur = 200;
    private double ascendDur = 900;
    private double retDur = 600;
    private int feeding;

    // SHOOTING VARS

    private final double OVERSHOOT_VEL_MULT = 1.68;
    private final double OVERSHOOT_ANG_MULT = 1;
    private final double ANGLE_CONST = 2.08833333;
    private final int ELBOW_GEAR_RATIO = 4;
    private final double MAX_HEIGHT = 1.4;

    private final double p = 0.021, i = 0.00001, d = 0.00011;
    private double lastError;
    private double iSum;

    private double shootVel;
    private double shootAngle;
    private double shootRot;

    private boolean shootPrep;
    private boolean shootReady;
    private boolean flysSpeedy;


    // OTHER VARS

    private ElapsedTime blockTimer;
    private int robotMode = 0;
    private int color = 0;
    private boolean modeSelected = false;

    @Override
    public void runOpMode(){
        // Motors are set to each of its variables
        lb = hardwareMap.get(DcMotor.class,"lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        ls = hardwareMap.get(DcMotorEx.class, "ls");
        rs = hardwareMap.get(DcMotorEx.class, "rs");
        belt = hardwareMap.get(DcMotor.class, "belt");
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

        // Motor direction is set for straight forward values
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        ls.setDirection(DcMotorEx.Direction.FORWARD);
        rs.setDirection(DcMotorEx.Direction.REVERSE);
        belt.setDirection(DcMotor.Direction.FORWARD);
        elbow.setDirection(DcMotorEx.Direction.REVERSE);

        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setElbowTarget(0);
        elbow.setPower(elbowSpeed);

        ls.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rs.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        ls.setMotorDisable();
        rs.setMotorDisable();

        ascension = hardwareMap.get(CRServo.class, "ascension");
        blocker = hardwareMap.get(Servo.class, "blocker");
        br = hardwareMap.get(CRServo.class, "br");
        bl = hardwareMap.get(CRServo.class, "bl");

        // Camera

        cam = hardwareMap.get(Limelight3A.class, "limelight");
        cam.pipelineSwitch(0);

        // Other Vars

        speed = mainSpeed;
        shootReady = false;
        shootPrep = false;

        blockTimer = new ElapsedTime();
        feedTimer = new ElapsedTime();
        blocker.scaleRange(feedPos, openPos);
        blocker.setPosition(1);

        cam.start();

        // The robot waits for the opmode to become active
        waitForStart();
        while (opModeIsActive()){
            if (!modeSelected){
                if (gamepad1.left_stick_button)
                    color = 1;
                else
                    color = 0;
                // This is where the mode is selected and only runs when there is no mode selected
                if (gamepad1.dpad_down){
                    if (robotMode == 3){
                        ls.setMotorDisable();
                        rs.setMotorDisable();
                        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        setElbowTarget(0);
                        elbow.setPower(elbowSpeed);
                    }
                    robotMode = 0;
                    modeSelected = true;
                }
                else if (gamepad1.dpad_right){
                    if (robotMode == 3){
                        ls.setMotorDisable();
                        rs.setMotorDisable();
                        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        setElbowTarget(0);
                        elbow.setPower(elbowSpeed);
                    }
                    robotMode = 1;
                    modeSelected = true;
                }
                else if (gamepad1.dpad_up){
                    if (robotMode == 3){
                        ls.setMotorDisable();
                        rs.setMotorDisable();
                        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        setElbowTarget(0);
                        elbow.setPower(elbowSpeed);
                    }
                    robotMode = 2;
                    modeSelected = true;
                }
                else if (gamepad1.right_stick_button){
                    robotMode = 3;
                    ls.setMotorEnable();
                    rs.setMotorEnable();
                    elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    elbow.setPower(0);
                    modeSelected = true;
                }
            } else if (modeSelected) {
                // This only runs when a mode is selected
                switch (robotMode) {
                    case 0: // Regular mode, the robot has basic presets and main controls given to the drivers

                        // MAIN DRIVER CONTROLS

                        if (!shootReady) {
                            // This block allows the movement to snap in one direction if the driver seems to want to go in just one direction
                            if (Math.abs(gamepad1.left_stick_y) < snapPos && Math.abs(gamepad1.left_stick_x) > snapPos) {
                                lStickPosX = gamepad1.left_stick_x;
                                lStickPosY = 0;
                            } else if (Math.abs(gamepad1.left_stick_y) > snapPos && Math.abs(gamepad1.left_stick_x) < snapPos) {
                                lStickPosY = gamepad1.left_stick_y;
                                lStickPosX = 0;
                            } else if (Math.abs(gamepad1.left_stick_y) < snapPos * 3 && Math.abs(gamepad1.left_stick_x) < snapPos * 3) {
                                lStickPosY = 0;
                                lStickPosX = 0;
                            } else {
                                lStickPosY = gamepad1.left_stick_y;
                                lStickPosX = gamepad1.left_stick_x;
                            }

                            // The main strafer movement of the robot, changed for the first time in years
                            lb.setPower((turnMult * gamepad1.right_stick_x * -speed) + (speed * lStickPosX) + (speed * lStickPosY));
                            rb.setPower((turnMult * gamepad1.right_stick_x * speed) + (-speed * lStickPosX) + (speed * lStickPosY));

                            lf.setPower((turnMult * gamepad1.right_stick_x * -speed) + (-speed * lStickPosX) + (speed * lStickPosY));
                            rf.setPower((turnMult * gamepad1.right_stick_x * speed) + (speed * lStickPosX) + (speed * lStickPosY));

                            // Changes the current speed of the robot
                            if (gamepad1.left_trigger > 0.2)
                                speed = mainSpeed * slowMult;
                            else if (gamepad1.right_trigger > 0.2)
                                speed = mainSpeed * fastMult;
                            else
                                speed = mainSpeed;

                            // ACCESSORY DRIVER CONTROLS
                            if (gamepad2.right_bumper)
                                runBelt(beltSpeed);
                            else if (gamepad2.left_bumper)
                                runBelt(-beltSpeed);
                            else if (gamepad2.a && !shootPrep) {
                                camPic = cam.getLatestResult();
                                if (camPic.isValid())
                                    initShooting(camPic);
                            } else
                                runBelt(0);

                            // Feeds the launcher a ball
                            if (gamepad2.y){
                                if (feeding == 0) {
                                    blocker.setPosition(0);
                                    feedTimer.reset();
                                    feeding = 1;
                                }
                                else if (feeding == 1 && feedTimer.milliseconds() > feedDur){
                                    blocker.setPosition(1);
                                    feeding = 2;
                                }
                            }
                            else
                                feeding = 0;
                        }

                        // Shoots the ball when conditions are met
                        if (gamepad2.a && shootReady){
                            shoot();
                        }
                        else if (shootReady){
                            resetBack();
                        }

                        break;

                    case 1: // Auto mode, the robot has very complex presets with minimal control to the drivers


                        break;

                    case 2: // Single player mode, only one controller is required

                        // MAIN DRIVER CONTROLS

                        if (!shootReady) {
                            // This block allows the movement to snap in one direction if the driver seems to want to go in just one direction
                            if (Math.abs(gamepad1.left_stick_y) < snapPos && Math.abs(gamepad1.left_stick_x) > snapPos) {
                                lStickPosX = gamepad1.left_stick_x;
                                lStickPosY = 0;
                            } else if (Math.abs(gamepad1.left_stick_y) > snapPos && Math.abs(gamepad1.left_stick_x) < snapPos) {
                                lStickPosY = gamepad1.left_stick_y;
                                lStickPosX = 0;
                            } else if (Math.abs(gamepad1.left_stick_y) < snapPos * 3 && Math.abs(gamepad1.left_stick_x) < snapPos * 3) {
                                lStickPosY = 0;
                                lStickPosX = 0;
                            } else {
                                lStickPosY = gamepad1.left_stick_y;
                                lStickPosX = gamepad1.left_stick_x;
                            }

                            // The main strafer movement of the robot, changed for the first time in years
                            lb.setPower((turnMult * gamepad1.right_stick_x * -speed) + (speed * lStickPosX) + (speed * lStickPosY));
                            rb.setPower((turnMult * gamepad1.right_stick_x * speed) + (-speed * lStickPosX) + (speed * lStickPosY));

                            lf.setPower((turnMult * gamepad1.right_stick_x * -speed) + (-speed * lStickPosX) + (speed * lStickPosY));
                            rf.setPower((turnMult * gamepad1.right_stick_x * speed) + (speed * lStickPosX) + (speed * lStickPosY));

                            // Changes the current speed of the robot
                            if (gamepad1.left_trigger > 0.2)
                                speed = mainSpeed * slowMult;
                            else if (gamepad1.right_trigger > 0.2)
                                speed = mainSpeed * fastMult;
                            else
                                speed = mainSpeed;

                            // ACCESSORY DRIVER CONTROLS

                            if (gamepad1.x)
                                runBelt(beltSpeed);
                            else if (gamepad1.b)
                                runBelt(-beltSpeed);
                            else if (gamepad1.a && !shootPrep) {
                                camPic = cam.getLatestResult();
                                if (camPic.isValid())
                                    initShooting(camPic);
                            } else
                                runBelt(0);

                            // Feeds the launcher a ball
                            if (gamepad1.left_stick_button){
                                if (feeding == 0) {
                                    blocker.setPosition(0);
                                    feedTimer.reset();
                                    feeding = 1;
                                }
                                else if (feeding == 1 && feedTimer.milliseconds() > feedDur){
                                    blocker.setPosition(1);
                                    feeding = 2;
                                }
                            }
                            else
                                feeding = 0;
                        }

                        // Shoots the ball when conditions are met
                        if (gamepad1.a && shootReady)
                            shoot();
                        else if (shootReady)
                            resetBack();

                        break;

                    case 3: // Free mode, the robot has zero presets and the drivers have full control
                        lb.setPower(turnMult * gamepad1.right_stick_x * -speed + speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);
                        rb.setPower(turnMult * gamepad1.right_stick_x * speed + -speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);

                        lf.setPower(turnMult * gamepad1.right_stick_x * -speed + -speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);
                        rf.setPower(turnMult * gamepad1.right_stick_x * speed + speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);

                        if (gamepad1.b){
                            runBelt(beltSpeed);
                        }
                        else if (gamepad1.x){
                            runBelt(-beltSpeed);
                        }
                        else
                            runBelt(0);

                        if (gamepad1.a){
                            ascension.setPower(1);
                        }
                        else if (gamepad2.b){
                            ascension.setPower(-1);
                        }
                        else {
                            ascension.setPower(0);
                        }

                        if (gamepad2.x){
                            ls.setPower(-0.4);
                            rs.setPower(-0.4);
                        }
                        else {
                            ls.setVelocity(gamepad1.left_trigger * 2800);
                            rs.setVelocity(gamepad1.right_trigger * 2800);
                        }
                        telemetry.addData("Flywheel Speed", "ls: " + Math.round(ls.getPower()) +
                                "    rs: " + Math.round(rs.getPower()));

                        if (gamepad1.left_bumper) {
                            elbow.setPower(0.3);
                        }
                        else if (gamepad1.right_bumper) {
                            elbow.setPower(-0.3);
                        }
                        else {
                            elbow.setPower(0);
                        }
                        telemetry.addData("Elbow Position", elbow.getCurrentPosition());

                        if (gamepad1.y) {
                            blocker.setPosition(0);
                        }
                        else {
                            blocker.setPosition(1);
                        }

                        telemetry.update();
                        break;
                }

                if (gamepad1.dpad_left) {
                    modeSelected = false;
                }
            }
        }
    }

    // ACCESSORY METHODS

    private void initShooting(LLResult pic){
        for (LLResultTypes.FiducialResult res : pic.getFiducialResults()) {
            int id = res.getFiducialId();
            if (id == 20 || id == 24) {
                double angle = 25.2 + res.getTargetYDegrees();
                double tagDist = (0.646 / Math.tan(Math.toRadians(angle)));

                setShootPos(tagDist);
                feeding = 0;
                blocker.setPosition(1);
                blockTimer.reset();

                ls.setMotorEnable();
                rs.setMotorEnable();

                iSum = 0;
                shootPrep = true;
            }
        }
    }

    private void shoot(){
        shootRot = velToRot(shootVel);
        setElbowTarget(angleToEncoder(shootAngle));

        telemetry.addData("Velocity", shootVel);
        telemetry.addData("Encoder Angle", shootAngle);
        telemetry.update();

        // This section uses PID to control the angle the robot is facing towards the april tag
        if (gamepad1.left_bumper){
            lb.setPower(turnMult * gamepad1.right_stick_x * -speed);
            rb.setPower(turnMult * gamepad1.right_stick_x * speed);

            lf.setPower(turnMult * gamepad1.right_stick_x * -speed);
            rf.setPower(turnMult * gamepad1.right_stick_x * speed);
        }
        else {
            camPic = cam.getLatestResult();
            for (LLResultTypes.FiducialResult res : camPic.getFiducialResults()) {
                int id = res.getFiducialId();
                if (id == 20 || id == 24) {
                    double error = res.getTargetXDegrees();
                    iSum += error;
                    double derError = lastError - error;

                    lb.setPower(-((error * p) + (iSum * i) + (derError * d)));
                    rb.setPower((error * p) + (iSum * i) + (derError * d));
                    lf.setPower(-((error * p) + (iSum * i) + (derError * d)));
                    rf.setPower((error * p) + (iSum * i) + (derError * d));

                    lastError = error;
                }
            }
        }

        if (!flysSpeedy && ls.getVelocity() >= shootRot && rs.getVelocity() >= shootRot)
            flysSpeedy = true;
        if (flysSpeedy)
            feedLauncher();

        ls.setVelocity(shootRot);
        rs.setVelocity(shootRot);
    }

    private void resetBack(){
        feeding = 1;
        blocker.setPosition(1);
        ascension.setPower(0);
        runBelt(0);
        ls.setPower(0);
        rs.setPower(0);
        ls.setMotorDisable();
        rs.setMotorDisable();

        flysSpeedy = false;
        shootPrep = false;
        shootReady = false;
    }

    public void setShootPos(double dist){
        /* dist is the total distance the ball will travel until it hits the ground
           it's multiplied by 1.3 because the ball will hit the goal first, so using the
           equation, it'll be about 1 meter high (the height of the goal) when it hit our requested distance
         */
        dist *= 1.3;

        // The angle and velocity are both calculated using the distance we found
        shootAngle = ((distToAngle(dist) * OVERSHOOT_ANG_MULT) - 53.5);
        shootVel = angleToVel(distToAngle(dist)) * OVERSHOOT_VEL_MULT;

        shootPrep = false;
        shootReady = true;
    }

    public double distToAngle(double dist){
        return Math.toDegrees(Math.atan(54.88 / (9.8 * dist)));
    }

    // This function translates angle to velocity using the already set maximum height
    public double angleToVel(double angle){
        return Math.sqrt((MAX_HEIGHT * 19.6) / Math.pow(Math.sin(Math.toRadians(angle)), 2));
    }

    // This function translates velocity to motor power specifically for 6000 RPM motors combined with 72 mm Gecko Wheels
    public double velToRot(double vel){
        return (vel / (7.2 * Math.PI)) * 2800;
    }

    // This function translates an angle in degrees to an encoder value on 223 RPM motors
    public double angleToEncoder(double angle){
        return angle * ANGLE_CONST * ELBOW_GEAR_RATIO;
    }

    private void setElbowTarget(double angle){
        elbow.setTargetPosition((int) angle);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void runBelt(double speed) {
        belt.setPower(speed);
        br.setPower(speed);
        bl.setPower(-speed);
    }

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
            runBelt(-beltSpeed);
        }
        else {
            if (ls.getVelocity() >= shootRot - 30 && rs.getVelocity() >= shootRot - 30) {
                if (feeding == 2)
                    feeding = 0;
                else
                    feeding++;
            }
            feedTimer.reset();
        }
    }
}
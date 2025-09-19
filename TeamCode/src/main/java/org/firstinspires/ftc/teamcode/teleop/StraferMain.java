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

    private Servo blocker;

    // CAMERA

    private Limelight3A cam;
    private LLResult camPic;

    // SPEED AND POSITIONS

    private double mainSpeed = 0.5;
    private double speed;
    private double turnMult = 1.6;
    private double slowMult = 0.4;
    private double fastMult = 1.8;

    private double shootSpeed = 1;
    private double beltSpeed = 0.3;

    private double blockPos = 0;
    private double openPos = 1;

    // OTHER VARS

    private int robotMode = 0;
    private boolean modeSelected = false;
    private boolean initSwitchZero = false;

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

        // The camera also needs a hardware map
        cam = hardwareMap.get(Limelight3A.class, "cam");

        // Zero power behaviors are set for the motors
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Motor direction is set for straight forward values
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        ls.setDirection(DcMotor.Direction.REVERSE);
        rs.setDirection(DcMotor.Direction.FORWARD);
        belt.setDirection(DcMotor.Direction.FORWARD);

        belt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        belt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        belt.setPower(beltSpeed);

        speed = mainSpeed;
        blocker.scaleRange(blockPos, openPos);
        cam.pipelineSwitch(0);

        // The robot waits for the opmode to become active
        waitForStart();
        while (opModeIsActive()){
            if (!modeSelected){
                // This is where the mode is selected and only runs when there is no mode selected
                if (gamepad1.dpad_down){
                    robotMode = 0;
                    initSwitchZero = true;
                    modeSelected = true;
                }
                else if (gamepad1.dpad_up){
                    robotMode = 1;
                    modeSelected = true;
                }
                else if (gamepad1.dpad_right){
                    robotMode = 2;
                    modeSelected = true;
                }
            } else if (modeSelected) {
                // This only runs when a mode is selected
                switch (robotMode) {
                    case 0: // Regular mode, the robot has basic presets and main controls given to the drivers

                        // MAIN DRIVER CONTROLS

                        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
                        // We negate this value so that the topmost position corresponds to maximum forward power.
                        lb.setPower(turnMult * gamepad1.right_stick_x * speed + speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);
                        rb.setPower(turnMult * gamepad1.right_stick_x * speed + speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);
                        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
                        // We negate this value so that the topmost position corresponds to maximum forward power.
                        lf.setPower(turnMult * gamepad1.right_stick_x * speed + speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);
                        rf.setPower(turnMult * gamepad1.right_stick_x * speed + speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);

                        if (gamepad1.left_bumper)
                            speed = mainSpeed * slowMult;
                        else if (gamepad1.right_bumper)
                            speed = mainSpeed * fastMult;
                        else
                            speed = mainSpeed;

                        // ACCESSORY DRIVER CONTROLS

                        if (gamepad2.right_bumper){
                            moveBelt(1);
                            blocker.setPosition(1);
                        }
                        else if (gamepad2.left_bumper){
                            moveBelt(-1);
                            blocker.setPosition(1);
                        }
                        else if (gamepad2.a){
                            moveBelt(1);
                            ls.setPower(shootSpeed);
                            rs.setPower(shootSpeed);
                            blocker.setPosition(0);
                        }
                        else{
                            ls.setPower(0);
                            rs.setPower(0);
                            blocker.setPosition(0);
                        }

                    case 1: // Auto mode, the robot has very complex presets with minimal control to the drivers
                        if (gamepad1.dpad_down || gamepad2.dpad_down)
                            robotMode = 0;

                    case 2: // Free mode, the robot has zero presets and the drivers have full control
                        if (gamepad1.dpad_down || gamepad2.dpad_down)
                            robotMode = 0;
                }

                if (gamepad1.dpad_left) // Starts mode selection to switch modes
                    modeSelected = false;
            }
        }
    }

    // ACCESSORY METHODS

    private void moveBelt(int direction){
        if (belt.getPower() < beltSpeed)
            belt.setTargetPosition(belt.getCurrentPosition() + (1 * direction));
        belt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
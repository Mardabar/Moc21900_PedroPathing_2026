package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SoloShoot;


@Configurable
@TeleOp(name = "SoloStrafer")
public class SoloStrafer extends LinearOpMode {

    // Subsystem stuff
    private Intake intake;
    private SoloShoot soloShoot;


    // Motors and Servos
    private DcMotor lb;
    private DcMotor rb;
    private DcMotor lf;
    private DcMotor rf;

    private double speed = 0.65;








    @Override
    public void runOpMode() {

        // subsystem stuff
        soloShoot = new SoloShoot(hardwareMap);
        intake = new Intake(hardwareMap);


        // Motors are set to each of its variables
        rf = hardwareMap.get(DcMotor.class, "rf");
        lf = hardwareMap.get(DcMotor.class, "lf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        lb = hardwareMap.get(DcMotor.class, "lb");



        // Zero power behaviors are set for the motors
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Motor direction is set for straight forward values prolly will have to change later
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);





        // Robot waits for the opmode to be activated
        waitForStart();


        while (opModeIsActive()) {

            soloShoot.update(gamepad1);
            intake.update(gamepad1);


            drive();
        }


    }


    // DRIVE CODE
    public void drive(){
        lb.setPower(gamepad1.right_stick_x * -speed + speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);
        rb.setPower(gamepad1.right_stick_x * speed + -speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);

        lf.setPower(gamepad1.right_stick_x * -speed + -speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);
        rf.setPower(gamepad1.right_stick_x * speed + speed * gamepad1.left_stick_x + speed * gamepad1.left_stick_y);

        if (gamepad1.leftStickButtonWasPressed())
            speed = .45;
        else if (gamepad1.rightStickButtonWasPressed())
            speed = 1.15;
        else
            speed = .65;
    }

}

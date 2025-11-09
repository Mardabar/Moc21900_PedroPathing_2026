/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotBaseCustomOdo {

    // Motors
    private DcMotor lb;
    private DcMotor rb;
    private DcMotor lf;
    private DcMotor rf;


    // Encoders **Is the odo pods and w this example it uses 3**
    private DcMotor encoderLeft;
    private DcMotor encoderRight;
    private DcMotor encoderAux;

    private HardwareMap hardwareMap;

    public RobotBaseCustomOdo(HardwareMap aHardwareMap){
        hardwareMap = aHardwareMap;

        lb = hardwareMap.get(DcMotor.class,"lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");

        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);

        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // The encoders are set to shadow the real motors, not sure why
        encoderLeft = lb;
        encoderRight = rb;
        encoderAux = rf;

    }

    // Here is where the math magic crap happens
    */
/*** THESE ARE ASSUMPTIONS **//*

    final static double L = 16;         // This is the distance between encoder 1  and 2 are in cm
    final static double B = 11;         // The distance between the midpoint of encoder 1 and 2 and 3
    final static double R = 3;          // Radius of the encoder wheel in cm
    final static double N = 8192;       // The encoder ticks per revolution, can get from the REV website


    // Temp variables that hold encoder pos between updates
    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentAuxPosition = 0;

    private int oldRightPosition = 0;
    private int oldLeftPosition = 0;
    private int oldAuxPosition = 0;


    public XyhVector START_POS = new XyhVector()
}
*/

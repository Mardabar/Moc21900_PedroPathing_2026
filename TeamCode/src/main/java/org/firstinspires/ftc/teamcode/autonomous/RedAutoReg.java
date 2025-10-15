package org.firstinspires.ftc.teamcode.autonomous;

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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "RedAutoReg", group = "autonomous")
public class RedAutoReg extends OpMode{

    // PEDROPATHING VARS

    private Follower fol;
    private Timer pathTimer, actionTimer, opmodeTimer; // Game timer
    private int pathState; // Current path #
    private int chainNum;

    // CAMERA VARS

    private Limelight3A cam;
    private VisionPortal visPort;
    private AprilTagProcessor apTag;
    private AprilTagDetection foundTag;
    private boolean tagFound;

    private final int GPP_ID = 21;
    private final int PGP_ID = 22;
    private final int PPG_ID = 23;

    // POSITIONS

    private final Pose startPose = new Pose(56, 8, Math.toRadians(0)); // POSITION

        // Obelisk #21
    private final Pose Ob21Grab1GP1 = new Pose(31, 35.5, Math.toRadians(0)); // POSITION
    private final Pose Ob21Grab1GP1CP = new Pose(56, 35.5, Math.toRadians(0)); // CONTROL POINT
    private final Pose Ob21Grab2P1 = new Pose(36, 59.8, Math.toRadians(0)); // POSITION
    private final Pose Ob21Grab2P1CP = new Pose(61, 59.8, Math.toRadians(0)); // CONTROL POINT

        // Obelisk #22


        // Obelisk #23


    // PID CONSTANTS & OTHER MOTORS

    private DcMotor ls;
    private DcMotor rs;
    private DcMotor intakeBelt;
    private DcMotor elbow;
    private PIDFController lsPID;
    private PIDFController rsPID;
    private PIDFController ibPID;
    private PIDFController elPID;
    private static double p = 0, i = 0, d = 0, f = 0;
    private static PIDFCoefficients coef;

    // Encoder resolution for launch motors is 28 PPR
    // Encoder resolution for intake motor is 384.5 PPR
    // Encoder resolution for elbow motor is 751.8 PPR

    private static double ticksInDeg = 0;
    private static double lsTarget;
    private static double rsTarget;
    private static double ibTarget;
    private static double elTarget;
    private double lsIntSum;
    private double rsIntSum;
    private double ibIntSum;
    private double elIntSum;
    private double lsLastErr;
    private double rsLastErr;
    private double ibLastErr;
    private double elLastErr;

    // PATH CHAINS

        // Obelisk #21
    private PathChain pathOb21Grab1GP1, pathOb21Grab2P1;
        // Obelisk #22
    private PathChain pathOb22Grab1G1;
        // Obelisk #23
    private PathChain pathOb23Grab1PP1;

    @Override
    public void init(){
        ls = hardwareMap.get(DcMotor.class, "ls");
        rs = hardwareMap.get(DcMotor.class, "rs");
        intakeBelt = hardwareMap.get(DcMotor.class, "intakeBelt");
        elbow = hardwareMap.get(DcMotor.class, "elbow");

        // PATH INIT

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        setPathState(0);

        // CAMERA INIT

        apTag = new AprilTagProcessor.Builder().build();
        apTag.setDecimation(2); // Trades some detection range for detection rate

        visPort = new VisionPortal.Builder()
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(apTag)
                .build();

        // PID INIT

        coef = new PIDFCoefficients(p, i, d, f);
        lsPID = new PIDFController(coef);
        rsPID = new PIDFController(coef);
        ibPID = new PIDFController(coef);
        elPID = new PIDFController(coef);
    }

    public void loop(){
        fol.update();
        autonomousPathUpdate();

        lsPID.setCoefficients(coef);
        rsPID.setCoefficients(coef);
        ibPID.setCoefficients(coef);
        elPID.setCoefficients(coef);

        double lsErr = lsTarget - ls.getCurrentPosition();
        double rsErr = rsTarget - rs.getCurrentPosition();
        double ibErr = ibTarget - intakeBelt.getCurrentPosition();
        double elErr = elTarget - elbow.getCurrentPosition();

        double lsD = (lsErr - lsLastErr) / opmodeTimer.getElapsedTimeSeconds();
        double rsD = (rsErr - rsLastErr) / opmodeTimer.getElapsedTimeSeconds();
        double ibD = (ibErr - ibLastErr) / opmodeTimer.getElapsedTimeSeconds();
        double elD = (elErr - elLastErr) / opmodeTimer.getElapsedTimeSeconds();

        lsIntSum = lsIntSum + (lsErr * opmodeTimer.getElapsedTimeSeconds());
        rsIntSum = rsIntSum + (rsErr * opmodeTimer.getElapsedTimeSeconds());
        ibIntSum = ibIntSum + (ibErr * opmodeTimer.getElapsedTimeSeconds());
        elIntSum = elIntSum + (elErr * opmodeTimer.getElapsedTimeSeconds());

        ls.setPower((p * lsErr) + (i * lsIntSum) + (d * lsD));
        rs.setPower((p * rsErr) + (i * rsIntSum) + (d * rsD));
        intakeBelt.setPower((p * ibErr) + (i * ibIntSum) + (d * ibD));
        elbow.setPower((p * elErr) + (i * elIntSum) + (d * elD));

        lsLastErr = lsErr;
        rsLastErr = rsErr;
        ibLastErr = ibErr;
        elLastErr = elErr;
    }

    public void buildPaths(int obNum){
        setChainNum(obNum);
        if (obNum == GPP_ID){

        }
        else if (obNum == PGP_ID){

        }
        else if (obNum == PPG_ID){

        }
    }

    public void autonomousPathUpdate(){
        if (!tagFound){
            List<AprilTagDetection> detections = apTag.getDetections();

            for (AprilTagDetection tag : detections){
                if (tag.metadata != null && (tag.id == GPP_ID || tag.id == PGP_ID || tag.id == PPG_ID)){
                    buildPaths(tag.id);
                    foundTag = tag;
                    break;
                }
            }
        }
        else if (chainNum == 21 && tagFound) {
            switch (pathState) {
                case 0:
                    break;

                case 1:
                    break;

                case 2:
                    if (!fol.isBusy()) {
                        setPathState(-1);
                    }
                    break;
            }
        }
        else if (chainNum == 22){

        }
        else if (chainNum == 23){

        }
    }

    private void setChainNum(int num){
        chainNum = num;
    }

    private void setPathState(int num){
        pathState = num;
        pathTimer.resetTimer();
    }

    private void setLsTarget(double num){
        lsTarget = num;
    }

    private void setRsTarget(double num){
        rsTarget = num;
    }

    private void setIbTarget(double num){
        ibTarget = num;
    }

    private void setElTarget(double num){
        elTarget = num;
    }
}
